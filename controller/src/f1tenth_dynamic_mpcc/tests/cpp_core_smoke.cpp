#include "f1tenth_dynamic_mpcc/core.hpp"
#include "f1tenth_dynamic_mpcc/stable_v5_tracking_gate.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>

namespace mpcc = f1tenth_dynamic_mpcc;

int main(int argc, char** argv) {
  if (argc != 4) return 2;
  const YAML::Node controller = YAML::LoadFile(argv[1]);
  const YAML::Node vehicle_config = YAML::LoadFile(argv[2]);
  const auto vehicle = mpcc::VehicleParameters::fromYaml(vehicle_config, controller);
  YAML::Node speed = controller["speed_planning"];
  speed["wheelbase_m"] = vehicle.wheelbase;
  speed["max_steer_rad"] = vehicle.max_steer;
  speed["max_steer_rate_radps"] = vehicle.max_steer_rate;
  speed["max_accel_mps2"] = vehicle.max_accel;
  speed["max_decel_mps2"] = vehicle.max_decel;
  speed["lateral_accel_limit_mps2"] = vehicle.lateral_accel_limit;
  mpcc::PeriodicTrack track(argv[3], speed, vehicle);
  if (!(track.length() > 20.0)) return 10;
  const auto seam0 = track.position(0.0);
  const auto seam1 = track.position(track.length());
  if (!(std::hypot(seam0[0] - seam1[0], seam0[1] - seam1[1]) < 1e-9)) return 11;
  const auto projection = track.project(seam0[0], seam0[1]);
  if (!(projection.distance < 1e-6)) return 12;

  const mpcc::PredictiveSlackPolicy slack_policy{0.08, 0.25, 0.8, 1.2};
  auto decision = mpcc::evaluateTrackSlack(
      mpcc::TrackSlackSummary{0.16, 0.02, 0.16, 12, -1},
      slack_policy, 2.2, false);
  if (!(decision.retry && !decision.reject && decision.speed_limited &&
        std::abs(decision.speed_cap - 1.76) < 1e-9)) return 15;
  decision = mpcc::evaluateTrackSlack(
      mpcc::TrackSlackSummary{0.16, 0.02, 0.16, 12, -1},
      slack_policy, 2.2, true);
  if (decision.retry || decision.reject ||
      decision.reason != "predictive_track_slowdown") return 16;
  decision = mpcc::evaluateTrackSlack(
      mpcc::TrackSlackSummary{0.10, 0.10, 0.10, 4, 1},
      slack_policy, 2.2, false);
  if (!decision.reject || decision.reason != "near_track_slack") return 17;
  decision = mpcc::evaluateTrackSlack(
      mpcc::TrackSlackSummary{0.30, 0.02, 0.30, 20, -1},
      slack_policy, 2.2, true);
  if (!decision.reject || decision.reason != "far_track_slack_hard") return 18;

  mpcc::PredictiveSpeedLimiter speed_limiter(2.0, 0.5);
  if (std::abs(speed_limiter.limit(3.0, 0.0) - 3.0) > 1e-12) return 19;
  speed_limiter.noteRisk(1.8, 1.0);
  if (std::abs(speed_limiter.limit(3.0, 2.9) - 1.8) > 1e-12) return 20;
  if (std::abs(speed_limiter.limit(3.0, 3.2) - 1.9) > 1e-12) return 21;
  speed_limiter.noteRisk(1.6, 3.3);
  if (std::abs(speed_limiter.limit(3.0, 5.2) - 1.6) > 1e-12) return 22;
  if (std::abs(speed_limiter.limit(3.0, 8.0) - 2.95) > 1e-12) return 23;
  if (std::abs(speed_limiter.limit(3.0, 8.2) - 3.0) > 1e-12 ||
      speed_limiter.active()) return 24;

  mpcc::DynamicBicycleModel model(vehicle);
  mpcc::State state = mpcc::State::Zero();
  mpcc::Control input; input << 1.0, 0.0, 1.0;
  const auto stepped = model.step(state, input, 0.01);
  if (!(stepped.allFinite() && stepped[3] > 0.0)) return 13;

  // Point-LIO may report lateral/yaw noise while the vehicle is stopped.
  // The controller state must satisfy the stationary Ackermann constraint,
  // preserve steering, and use hysteresis before returning to dynamics.
  mpcc::LowSpeedStateConditioner conditioner(
      controller["low_speed_state_conditioning"], vehicle.wheelbase);
  mpcc::State noisy_stop = mpcc::State::Zero();
  noisy_stop[3] = 0.01;
  noisy_stop[4] = -0.19;
  noisy_stop[5] = -0.20;
  noisy_stop[6] = 0.30;
  auto conditioned = conditioner.updateAndCondition(noisy_stop, 0.0);
  if (!conditioned.stationary || !conditioned.entered || conditioned.exited ||
      conditioned.dynamic_blend != 0.0 || conditioned.state[3] != 0.0 ||
      conditioned.state[4] != 0.0 || conditioned.state[5] != 0.0 ||
      conditioned.state[6] != noisy_stop[6]) return 36;
  conditioned = conditioner.updateAndCondition(noisy_stop, 0.15);
  if (!conditioned.stationary || conditioned.entered || conditioned.exited)
    return 37;
  conditioned = conditioner.updateAndCondition(noisy_stop, 0.30);
  if (conditioned.stationary || !conditioned.exited ||
      std::abs(conditioned.state[4]) > 1e-12 ||
      std::abs(conditioned.state[5] -
        conditioned.state[3] * std::tan(conditioned.state[6]) /
          vehicle.wheelbase) > 1e-12) return 38;
  mpcc::State dynamic_state = noisy_stop;
  dynamic_state[3] = 0.50;
  conditioned = conditioner.updateAndCondition(dynamic_state, 0.50);
  if (conditioned.dynamic_blend != 1.0 ||
      std::abs(conditioned.state[4] - dynamic_state[4]) > 1e-12 ||
      std::abs(conditioned.state[5] - dynamic_state[5]) > 1e-12) return 39;

  // The safety candidate is independent of acados, but is rolled out through
  // the exact same vehicle model and checked against the exact same track.
  mpcc::State candidate_state=mpcc::State::Zero();
  const auto candidate_start=track.geometry(0.5);
  candidate_state<<candidate_start.x,candidate_start.y,candidate_start.yaw,
      1.5,0,0,0,0,0.5;
  mpcc::PurePursuitCandidate pp(controller["pure_pursuit_candidate"],track,model);
  const auto pp_result=pp.generate(candidate_state,3.0);
  if(!pp_result.valid||!pp_result.safety.safe||pp_result.states.size()!=26||
     !(pp_result.command_speed>0&&std::abs(pp_result.command_steering)<=vehicle.max_steer))return 31;
  mpcc::State high_error_candidate_state=candidate_state;
  high_error_candidate_state[2]=mpcc::wrapAngle(candidate_state[2]+0.69);
  const auto high_error_pp=pp.generate(high_error_candidate_state,3.0);
  if(!high_error_pp.valid||high_error_pp.command_speed<1.20-1e-12)return 40;
  const auto upstream_limited_pp=pp.generate(high_error_candidate_state,0.35);
  if(!upstream_limited_pp.valid||upstream_limited_pp.command_speed>0.35+1e-12)
    return 41;
  auto unsafe_states=pp_result.states;
  for(auto& x:unsafe_states)x[1]+=100.0;
  const auto unsafe=mpcc::evaluateTrajectorySafety(unsafe_states,track,
      vehicle.body_width,controller["pure_pursuit_candidate"]["boundary_margin_m"].as<double>(),.05);
  if(unsafe.safe||!(unsafe.minimum_margin<0)||unsafe.first_violation_time<0)return 32;
  mpcc::TrajectorySafety risky;risky.safe=false;risky.minimum_margin=-.08;
  mpcc::TrajectorySafety safe_trajectory;risky.finite=true;
  safe_trajectory.finite=true;safe_trajectory.safe=true;safe_trajectory.minimum_margin=.12;
  mpcc::CandidateArbitrator arbitrator(
      controller["pure_pursuit_candidate"]["arbitration"]);
  auto selection=arbitrator.choose(true,risky,safe_trajectory,1.0);
  if(!selection.use_candidate||!selection.activated)return 33;
  selection=arbitrator.choose(false,safe_trajectory,safe_trajectory,1.2);
  if(!selection.use_candidate||selection.released)return 34;

  mpcc::CommandManager candidate_manager(controller,vehicle.max_steer);
  candidate_manager.setCandidate(1.0,.12,1.0,0.0,"pure_pursuit_candidate");
  const mpcc::StartupProfile candidate_race{mpcc::StartupMode::RACE,1,3,1.5,1};
  const auto candidate_command=candidate_manager.tick(0.0,candidate_race);
  if(candidate_command.source!="pure_pursuit_candidate")return 35;

  // Optional dual-rate braking must preserve normal MPCC slew while allowing
  // only PP/checkpoint safety sources to use the stronger emergency rate.
  if(controller["publisher"]["emergency_deceleration_limit_mps2"]){
    const double normal_decel=controller["publisher"]
      ["deceleration_limit_mps2"].as<double>();
    const double emergency_decel=controller["publisher"]
      ["emergency_deceleration_limit_mps2"].as<double>();
    auto warm_manager=[&](mpcc::CommandManager& manager,
                          const std::string& source){
      double now=0.0;
      for(int i=0;i<150;++i){
        manager.setCandidate(3.0,0.0,3.0,now,source);
        manager.tick(now,candidate_race);now+=0.02;
      }
      return now;
    };
    mpcc::CommandManager normal_manager(controller,vehicle.max_steer);
    const double normal_now=warm_manager(normal_manager,"mpcc");
    const double normal_before=normal_manager.published()[0];
    normal_manager.setCandidate(0.0,0.0,0.0,normal_now,"mpcc");
    const auto normal_after=normal_manager.tick(normal_now,candidate_race);
    if(std::abs((normal_before-normal_after.speed)-normal_decel*.02)>1e-9)
      return 42;
    mpcc::CommandManager emergency_manager(controller,vehicle.max_steer);
    const double emergency_now=warm_manager(emergency_manager,"mpcc");
    const double emergency_before=emergency_manager.published()[0];
    emergency_manager.setCandidate(0.0,0.0,0.0,emergency_now,
                                   "pure_pursuit_candidate");
    const auto emergency_after=emergency_manager.tick(
      emergency_now,candidate_race);
    if(std::abs((emergency_before-emergency_after.speed)-emergency_decel*.02)>1e-9)
      return 43;

    // A predictive-risk MPCC solution keeps the normal "mpcc" source name,
    // so its explicit safety flag must select the emergency slew as well.
    std::vector<mpcc::State> risk_states(3,mpcc::State::Zero());
    std::vector<mpcc::Control> risk_controls(2,mpcc::Control::Zero());
    risk_controls[0][0]=0.0;
    risk_states[1][7]=0.0;
    emergency_manager.setSolution(risk_states,risk_controls,
                                  emergency_now+.02,true);
    const double risk_before=emergency_manager.published()[0];
    const auto risk_after=emergency_manager.tick(emergency_now+.02,
                                                 candidate_race);
    if(std::abs((risk_before-risk_after.speed)-emergency_decel*.02)>1e-9)
      return 44;

    // A first solver fault can still publish the shifted solution; it must
    // already use the emergency deceleration rather than waiting for fallback.
    emergency_manager.setSolution(risk_states,risk_controls,
                                  emergency_now+.04,false);
    emergency_manager.noteFailure();
    const double fault_before=emergency_manager.published()[0];
    const auto fault_after=emergency_manager.tick(emergency_now+.04,
                                                  candidate_race);
    if(std::abs((fault_before-fault_after.speed)-emergency_decel*.02)>1e-9)
      return 45;
  }

  mpcc::CommandHistory commands;
  commands.push({0.0, 0.0, 0.0, 0.0, 0.0});
  commands.push({1.0, 1.0, 0.1, 1.0, 0.2});
  const auto delayed = commands.effectiveAt(1.10, vehicle.speed_dead_time);
  if (!(delayed && delayed->speed == 0.0 &&
        std::abs(delayed->steering - 0.1) < 1e-12)) return 14;
  const auto released = commands.effectiveAt(1.13, vehicle.speed_dead_time);
  if (!(released && released->speed == 1.0)) return 25;
  const auto steering_held = commands.effectiveAt(
      1.019, vehicle.speed_dead_time, 0.02);
  if (!(steering_held && steering_held->steering == 0.0)) return 36;
  const auto steering_released = commands.effectiveAt(
      1.020, vehicle.speed_dead_time, 0.02);
  if (!(steering_released &&
        std::abs(steering_released->steering - 0.1) < 1e-12)) return 37;

  // Regression test for the real boundary-departure failure: once repeated
  // solution rejection enters SAFE_DECEL, keep the corrective steering while
  // braking, then recenter only after the command speed is low.
  mpcc::CommandManager manager(controller,vehicle.max_steer);
  std::vector<mpcc::State> solution_states(3,mpcc::State::Zero());
  std::vector<mpcc::Control> solution_controls(2,mpcc::Control::Zero());
  solution_states[1][7]=0.20;solution_states[2][7]=0.20;
  solution_controls[0]<<3.0,0.0,3.0;solution_controls[1]<<3.0,0.0,3.0;
  mpcc::StartupProfile race{mpcc::StartupMode::RACE,1.0,3.0,1.5,1.0};
  double command_time=0.0;
  for(int i=0;i<40;++i){
    manager.setSolution(solution_states,solution_controls,command_time);
    manager.tick(command_time,race);command_time+=0.05;
  }
  manager.noteFailure();manager.noteFailure();
  mpcc::StartupProfile safe{mpcc::StartupMode::SAFE_DECEL,0.0,3.0,0.75,0.0,
                            std::numeric_limits<double>::infinity()};
  const auto before_fallback=manager.published();
  const auto held=manager.tick(command_time,safe);command_time+=0.05;
  if(held.source!="safe_decel_hold_steering"||
     std::abs(held.steering-before_fallback[1])>1e-12||
     !(held.speed<before_fallback[0]))return 29;
  mpcc::PublishedCommand recentered=held;
  for(int i=0;i<20;++i){recentered=manager.tick(command_time,safe);command_time+=0.05;}
  if(!(std::abs(recentered.steering)<std::abs(held.steering)))return 30;
  std::cout<<"Fallback hold-steering regression passed\n";

  mpcc::State lag = mpcc::State::Zero();
  const double lag_dt = 0.0005;
  mpcc::Control lag_input;lag_input<<0.4,0.0,0.4;
  for(double t=0.0;t<vehicle.speed_tau-0.5*lag_dt;t+=lag_dt)
    lag=model.step(lag,lag_input,lag_dt);
  if(std::abs(lag[3]-0.4*(1.0-std::exp(-1.0)))>0.01) return 26;
  // At vx=2.36 m/s, a 0.50 m/s hard command cap conflicts with the
  // -4 m/s^2 acceleration constraint and tau=0.20 s.  Keep the target at
  // 0.50, but expose at least 1.58 m/s to the first QP input bound.
  const double feasible=mpcc::dynamicallyFeasibleSpeedCap(
      2.36,0.50,vehicle.speed_gain,vehicle.speed_tau,vehicle.max_decel);
  if(std::abs(feasible-1.58)>1e-9) return 27;

  // StableV5 earns only the speed above 3.2 m/s as lateral/heading/margin
  // quality improves.  The gate is continuous and leaves ordinary caps alone.
  const auto tracking_gate_config=YAML::Load(R"(
enabled: true
base_speed_mps: 3.2
lateral_full_speed_m: 0.10
lateral_base_speed_m: 0.30
heading_full_speed_rad: 0.04
heading_base_speed_rad: 0.12
margin_base_speed_m: 0.12
margin_full_speed_m: 0.30
)");
  mpcc::TrackingQualitySpeedGate tracking_gate(tracking_gate_config);
  const auto full_speed=tracking_gate.limit(5.0,0.05,0.02,0.40);
  if(std::abs(full_speed.speed_cap-5.0)>1e-12||
     std::abs(full_speed.scale-1.0)>1e-12)return 44;
  const auto lateral_mid=tracking_gate.limit(5.0,0.20,0.02,0.40);
  if(std::abs(lateral_mid.scale-0.5)>1e-12||
     std::abs(lateral_mid.speed_cap-4.10)>1e-12)return 45;
  const auto heading_limited=tracking_gate.limit(5.0,0.05,0.12,0.40);
  if(std::abs(heading_limited.speed_cap-3.2)>1e-12)return 46;
  const auto margin_limited=tracking_gate.limit(5.0,0.05,0.02,0.12);
  if(std::abs(margin_limited.speed_cap-3.2)>1e-12)return 47;
  const auto ordinary=tracking_gate.limit(3.0,0.50,0.50,-0.20);
  if(std::abs(ordinary.speed_cap-3.0)>1e-12)return 48;

  // A raceline-local limit must override the controller-wide maximum in the
  // same C++ track implementation used by the hardware node.
  const std::string limited_path="/tmp/f1tenth_limited_track_smoke.csv";
  {
    std::ifstream in(argv[3]);std::ofstream out(limited_path);
    std::string row;std::getline(in,row);
    std::vector<std::string> header;std::string item;std::stringstream header_stream(row);
    while(std::getline(header_stream,item,','))header.push_back(item);
    auto limit_it=std::find(header.begin(),header.end(),"speed_limit_mps");
    const bool has_limit=limit_it!=header.end();
    const std::size_t limit_index=has_limit
      ?static_cast<std::size_t>(limit_it-header.begin()):header.size();
    if(has_limit)out<<row<<'\n';else out<<row<<",speed_limit_mps\n";
    while(std::getline(in,row))if(!row.empty()){
      if(!has_limit){out<<row<<",0.9\n";continue;}
      std::vector<std::string> columns;std::stringstream stream(row);
      while(std::getline(stream,item,','))columns.push_back(item);
      columns.at(limit_index)="0.9";
      for(std::size_t i=0;i<columns.size();++i)
        out<<(i?",":"")<<columns[i];
      out<<'\n';
    }
  }
  mpcc::PeriodicTrack limited_track(limited_path,speed,vehicle);
  for(const double planned_speed:limited_track.speedNodes())
    if(planned_speed>0.9+1e-9)return 42;
  for(int i=0;i<20000;++i)
    if(limited_track.speedPrior(limited_track.length()*i/20000.0)>0.9+1e-9)
      return 43;
  std::remove(limited_path.c_str());

  // The C++ race track also accepts the minimal direct interface without
  // independent psi/kappa/vx/ax columns.
  const std::string minimal_path="/tmp/f1tenth_minimal_track_smoke.csv";
  {
    std::ifstream in(argv[3]);std::ofstream out(minimal_path);
    std::string row;std::getline(in,row);
    std::vector<std::string> header;std::string item;std::stringstream header_stream(row);
    while(std::getline(header_stream,item,',')){
      if(!item.empty()&&item.back()=='\r')item.pop_back();
      header.push_back(item);
    }
    auto column=[&](const std::string& name){
      return static_cast<std::size_t>(std::find(header.begin(),header.end(),name)-header.begin());
    };
    const auto is=column("s_m"),ix=column("x_m"),iy=column("y_m");
    const auto ir=column("w_tr_right_m"),il=column("w_tr_left_m");
    out<<"s_m,x_m,y_m,w_tr_right_m,w_tr_left_m\n";
    while(std::getline(in,row)){
      std::vector<std::string> columns;std::stringstream stream(row);
      while(std::getline(stream,item,','))columns.push_back(item);
      if(columns.size()==header.size())out<<columns[is]<<','<<columns[ix]<<','
        <<columns[iy]<<','<<columns[ir]<<','<<columns[il]<<'\n';
    }
  }
  mpcc::PeriodicTrack minimal_track(minimal_path,speed,vehicle);
  if(!(minimal_track.length()>20.0&&
       std::isfinite(minimal_track.curvature(0.3))))return 28;
  std::cout << "C++ core smoke test passed, track length=" << track.length() << '\n';
  return 0;
}
