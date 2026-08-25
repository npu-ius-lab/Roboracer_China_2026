#include "f1tenth_dynamic_mpcc/acados_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace mpcc = f1tenth_dynamic_mpcc;

namespace {
struct TimedState { double stamp; mpcc::State state; };

mpcc::State sample(const std::deque<TimedState>& history,double stamp) {
  auto upper=std::upper_bound(history.begin(),history.end(),stamp,
      [](double t,const TimedState& x){return t<x.stamp;});
  if(upper==history.begin())return upper->state;
  if(upper==history.end())return history.back().state;
  const auto& b=*upper;const auto& a=*std::prev(upper);
  const double z=(stamp-a.stamp)/std::max(b.stamp-a.stamp,1e-9);
  mpcc::State out=a.state+z*(b.state-a.state);
  out[2]=a.state[2]+z*mpcc::wrapAngle(b.state[2]-a.state[2]);
  return out;
}

double quantile(std::vector<double> values,double q) {
  if(values.empty())return 0.0;
  std::sort(values.begin(),values.end());
  const size_t i=std::min(values.size()-1,
      static_cast<size_t>(q*(values.size()-1)));
  return values[i];
}
}  // namespace

int main(int argc,char** argv) {
  if(argc!=5&&argc!=6)return 2;
  YAML::Node controller=YAML::LoadFile(argv[1]);controller["mode"]="race";
  const std::string variant=argc==6?argv[5]:"full";
  if(variant=="legacy"||variant=="cap_guard"){
    controller["cost"]["speed_prior_race"]=5.0;
    controller["cost"]["progress_reward_race"]=0.0;
    controller["cost"]["heading_race"]=3.0;
  }
  if(variant=="legacy"||variant=="simplified")
    controller["safety"]["dynamic_speed_cap_feasibility_guard"]=false;
  const YAML::Node vehicle_cfg=YAML::LoadFile(argv[2]);
  const auto vehicle=mpcc::VehicleParameters::fromYaml(vehicle_cfg,controller);
  std::shared_ptr<const f1tenth_residual_dynamics::ResidualModel> residual;
  const auto residual_cfg=controller["residual_dynamics"];
  if(residual_cfg&&residual_cfg["enabled"].as<bool>()){
    std::filesystem::path path=residual_cfg["model_path"].as<std::string>();
    if(path.is_relative())path=std::filesystem::path(argv[1]).parent_path().parent_path()/path;
    residual=std::make_shared<f1tenth_residual_dynamics::ResidualModel>(
      f1tenth_residual_dynamics::ResidualModel::load(path.string()));
  }
  YAML::Node speed=controller["speed_planning"];
  speed["wheelbase_m"]=vehicle.wheelbase;
  speed["max_steer_rad"]=vehicle.max_steer;
  speed["max_steer_rate_radps"]=vehicle.max_steer_rate;
  speed["max_accel_mps2"]=vehicle.max_accel;
  speed["max_decel_mps2"]=vehicle.max_decel;
  speed["lateral_accel_limit_mps2"]=vehicle.lateral_accel_limit;
  mpcc::PeriodicTrack track(argv[3],speed,vehicle);
  mpcc::DynamicBicycleModel plant_model(vehicle,5,residual);
  mpcc::CommandHistory commands(3.0);
  mpcc::LowLatencyPredictor predictor(plant_model,commands,0.005);
  mpcc::CommandManager manager(controller,vehicle.max_steer);
  mpcc::PredictiveSpeedLimiter limiter(
      controller["safety"]["predictive_speed_hold_s"].as<double>(),
      controller["safety"]["predictive_speed_recovery_mps2"].as<double>());
  mpcc::PurePursuitCandidate candidate(
      controller["pure_pursuit_candidate"],track,plant_model);
  mpcc::CandidateArbitrator arbitrator(
      controller["pure_pursuit_candidate"]["arbitration"]);
  mpcc::AcadosRuntimeSolver solver("mpcc",argv[4],track,vehicle,controller,residual);

  const auto g=track.geometry(0.0);mpcc::State truth=mpcc::State::Zero();
  truth<<g.x,g.y,g.yaw,0,0,0,0,0,0;
  std::deque<TimedState> truth_history{{-1.0,truth},{0.0,truth}};
  commands.push({-1.0,0,0,0,0});
  mpcc::Control fallback=mpcc::Control::Zero();
  double publisher_cap=3.0;
  double previous_steering=0.0;
  const double plant_dt=0.005;
  const double control_dt=1.0/controller["timing"]["control_rate_hz"].as<double>();
  const double publisher_dt=1.0/controller["publisher"]["rate_hz"].as<double>();
  const int control_stride=static_cast<int>(std::lround(control_dt/plant_dt));
  const int publisher_stride=static_cast<int>(std::lround(publisher_dt/plant_dt));
  const double duration=30.0;
  int failures=0,status4=0,solves=0,retries=0,takeovers=0,candidate_cycles=0;
  double max_abs_ec=0.0,min_margin=1e9,max_age=0.0;
  std::vector<double> solve_times,ages;

  for(int step=0;step<static_cast<int>(duration/plant_dt);++step){
    const double now=step*plant_dt;
    if(step%4==0)predictor.pushWheel(now,truth[3]);       // 50 Hz
    predictor.pushImu(now,truth[5]);                    // 200 Hz
    if(step%control_stride==0){                         // configured controller rate
      const double age=0.11+0.02*std::sin(0.7*now);     // median .11, p95 ~.13
      const double source=std::max(0.0,now-age);
      mpcc::State measured=sample(truth_history,source);
      const auto measured_projection=track.project(
          measured[0],measured[1],measured[8]);
      measured[8]=measured_projection.s;
      try{
        auto current=predictor.correctAndRepropagate(
            measured,source,now,0.30,fallback);
        auto committed=predictor.committedHorizon(current.state,now,fallback);
        auto projected=track.project(
            committed.state[0],committed.state[1],committed.state[8]);
        committed.state[7]=manager.published()[1];
        committed.state[8]=projected.s;
        // Reproduce the real failure mode: a latency/risk speed target drops
        // abruptly while the vehicle is already fast.  The command manager
        // decelerates continuously; the QP receives a reachable input bound.
        double requested=(now>=8.0&&now<10.0)?0.50:3.0;
        const double target=limiter.limit(requested,now);
        auto result=solver.solve(committed.state,target,1.5,1.0,control_dt);
        ++solves;solve_times.push_back(result.solve_time);ages.push_back(age);
        status4+=result.status==4;retries+=result.speed_retry;
        if(result.speed_retry)limiter.noteRisk(result.speed_cap_used,now);
        publisher_cap=limiter.limit(requested,now);
        const double safety_margin=controller["pure_pursuit_candidate"]
            ["boundary_margin_m"].as<double>();
        auto mpcc_safety=mpcc::evaluateTrajectorySafety(result.states,track,
            vehicle.body_width,safety_margin,.05);
        // Inject a bad MPCC horizon without perturbing the plant. This
        // exercises takeover and hysteretic release while acados keeps
        // solving on every control cycle.
        const bool injected=now>=12.0&&now<12.15;
        if(injected){
          auto unsafe_states=result.states;
          for(auto& x:unsafe_states){
            const auto p=track.project(x[0],x[1],x[8]);
            const auto geometry=track.geometry(p.s);
            const double offset=geometry.width_left-vehicle.body_width/2+.08;
            x[0]=geometry.x-std::sin(geometry.yaw)*offset;
            x[1]=geometry.y+std::cos(geometry.yaw)*offset;
          }
          mpcc_safety=mpcc::evaluateTrajectorySafety(unsafe_states,track,
              vehicle.body_width,safety_margin,.05);
        }
        mpcc::PurePursuitCandidateResult pp;
        if(injected||arbitrator.active())pp=candidate.generate(committed.state,target);
        const auto selection=arbitrator.choose(injected,mpcc_safety,pp.safety,now);
        takeovers+=selection.activated;
        candidate_cycles+=selection.use_candidate;
        if(selection.use_candidate&&pp.valid)
          manager.setCandidate(pp.command_speed,pp.command_steering,
                               pp.virtual_speed,now,"pure_pursuit_candidate");
        else if(result.success)manager.setSolution(result.states,result.controls,now);
        else{++failures;manager.noteFailure();}
      }catch(const std::exception& e){
        ++failures;manager.noteFailure();
        std::cerr<<"simulation predictor/solver exception: "<<e.what()<<'\n';
      }
    }
    if(step%publisher_stride==0){                       // 50 Hz hardware output
      const mpcc::StartupProfile race{
          mpcc::StartupMode::RACE,1.0,publisher_cap,1.5,1.0};
      auto out=manager.tick(now,race);
      const double rate=(out.steering-previous_steering)/publisher_dt;
      previous_steering=out.steering;
      commands.push({now,out.speed,out.steering,out.virtual_speed,rate});
      fallback<<out.speed,rate,out.virtual_speed;
    }
    const auto effective=commands.effectiveAt(
        now,vehicle.speed_dead_time,vehicle.steering_dead_time);
    mpcc::Control plant_u=fallback;
    if(effective)plant_u<<effective->speed,effective->steering_rate,
                         effective->virtual_speed;
    truth=plant_model.step(truth,plant_u,plant_dt);
    auto projection=track.project(truth[0],truth[1],truth[8]);truth[8]=projection.s;
    max_abs_ec=std::max(max_abs_ec,std::abs(projection.e_contour));
    const auto geometry=track.geometry(truth[8]);
    const double margin=std::min(
      geometry.width_left-vehicle.body_width/2-projection.e_contour,
      geometry.width_right-vehicle.body_width/2+projection.e_contour);
    min_margin=std::min(min_margin,margin);
    truth_history.push_back({now+plant_dt,truth});
    while(truth_history.size()>2&&truth_history[1].stamp<now-1.0)
      truth_history.pop_front();
  }
  max_age=ages.empty()?0:*std::max_element(ages.begin(),ages.end());
  std::cout<<std::fixed<<std::setprecision(6)
    <<"{\"variant\":\""<<variant<<"\",\"solves\":"<<solves<<",\"failures\":"<<failures
    <<",\"status4\":"<<status4<<",\"predictive_retries\":"<<retries
    <<",\"candidate_takeovers\":"<<takeovers
    <<",\"candidate_cycles\":"<<candidate_cycles
    <<",\"completed_laps\":"<<truth[8]/track.length()
    <<",\"max_abs_contour_error_m\":"<<max_abs_ec
    <<",\"minimum_track_margin_m\":"<<min_margin
    <<",\"pointlio_age_p95_s\":"<<quantile(ages,.95)
    <<",\"pointlio_age_max_s\":"<<max_age
    <<",\"solve_p95_ms\":"<<1000*quantile(solve_times,.95)
    <<",\"solve_max_ms\":"<<1000*quantile(solve_times,1.0)<<"}\n";
  return status4==0&&failures<10&&min_margin>0.0&&takeovers==1&&candidate_cycles>=10?0:20;
}
