#include "f1tenth_dynamic_mpcc/core.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace mpcc = f1tenth_dynamic_mpcc;

int main(int argc,char** argv) {
  if(argc!=4)return 2;
  const YAML::Node controller=YAML::LoadFile(argv[1]);
  const YAML::Node vehicle_config=YAML::LoadFile(argv[2]);
  const auto vehicle=mpcc::VehicleParameters::fromYaml(vehicle_config,controller);
  YAML::Node speed=controller["speed_planning"];
  speed["wheelbase_m"]=vehicle.wheelbase;
  speed["max_steer_rad"]=vehicle.max_steer;
  speed["max_steer_rate_radps"]=vehicle.max_steer_rate;
  speed["max_accel_mps2"]=vehicle.max_accel;
  speed["max_decel_mps2"]=vehicle.max_decel;
  speed["lateral_accel_limit_mps2"]=vehicle.lateral_accel_limit;
  mpcc::PeriodicTrack track(argv[3],speed,vehicle);
  mpcc::DynamicBicycleModel model(vehicle);
  mpcc::PurePursuitCandidate candidate(
      controller["pure_pursuit_candidate"],track,model,
      controller["publisher"]["acceleration_limit_mps2"].as<double>(),
      controller["publisher"]["deceleration_limit_mps2"].as<double>());
  mpcc::CandidateArbitrator arbitrator(
      controller["pure_pursuit_candidate"]["arbitration"]);

  // The visualized PP rollout must obey the exact publisher speed slew.  A
  // 5 m/s car cannot instantaneously become the requested 0.8 m/s recovery
  // command; otherwise the pink path is tighter than the physical vehicle.
  const auto speed_geometry=track.geometry(0.0);
  mpcc::State fast=mpcc::State::Zero();
  fast<<speed_geometry.x,speed_geometry.y,speed_geometry.yaw,
      5.0,0,0,0,0,0;
  const auto slew_candidate=candidate.generate(fast,.8,false,5.0);
  const double publisher_decel=controller["pure_pursuit_candidate"]
      ["emergency_deceleration_limit_mps2"]
      ?controller["pure_pursuit_candidate"]
         ["emergency_deceleration_limit_mps2"].as<double>()
      :controller["publisher"]["deceleration_limit_mps2"].as<double>();
  const double initial_candidate_speed=std::min(
      5.0,controller["pure_pursuit_candidate"]["max_speed_mps"].as<double>());
  if(!slew_candidate.valid||slew_candidate.command_speed>.800001||
     slew_candidate.controls.empty()||
     std::abs(slew_candidate.controls.front()[0]-
              (initial_candidate_speed-publisher_decel*.05))>1e-6)
    return 21;
  for(size_t i=1;i<slew_candidate.controls.size();++i)
    if(slew_candidate.controls[i-1][0]-slew_candidate.controls[i][0]>
       publisher_decel*.05+1e-9)return 22;

  // Checkpoint recovery is armed on every process start. It selects a safe
  // low-speed PP path for a displaced car, refuses deep/unsafe placement, and
  // hands back only after several consecutive safe MPCC horizons.
  mpcc::CheckpointRecoveryManager checkpoint(
      controller["pure_pursuit_candidate"]["checkpoint_recovery"]);
  mpcc::TrajectorySafety safe;
  safe.finite=true;safe.safe=true;safe.minimum_margin=.20;
  safe.initial_margin=.20;safe.final_margin=.20;safe.first_safe_time=0.0;
  mpcc::TrajectorySafety synthetic_unsafe=safe;
  synthetic_unsafe.safe=false;synthetic_unsafe.minimum_margin=-.20;
  auto checkpoint_selection=checkpoint.update(
      .30,.10,.15,safe,synthetic_unsafe);
  if(!checkpoint_selection.active||!checkpoint_selection.activated||
     !checkpoint_selection.use_candidate||checkpoint_selection.blocked)
    return 23;
  checkpoint.arm();
  mpcc::TrajectorySafety reentry=safe;reentry.safe=false;
  reentry.initial_margin=-.20;reentry.minimum_margin=-.20;
  reentry.final_margin=.10;reentry.first_safe_time=.30;
  checkpoint_selection=checkpoint.update(
      .40,.10,-.20,reentry,synthetic_unsafe);
  if(!checkpoint_selection.blocked||checkpoint_selection.use_candidate)
    return 24;
  checkpoint.arm();
  checkpoint_selection=checkpoint.update(
      .30,.10,.15,safe,synthetic_unsafe);
  bool released=false;
  const int release_cycles=controller["pure_pursuit_candidate"]
      ["checkpoint_recovery"]["release_safe_cycles"].as<int>();
  for(int i=0;i<release_cycles;++i){
    checkpoint_selection=checkpoint.update(.05,.05,.20,safe,safe);
    released=checkpoint_selection.released;
  }
  if(!released||checkpoint.armed()||checkpoint.active())return 25;

  const auto start=track.geometry(0.0);
  const double nx=-std::sin(start.yaw),ny=std::cos(start.yaw);
  mpcc::State state=mpcc::State::Zero();
  state<<start.x+0.08*nx,start.y+0.08*ny,start.yaw+0.06,
      1.2,0,0,0,0,0;

  // Fault injection: make the MPCC candidate cross the left control boundary.
  // Arbitration must select a safe, higher-margin PP rollout.
  auto initial_candidate=candidate.generate(state,3.0);
  if(!initial_candidate.valid||!initial_candidate.safety.safe)return 10;
  auto bad_mpcc_states=initial_candidate.states;
  for(auto& x:bad_mpcc_states){
    const auto p=track.project(x[0],x[1],x[8]);
    const auto g=track.geometry(p.s);
    const double offset=g.width_left-vehicle.body_width/2+0.08;
    x[0]=g.x-std::sin(g.yaw)*offset;
    x[1]=g.y+std::cos(g.yaw)*offset;
    x[8]=p.s;
  }
  const double boundary_margin=
      controller["pure_pursuit_candidate"]["boundary_margin_m"].as<double>();
  const auto unsafe=mpcc::evaluateTrajectorySafety(
      bad_mpcc_states,track,vehicle.body_width,boundary_margin,.05);
  auto selection=arbitrator.choose(true,unsafe,initial_candidate.safety,0.0);
  if(unsafe.safe||!selection.use_candidate||!selection.activated)return 11;
  mpcc::TrajectorySafety recovered=initial_candidate.safety;
  recovered.minimum_margin=.15;
  for(int i=1;i<=5;++i){
    selection=arbitrator.choose(false,recovered,initial_candidate.safety,.1*i);
    if(i<5&&!selection.use_candidate)return 13;
  }
  if(selection.use_candidate||!selection.released)return 14;

  // A shallow stage-zero excursion is recoverable: unlike the normal safety
  // rule, arbitration evaluates improvement, re-entry time and terminal
  // margin instead of rejecting the immutable initial sample.
  const auto recovery_geometry=track.geometry(2.0);
  const double recovery_nx=-std::sin(recovery_geometry.yaw);
  const double recovery_ny=std::cos(recovery_geometry.yaw);
  const double footprint=vehicle.body_width/2+boundary_margin;
  const double shallow_offset=-(recovery_geometry.width_right-footprint+.04);
  mpcc::State shallow=mpcc::State::Zero();
  shallow<<recovery_geometry.x+recovery_nx*shallow_offset,
      recovery_geometry.y+recovery_ny*shallow_offset,
      recovery_geometry.yaw,2.5,0,0,0,0,2.0;
  const auto recovery_candidate=candidate.generate(shallow,3.0);
  mpcc::TrajectorySafety rejected=recovery_candidate.safety;
  rejected.minimum_margin=recovery_candidate.safety.initial_margin-.10;
  rejected.final_margin=recovery_candidate.safety.initial_margin-.15;
  rejected.safe=false;
  mpcc::CandidateArbitrator recovery_arbitrator(
      controller["pure_pursuit_candidate"]["arbitration"]);
  const auto recovery_selection=recovery_arbitrator.choose(
      true,rejected,recovery_candidate.safety,0.0);
  std::cout<<"Recovery rollout initial="<<recovery_candidate.safety.initial_margin
      <<" minimum="<<recovery_candidate.safety.minimum_margin
      <<" final="<<recovery_candidate.safety.final_margin
      <<" reentry="<<recovery_candidate.safety.first_safe_time
      <<" speed="<<recovery_candidate.command_speed<<'\n';
  if(!recovery_candidate.valid||!recovery_selection.use_candidate||
     !recovery_selection.recovery)return 15;

  mpcc::State recovery_state=shallow;
  mpcc::Control recovery_control=mpcc::Control::Zero();
  double recovery_target=0.0,recovery_worst=1e9;
  for(int step=0;step<200;++step){
    if(step%5==0){
      const auto rollout=candidate.generate(recovery_state,3.0);
      recovery_control<<rollout.command_speed,
          mpcc::clamp((rollout.command_steering-recovery_target)/.05,-1.5,1.5),
          rollout.virtual_speed;
      recovery_target=rollout.command_steering;
    }
    recovery_state=model.step(recovery_state,recovery_control,.01);
    const auto projection=track.project(recovery_state[0],recovery_state[1],
                                        recovery_state[8]);
    recovery_state[8]=projection.s;
    const auto geometry=track.geometry(projection.s);
    const double margin=std::min(
        geometry.width_left-footprint-projection.e_contour,
        geometry.width_right-footprint+projection.e_contour);
    recovery_worst=std::min(recovery_worst,margin);
  }
  const auto recovered_state_safety=mpcc::evaluateTrajectorySafety(
      {recovery_state},track,vehicle.body_width,boundary_margin,.05);
  if(recovery_worst<-.065||recovered_state_safety.final_margin<.05)return 17;

  mpcc::TrajectorySafety deep=recovery_candidate.safety;
  deep.initial_margin=-.20;deep.minimum_margin=-.20;deep.final_margin=.08;
  deep.first_safe_time=.4;deep.safe=false;
  mpcc::CandidateArbitrator deep_arbitrator(
      controller["pure_pursuit_candidate"]["arbitration"]);
  if(deep_arbitrator.choose(true,rejected,deep,0.0).use_candidate)return 16;

  const double dt=.01,control_dt=.05,duration=45.0;
  mpcc::Control held=mpcc::Control::Zero();
  double previous_target=0,min_margin=1e9,max_error=0;
  int takeovers=1,unsafe_rollouts=0;
  for(int step=0;step<static_cast<int>(duration/dt);++step){
    if(step%5==0){
      auto rollout=candidate.generate(state,3.0);
      if(!rollout.valid)return 12;
      unsafe_rollouts+=!rollout.safety.safe;
      const double target=rollout.command_steering;
      held<<rollout.command_speed,
          mpcc::clamp((target-previous_target)/control_dt,
                      -1.5,1.5),rollout.virtual_speed;
      previous_target=target;
    }
    state=model.step(state,held,dt);
    auto projection=track.project(state[0],state[1],state[8]);
    state[8]=projection.s;
    const auto geometry=track.geometry(projection.s);
    const double margin=std::min(
        geometry.width_left-vehicle.body_width/2-projection.e_contour,
        geometry.width_right-vehicle.body_width/2+projection.e_contour);
    min_margin=std::min(min_margin,margin);
    max_error=std::max(max_error,std::abs(projection.e_contour));
  }
  std::cout<<std::fixed<<std::setprecision(4)
      <<"{\"fault_injection_takeovers\":"<<takeovers
      <<",\"candidate_unsafe_rollouts\":"<<unsafe_rollouts
      <<",\"recovery_worst_margin_m\":"<<recovery_worst
      <<",\"recovery_final_margin_m\":"<<recovered_state_safety.final_margin
      <<",\"completed_laps\":"<<state[8]/track.length()
      <<",\"minimum_physical_margin_m\":"<<min_margin
      <<",\"maximum_contour_error_m\":"<<max_error
      <<",\"final_speed_mps\":"<<state[3]<<"}\n";
  return min_margin>0.02&&state[8]/track.length()>1.0?0:20;
}
