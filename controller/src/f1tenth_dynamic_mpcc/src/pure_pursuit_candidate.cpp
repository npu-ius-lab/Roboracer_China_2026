#include "f1tenth_dynamic_mpcc/core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace f1tenth_dynamic_mpcc {
namespace {
double required(const YAML::Node& node,const char* key){
  if(!node[key])throw std::runtime_error(std::string("missing Pure Pursuit candidate key: ")+key);
  return node[key].as<double>();
}
template<typename T>T value(const YAML::Node& node,const char* key,const T& fallback){
  return node[key]?node[key].as<T>():fallback;
}
double errorScale(double x,double soft,double hard,double minimum){
  x=std::abs(x);if(x<=soft)return 1.0;if(x>=hard)return minimum;
  return 1-(1-minimum)*(x-soft)/(hard-soft);
}
}  // namespace

TrajectorySafety evaluateTrajectorySafety(
    const std::vector<State>& states,const PeriodicTrack& track,
    double body_width,double boundary_margin,double dt,double maximum_time){
  if(body_width<=0||boundary_margin<0||dt<=0||maximum_time<0)
    throw std::invalid_argument("invalid trajectory safety parameters");
  TrajectorySafety out;const double footprint=body_width/2+boundary_margin;
  std::optional<double> guess;
  for(size_t i=0;i<states.size();++i){
    if(i*dt>maximum_time+1e-9)break;
    const auto& x=states[i];
    if(!x.allFinite()){out.finite=false;out.safe=false;return out;}
    const auto projection=track.project(x[0],x[1],guess);guess=projection.s;
    const auto geometry=track.geometry(projection.s);
    const double margin=std::min(
        geometry.width_left-footprint-projection.e_contour,
        geometry.width_right-footprint+projection.e_contour);
    if(i==0)out.initial_margin=margin;
    out.final_margin=margin;
    if(margin>=0&&out.first_safe_time<0)out.first_safe_time=i*dt;
    if(margin<out.minimum_margin){out.minimum_margin=margin;
      out.minimum_margin_stage=static_cast<int>(i);}
    if(margin<0){out.maximum_violation=std::max(out.maximum_violation,-margin);
      if(out.first_violation_time<0)out.first_violation_time=i*dt;}
  }
  out.safe=out.finite&&!states.empty()&&out.minimum_margin>=0;return out;
}

PurePursuitCandidate::PurePursuitCandidate(
    const YAML::Node& c,const PeriodicTrack& track,DynamicBicycleModel model,
    double command_acceleration_limit,double command_deceleration_limit)
    :track_(track),model_(std::move(model)){
  enabled_=value(c,"enabled",true);horizon_s_=required(c,"horizon_s");
  dt_=required(c,"dt_s");trigger_slack_=required(c,"trigger_track_slack_m");
  boundary_margin_=required(c,"boundary_margin_m");
  lookahead_base_=required(c,"lookahead_base_m");
  lookahead_speed_gain_=required(c,"lookahead_speed_gain_s");
  lookahead_lateral_gain_=required(c,"lookahead_lateral_gain");
  lookahead_min_=required(c,"lookahead_min_m");lookahead_max_=required(c,"lookahead_max_m");
  steering_delay_=required(c,"steering_delay_s");max_speed_=required(c,"max_speed_mps");
  recovery_speed_min_=required(c,"recovery_speed_min_mps");
  recovery_speed_max_=required(c,"recovery_speed_max_mps");
  recovery_depth_=required(c,"recovery_full_slowdown_depth_m");
  max_steering_rate_=required(c,"max_steering_rate_radps");
  lateral_accel_limit_=required(c,"lateral_accel_limit_mps2");
  curvature_preview_=required(c,"curvature_preview_m");
  lateral_soft_=required(c,"lateral_error_soft_m");lateral_hard_=required(c,"lateral_error_hard_m");
  heading_soft_=required(c,"heading_error_soft_rad");heading_hard_=required(c,"heading_error_hard_rad");
  minimum_speed_scale_=required(c,"minimum_speed_scale");
  command_acceleration_limit_=command_acceleration_limit;
  command_deceleration_limit_=value(
      c,"emergency_deceleration_limit_mps2",command_deceleration_limit);
  if(horizon_s_<=0||dt_<=0||trigger_slack_<0||boundary_margin_<0||
     lookahead_min_<=0||lookahead_max_<lookahead_min_||steering_delay_<0||
     max_speed_<=0||recovery_speed_min_<0||
     recovery_speed_max_<recovery_speed_min_||recovery_depth_<=0||
     max_steering_rate_<=0||
     lateral_accel_limit_<=0||curvature_preview_<=0||
     lateral_hard_<=lateral_soft_||heading_hard_<=heading_soft_||
     minimum_speed_scale_<0||minimum_speed_scale_>1||
     command_acceleration_limit_<=0||command_deceleration_limit_<=0)
    throw std::runtime_error("invalid Pure Pursuit candidate configuration");
}

std::pair<double,double> PurePursuitCandidate::command(
    const State& state,double requested_cap,double& lookahead,
    double& lateral,double& heading,bool out_of_bounds_recovery)const{
  const auto projection=track_.project(state[0],state[1],state[8]);
  lateral=projection.e_contour;heading=wrapAngle(state[2]-projection.psi_ref);
  lookahead=clamp(lookahead_base_+lookahead_speed_gain_*std::max(state[3],0.0)+
      lookahead_lateral_gain_*std::abs(lateral),lookahead_min_,lookahead_max_);
  const auto target=track_.position(projection.s+
      std::max(state[3],0.0)*steering_delay_+lookahead);
  const double dx=target[0]-state[0],dy=target[1]-state[1];
  const double alpha=wrapAngle(std::atan2(dy,dx)-state[2]);
  double steering=std::atan2(2*model_.parameters().wheelbase*std::sin(alpha),
                             std::max(std::hypot(dx,dy),0.05));
  steering=clamp(steering,-model_.parameters().max_steer,model_.parameters().max_steer);
  double kappa=0;for(int i=0;i<=8;++i)kappa=std::max(kappa,
      std::abs(track_.curvature(projection.s+curvature_preview_*i/8.0)));
  const double curve_cap=kappa<1e-4?max_speed_:std::sqrt(lateral_accel_limit_/kappa);
  const double scale=std::min(
      errorScale(lateral,lateral_soft_,lateral_hard_,minimum_speed_scale_),
      errorScale(heading,heading_soft_,heading_hard_,minimum_speed_scale_));
  const auto geometry=track_.geometry(projection.s);
  const double footprint=model_.parameters().body_width/2+boundary_margin_;
  const double margin=std::min(geometry.width_left-footprint-lateral,
                               geometry.width_right-footprint+lateral);
  double speed=std::min({requested_cap,max_speed_,track_.speedPrior(projection.s),curve_cap})*scale;
  if(margin<0){
    const double severity=clamp(-margin/recovery_depth_,0.0,1.0);
    const double recovery_cap=recovery_speed_max_-
        severity*(recovery_speed_max_-recovery_speed_min_);
    speed=std::min(speed,recovery_cap);
  }
  // The candidate is a steering-based recovery controller, not a stop
  // fallback.  Keep enough forward motion for the front axle to generate a
  // useful lateral response, while never overriding an upstream safety cap.
  const double recovery_floor=out_of_bounds_recovery
      ?std::min({requested_cap,recovery_speed_min_,std::max(curve_cap,0.0)})
      :std::min(requested_cap,recovery_speed_min_);
  speed=std::max(speed,recovery_floor);
  return{std::max(speed,0.0),steering};
}

PurePursuitCandidateResult PurePursuitCandidate::generate(
    const State& initial,double requested_cap,
    bool out_of_bounds_recovery,double initial_command_speed)const{
  PurePursuitCandidateResult out;if(!enabled_||!initial.allFinite())return out;
  State state=initial;out.states.push_back(state);
  double applied_speed=std::isfinite(initial_command_speed)
      ?clamp(initial_command_speed,0.0,max_speed_)
      :clamp(initial[3],0.0,max_speed_);
  const int steps=std::max(1,static_cast<int>(std::ceil(horizon_s_/dt_)));
  for(int i=0;i<steps;++i){
    double lookahead=0,lateral=0,heading=0;
    const auto target=command(state,requested_cap,lookahead,lateral,heading,
                              out_of_bounds_recovery);
    const double rate=clamp((target.second-state[7])/dt_,
                            -max_steering_rate_,max_steering_rate_);
    const double speed_change=clamp(
        target.first-applied_speed,
        -command_deceleration_limit_*dt_,command_acceleration_limit_*dt_);
    applied_speed=clamp(applied_speed+speed_change,0.0,max_speed_);
    Control u;u<<applied_speed,rate,std::max(target.first,0.2);
    if(i==0){out.command_speed=target.first;out.command_steering=target.second;
      out.virtual_speed=u[2];out.lookahead=lookahead;
      out.lateral_error=lateral;out.heading_error=heading;}
    out.controls.push_back(u);state=model_.step(state,u,dt_);
    const auto projection=track_.project(state[0],state[1],state[8]);state[8]=projection.s;
    out.states.push_back(state);
  }
  out.safety=evaluateTrajectorySafety(out.states,track_,model_.parameters().body_width,
                                       boundary_margin_,dt_);
  out.valid=out.safety.finite&&!out.controls.empty();return out;
}

CheckpointRecoveryManager::CheckpointRecoveryManager(const YAML::Node& c){
  if(!c||!c.IsMap()){enabled_=false;return;}
  enabled_=value(c,"enabled",false);
  speed_cap_=value(c,"speed_cap_mps",0.8);
  activation_lateral_=value(c,"activation_lateral_error_m",0.20);
  activation_heading_=value(c,"activation_heading_error_rad",0.25);
  maximum_outside_margin_=value(c,"maximum_outside_margin_m",0.12);
  maximum_worsening_=value(c,"candidate_maximum_worsening_m",0.02);
  reentry_time_=value(c,"candidate_reentry_time_s",0.60);
  final_margin_=value(c,"candidate_final_margin_m",0.05);
  release_lateral_=value(c,"release_lateral_error_m",0.10);
  release_heading_=value(c,"release_heading_error_rad",0.12);
  release_mpcc_margin_=value(c,"release_mpcc_margin_m",0.08);
  release_safe_cycles_=value(c,"release_safe_cycles",8);
  if(speed_cap_<=0||activation_lateral_<=release_lateral_||
     activation_heading_<=release_heading_||maximum_outside_margin_<0||
     maximum_worsening_<0||reentry_time_<=0||final_margin_<0||
     release_lateral_<0||release_heading_<0||release_mpcc_margin_<0||
     release_safe_cycles_<1)
    throw std::runtime_error("invalid checkpoint recovery configuration");
  arm();
}

void CheckpointRecoveryManager::arm(){
  armed_=enabled_;active_=false;safe_cycles_=0;
}

void CheckpointRecoveryManager::disarm(){
  armed_=false;active_=false;safe_cycles_=0;
}

bool CheckpointRecoveryManager::candidateAcceptable(
    double current_margin,const TrajectorySafety& candidate)const{
  if(!candidate.finite)return false;
  if(candidate.safe&&candidate.minimum_margin>=0.0)return true;
  return current_margin>=-maximum_outside_margin_&&
      candidate.initial_margin>=-maximum_outside_margin_&&
      candidate.minimum_margin>=candidate.initial_margin-maximum_worsening_&&
      candidate.first_safe_time>=0.0&&candidate.first_safe_time<=reentry_time_&&
      candidate.final_margin>=final_margin_;
}

CheckpointRecoveryDecision CheckpointRecoveryManager::update(
    double lateral_error,double heading_error,double current_margin,
    const TrajectorySafety& candidate,const TrajectorySafety& mpcc){
  CheckpointRecoveryDecision out;
  if(!enabled_||!armed_){out.reason="disabled_or_disarmed";return out;}
  const bool aligned=std::abs(lateral_error)<=release_lateral_&&
      std::abs(heading_error)<=release_heading_;
  const bool mpcc_ready=mpcc.finite&&mpcc.safe&&
      mpcc.minimum_margin>=release_mpcc_margin_;
  safe_cycles_=(aligned&&mpcc_ready)?safe_cycles_+1:0;
  if(safe_cycles_>=release_safe_cycles_){
    disarm();out.released=true;out.reason="checkpoint_recovery_complete";
    return out;
  }
  const bool needs_recovery=std::abs(lateral_error)>=activation_lateral_||
      std::abs(heading_error)>=activation_heading_||active_;
  if(!needs_recovery&&mpcc_ready){
    disarm();out.released=true;out.reason="checkpoint_already_aligned";
    return out;
  }
  if(!active_){active_=true;out.activated=true;}
  out.active=true;
  out.use_candidate=candidateAcceptable(current_margin,candidate);
  out.blocked=!out.use_candidate;
  out.reason=out.use_candidate?"checkpoint_pure_pursuit":
      (current_margin<-maximum_outside_margin_?
       "checkpoint_pose_outside_safe_recovery":"checkpoint_candidate_unsafe");
  return out;
}

CandidateArbitrator::CandidateArbitrator(const YAML::Node& c){
  enabled_=value(c,"enabled",true);minimum_improvement_=required(c,"minimum_margin_improvement_m");
  minimum_candidate_margin_=required(c,"minimum_candidate_margin_m");
  minimum_candidate_margin_tolerance_=required(
      c,"minimum_candidate_margin_tolerance_m");
  recovery_minimum_initial_margin_=required(c,"recovery_minimum_initial_margin_m");
  recovery_maximum_worsening_=required(c,"recovery_maximum_worsening_m");
  recovery_reentry_time_=required(c,"recovery_reentry_time_s");
  recovery_final_margin_=required(c,"recovery_final_margin_m");
  recovery_minimum_progress_=required(c,"recovery_minimum_progress_m");
  hold_s_=required(c,"minimum_hold_s");release_mpcc_margin_=required(c,"release_mpcc_margin_m");
  release_safe_cycles_=c["release_safe_cycles"].as<int>();
  if(minimum_improvement_<0||minimum_candidate_margin_<0||
     minimum_candidate_margin_tolerance_<0||
     minimum_candidate_margin_tolerance_>minimum_candidate_margin_||
     recovery_minimum_initial_margin_>=0||recovery_maximum_worsening_<0||
     recovery_reentry_time_<=0||recovery_final_margin_<0||
     recovery_minimum_progress_<=0||hold_s_<0||
     release_mpcc_margin_<0||release_safe_cycles_<1)
    throw std::runtime_error("invalid candidate arbitration configuration");
}
void CandidateArbitrator::reset(){active_=false;active_recovery_=false;hold_until_=0;safe_cycles_=0;}
CandidateSelection CandidateArbitrator::choose(
    bool risk,const TrajectorySafety& mpcc,const TrajectorySafety& candidate,double now){
  CandidateSelection out;
  out.improvement=std::isfinite(candidate.minimum_margin)&&
      std::isfinite(mpcc.minimum_margin)
      ?candidate.minimum_margin-mpcc.minimum_margin
      :-std::numeric_limits<double>::infinity();
  const bool candidate_safe=candidate.finite&&candidate.safe&&
      candidate.minimum_margin>=minimum_candidate_margin_-
                                minimum_candidate_margin_tolerance_;
  const bool better=candidate_safe&&out.improvement>=minimum_improvement_;
  const double recovery_progress=candidate.final_margin-candidate.initial_margin;
  const bool recovery=candidate.finite&&
      std::isfinite(candidate.initial_margin)&&std::isfinite(candidate.final_margin)&&
      candidate.initial_margin>=recovery_minimum_initial_margin_&&
      candidate.initial_margin<minimum_candidate_margin_&&
      candidate.minimum_margin>=candidate.initial_margin-recovery_maximum_worsening_&&
      candidate.first_safe_time>=0&&candidate.first_safe_time<=recovery_reentry_time_&&
      candidate.final_margin>=recovery_final_margin_&&
      recovery_progress>=recovery_minimum_progress_;
  const bool acceptable=better||recovery;
  if(!enabled_){out.reason="disabled";return out;}
  if(!active_){
    if(risk&&acceptable){active_=true;active_recovery_=recovery;
      hold_until_=now+hold_s_;safe_cycles_=0;out.activated=true;
      out.reason=recovery?"shallow_boundary_recovery":"prediction_risk_candidate_safer";}
    else out.reason=!risk?"no_prediction_risk":
        (!candidate_safe&&!recovery?"candidate_unsafe":"insufficient_improvement");
  }else{
    const bool recovered=mpcc.safe&&mpcc.minimum_margin>=release_mpcc_margin_;
    safe_cycles_=recovered?safe_cycles_+1:0;
    if(now>=hold_until_&&safe_cycles_>=release_safe_cycles_){active_=false;
      out.released=true;out.reason="mpcc_recovered";}
    else if(!(candidate_safe||recovery)){active_=false;out.released=true;
      out.reason="candidate_became_unsafe";}
    else out.reason="candidate_hold";
  }
  out.recovery=active_recovery_;
  if(out.released)active_recovery_=false;
  out.use_candidate=active_;return out;
}

}  // namespace f1tenth_dynamic_mpcc
