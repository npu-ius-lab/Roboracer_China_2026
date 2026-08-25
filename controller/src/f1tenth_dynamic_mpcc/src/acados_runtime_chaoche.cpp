#include "f1tenth_dynamic_mpcc/acados_runtime_chaoche.hpp"

#include <dlfcn.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <utility>

namespace f1tenth_dynamic_mpcc {
namespace {
double ydouble(const YAML::Node& n, const char* key) { return n[key].as<double>(); }
int yint(const YAML::Node& n, const char* key, int fallback) {
  return n[key] ? n[key].as<int>() : fallback;
}
}

LocalReferenceProfile::LocalReferenceProfile(
    std::vector<LocalReferencePoint> points, double track_length)
    : points_(std::move(points)), track_length_(track_length) {
  if (track_length_ <= 0.0 || points_.size() < 4) {
    throw std::invalid_argument("local reference needs at least four points");
  }
  for (std::size_t index = 0; index < points_.size(); ++index) {
    const auto& point = points_[index];
    const auto& geometry = point.geometry;
    const bool finite = std::isfinite(point.s) && std::isfinite(geometry.x) &&
                        std::isfinite(geometry.y) && std::isfinite(geometry.yaw) &&
                        std::isfinite(geometry.curvature) &&
                        std::isfinite(geometry.width_left) &&
                        std::isfinite(geometry.width_right) &&
                        std::isfinite(geometry.speed_prior);
    if (!finite || geometry.width_left < 0.0 || geometry.width_right < 0.0 ||
        geometry.speed_prior < 0.0) {
      throw std::invalid_argument("invalid local reference geometry");
    }
    if (index > 0 && point.s <= points_[index - 1].s + 1.0e-5) {
      throw std::invalid_argument("local reference progress must increase");
    }
  }
}

double LocalReferenceProfile::unwrapNear(double s, double reference) const {
  const double wrapped = std::fmod(s, track_length_);
  const double positive = wrapped < 0.0 ? wrapped + track_length_ : wrapped;
  return positive + std::round((reference - positive) / track_length_) * track_length_;
}

bool LocalReferenceProfile::covers(double s) const {
  const double middle = 0.5 * (startS() + endS());
  const double query = unwrapNear(s, middle);
  return query >= startS() - 1.0e-3 && query <= endS() + 1.0e-3;
}

TrackGeometry LocalReferenceProfile::geometry(
    const PeriodicTrack& global_track, double s) const {
  const double middle = 0.5 * (startS() + endS());
  const double query = unwrapNear(s, middle);
  if (query < startS() || query > endS()) return global_track.geometry(s);
  auto upper = std::upper_bound(
      points_.begin(), points_.end(), query,
      [](double value, const LocalReferencePoint& point) { return value < point.s; });
  if (upper == points_.begin()) return upper->geometry;
  if (upper == points_.end()) return points_.back().geometry;
  const auto& right = *upper;
  const auto& left = *std::prev(upper);
  const double ratio = clamp((query - left.s) / (right.s - left.s), 0.0, 1.0);
  const auto blend = [ratio](double a, double b) { return a + ratio * (b - a); };
  TrackGeometry result;
  result.x = blend(left.geometry.x, right.geometry.x);
  result.y = blend(left.geometry.y, right.geometry.y);
  result.yaw = left.geometry.yaw +
               ratio * wrapAngle(right.geometry.yaw - left.geometry.yaw);
  result.curvature = blend(left.geometry.curvature, right.geometry.curvature);
  result.width_left = std::max(
      0.0, blend(left.geometry.width_left, right.geometry.width_left));
  result.width_right = std::max(
      0.0, blend(left.geometry.width_right, right.geometry.width_right));
  result.speed_prior = std::max(
      0.0, blend(left.geometry.speed_prior, right.geometry.speed_prior));
  return result;
}

void AcadosRuntimeSolverChaoche::setLocalReference(
    std::shared_ptr<const LocalReferenceProfile> reference) {
  if (!reference) throw std::invalid_argument("local reference is null");
  std::atomic_store_explicit(
      &local_reference_, std::move(reference), std::memory_order_release);
}

void AcadosRuntimeSolverChaoche::clearLocalReference() {
  std::atomic_store_explicit(
      &local_reference_, std::shared_ptr<const LocalReferenceProfile>{},
      std::memory_order_release);
}

bool AcadosRuntimeSolverChaoche::localReferenceActive() const {
  return static_cast<bool>(std::atomic_load_explicit(
      &local_reference_, std::memory_order_acquire));
}

TrackGeometry AcadosRuntimeSolverChaoche::referenceGeometry(double s) const {
  const auto reference = std::atomic_load_explicit(
      &local_reference_, std::memory_order_acquire);
  return reference ? reference->geometry(track_, s) : track_.geometry(s);
}

void* AcadosRuntimeSolverChaoche::symbol(void* handle, const std::string& name) const {
  dlerror(); void* result=dlsym(handle,name.c_str());
  if(const char* error=dlerror()) throw std::runtime_error("missing symbol "+name+": "+error);
  return result;
}

AcadosRuntimeSolverChaoche::AcadosRuntimeSolverChaoche(const std::string& formulation,
    const std::string& root, const PeriodicTrack& track,
    const VehicleParameters& vehicle, const YAML::Node& controller,
    std::shared_ptr<const f1tenth_residual_dynamics::ResidualModel> residual)
    : formulation_(formulation), track_(track), vehicle_(vehicle), controller_(controller),
      residual_(std::move(residual)) {
  if(formulation_!="baseline"&&formulation_!="mpcc") throw std::invalid_argument("invalid formulation");
  prefix_="f1tenth_dynamic_"+formulation_;
  horizon_=controller_["timing"]["horizon_steps"].as<int>();
  dt_=controller_["timing"]["horizon_dt_s"].as<double>();
  if(horizon_!=25) throw std::runtime_error("generated acados solver requires horizon_steps=25");
  const char* acados_env=std::getenv("ACADOS_SOURCE_DIR");
  std::string acados_root=acados_env?acados_env:"/home/tianbot/opt/acados-0.5.5";
  acados_handle_=dlopen((acados_root+"/lib/libacados.so").c_str(),RTLD_NOW|RTLD_GLOBAL);
  if(!acados_handle_) throw std::runtime_error(std::string("cannot load libacados: ")+dlerror());
  std::string directory=root+"/c_generated_code_"+formulation_;
  std::string library=directory+"/libacados_ocp_solver_"+prefix_+".so";
  solver_handle_=dlopen(library.c_str(),RTLD_NOW|RTLD_GLOBAL);
  if(!solver_handle_) throw std::runtime_error("cannot load generated solver "+library+": "+dlerror());
  create_capsule_=reinterpret_cast<CapsuleCreate>(symbol(solver_handle_,prefix_+"_acados_create_capsule"));
  free_capsule_=reinterpret_cast<CapsuleFree>(symbol(solver_handle_,prefix_+"_acados_free_capsule"));
  create_=reinterpret_cast<Create>(symbol(solver_handle_,prefix_+"_acados_create"));
  free_=reinterpret_cast<Free>(symbol(solver_handle_,prefix_+"_acados_free"));
  solve_=reinterpret_cast<Solve>(symbol(solver_handle_,prefix_+"_acados_solve"));
  update_params_=reinterpret_cast<UpdateParams>(symbol(solver_handle_,prefix_+"_acados_update_params"));
  auto getter=[&](const char* suffix){return reinterpret_cast<GetObject>(symbol(solver_handle_,prefix_+suffix));};
  capsule_=create_capsule_(); if(!capsule_||create_(capsule_)!=0) throw std::runtime_error("acados solver create failed");
  input_=getter("_acados_get_nlp_in")(capsule_); output_=getter("_acados_get_nlp_out")(capsule_);
  config_=getter("_acados_get_nlp_config")(capsule_); dims_=getter("_acados_get_nlp_dims")(capsule_);
  nlp_solver_=getter("_acados_get_nlp_solver")(capsule_);
  constraint_set_=reinterpret_cast<ConstraintSet>(symbol(acados_handle_,"ocp_nlp_constraints_model_set"));
  out_set_=reinterpret_cast<OutSet>(symbol(acados_handle_,"ocp_nlp_out_set"));
  out_get_=reinterpret_cast<OutGet>(symbol(acados_handle_,"ocp_nlp_out_get"));
  nlp_get_=reinterpret_cast<NlpGet>(symbol(acados_handle_,"ocp_nlp_get"));
  warm_states_.resize(horizon_+1,State::Zero()); warm_controls_.resize(horizon_,Control::Zero());
}

AcadosRuntimeSolverChaoche::~AcadosRuntimeSolverChaoche(){
  if(capsule_){free_(capsule_);free_capsule_(capsule_);} if(solver_handle_)dlclose(solver_handle_);if(acados_handle_)dlclose(acados_handle_);
}

void AcadosRuntimeSolverChaoche::resetWarmStart(){
  warm_valid_=false;
  for(auto& state:warm_states_){state[4]=0.0;state[5]=0.0;}
}

void AcadosRuntimeSolverChaoche::initializeWarmStart(const State& initial,double target_cap,
                                              double input_cap,double rate){
  DynamicBicycleModel model(vehicle_,5,residual_); State state=initial; warm_states_[0]=state;
  double previous_speed=std::max(state[3],0.0),previous_steer=state[7];
  for(int k=0;k<horizon_;++k){auto g=referenceGeometry(state[8]);double prior=std::min(g.speed_prior,target_cap);double next=clamp(prior,previous_speed-vehicle_.max_decel*dt_,previous_speed+vehicle_.max_accel*dt_);double accel=(next-previous_speed)/dt_;double tau=accel<0.0?vehicle_.braking_speed_tau:vehicle_.speed_tau;double speed=clamp((previous_speed+tau*accel)/vehicle_.speed_gain,0,input_cap);double ff=clamp(std::atan(vehicle_.wheelbase*g.curvature),-vehicle_.max_steer,vehicle_.max_steer);double steer_rate=clamp((ff-previous_steer)/dt_,-rate,rate);Control u;u<<speed,steer_rate,std::max(next,.2);warm_controls_[k]=u;state=model.step(state,u,dt_);warm_states_[k+1]=state;previous_speed=std::max(state[3],0.0);previous_steer=state[7];}warm_valid_=true;
}

void AcadosRuntimeSolverChaoche::advanceWarmStart(double elapsed_s){
  if(!warm_valid_||elapsed_s<=0.0)return;
  const double horizon_time=horizon_*dt_;
  if(!std::isfinite(elapsed_s)||elapsed_s>=horizon_time){
    warm_valid_=false;
    return;
  }
  const auto previous_states=warm_states_;
  const auto previous_controls=warm_controls_;
  const double offset=elapsed_s/dt_;
  for(int k=0;k<=horizon_;++k){
    const double source=std::min(k+offset,static_cast<double>(horizon_));
    const int lower=std::min(static_cast<int>(std::floor(source)),horizon_);
    const int upper=std::min(lower+1,horizon_);
    const double ratio=source-lower;
    warm_states_[k]=previous_states[lower]+ratio*(previous_states[upper]-previous_states[lower]);
    warm_states_[k][2]=previous_states[lower][2]+ratio*wrapAngle(
        previous_states[upper][2]-previous_states[lower][2]);
  }
  for(int k=0;k<horizon_;++k){
    const double source=std::min(k+offset,static_cast<double>(horizon_-1));
    const int lower=std::min(static_cast<int>(std::floor(source)),horizon_-1);
    const int upper=std::min(lower+1,horizon_-1);
    const double ratio=source-lower;
    warm_controls_[k]=previous_controls[lower]+ratio*(previous_controls[upper]-previous_controls[lower]);
  }
}

void AcadosRuntimeSolverChaoche::setInputBounds(double speed_cap,double rate_cap){
  const auto constraints=controller_["constraints"];
  double lbu[kNu]={0,-rate_cap,0};
  double ubu[kNu]={speed_cap,rate_cap,ydouble(constraints,"virtual_speed_max_mps")};
  for(int k=0;k<horizon_;++k){
    constraint_set_(config_,dims_,input_,output_,k,"lbu",lbu);
    constraint_set_(config_,dims_,input_,output_,k,"ubu",ubu);
  }
}

void AcadosRuntimeSolverChaoche::updateStageParameters(
    const std::vector<State>& states,const std::vector<Control>& controls,
    double speed_cap,double progress_scale){
  const auto cost=controller_["cost"],safety=controller_["safety"];
  const double speed_weight=formulation_=="baseline"||
      controller_["mode"].as<std::string>()=="debug"
      ?ydouble(cost,"speed_prior_debug"):ydouble(cost,"speed_prior_race");
  const double reward=(formulation_=="baseline"||
      controller_["mode"].as<std::string>()=="debug"
      ?ydouble(cost,"progress_reward_debug"):ydouble(cost,"progress_reward_race"))*
      clamp(progress_scale,0,1);
  const double heading_weight=formulation_=="baseline"||
      controller_["mode"].as<std::string>()=="debug"
      ?ydouble(cost,"heading_debug"):ydouble(cost,"heading_race");
  const double margin=vehicle_.body_width/2+
      ydouble(safety,"track_control_margin_m")+.05;
  for(int k=0;k<=horizon_;++k){
    const auto g=referenceGeometry(states[k][8]);
    const Control previous=k==0?previous_control_:controls[k-1];
    double p[kNp]={g.x,g.y,g.yaw,g.curvature,g.width_left,g.width_right,
      std::min(g.speed_prior,speed_cap),previous[0],previous[1],previous[2],
      margin,speed_weight,reward,heading_weight};
    update_params_(capsule_,k,p,kNp);
  }
}

void AcadosRuntimeSolverChaoche::seedSolver(const State& initial,double target_speed_cap,
                                     double input_speed_cap,double rate_cap,
                                     double progress_scale){
  if(!warm_valid_)initializeWarmStart(initial,target_speed_cap,input_speed_cap,rate_cap);
  if(formulation_=="baseline"){
    double theta=initial[8], speed=std::max(initial[3],.2);
    for(int k=0;k<=horizon_;++k){
      warm_states_[k][8]=theta;
      speed=clamp(std::min(referenceGeometry(theta).speed_prior,target_speed_cap),
                  speed-vehicle_.max_decel*dt_,speed+vehicle_.max_accel*dt_);
      theta+=dt_*std::max(speed,.2);
    }
  }
  updateStageParameters(warm_states_,warm_controls_,target_speed_cap,progress_scale);
  for(int k=0;k<=horizon_;++k){
    out_set_(config_,dims_,output_,input_,k,"x",warm_states_[k].data());
    if(k<horizon_)out_set_(config_,dims_,output_,input_,k,"u",warm_controls_[k].data());
  }
  double lbx[kNx],ubx[kNx];
  std::copy(initial.data(),initial.data()+kNx,lbx);
  std::copy(initial.data(),initial.data()+kNx,ubx);
  constraint_set_(config_,dims_,input_,output_,0,"lbx",lbx);
  constraint_set_(config_,dims_,input_,output_,0,"ubx",ubx);
  setInputBounds(input_speed_cap,rate_cap);
}

double AcadosRuntimeSolverChaoche::feasibleInputSpeedCap(
    const State& initial,double target_speed_cap) const {
  // h_accel >= -max_decel requires
  // speed_cmd >= (vx - max_decel*tau)/gain.  A runtime target may drop below
  // this value, but making it the hard upper bound would make the first QP
  // infeasible before the vehicle has had time to decelerate.
  if(controller_["safety"]["dynamic_speed_cap_feasibility_guard"] &&
     !controller_["safety"]["dynamic_speed_cap_feasibility_guard"].as<bool>())
    return target_speed_cap;
  return clamp(dynamicallyFeasibleSpeedCap(initial[3],target_speed_cap,
      vehicle_.speed_gain,vehicle_.braking_speed_tau,vehicle_.max_decel),
               0.0,vehicle_.max_speed);
}

void AcadosRuntimeSolverChaoche::readSolution(std::vector<State>& states,
                                      std::vector<Control>& controls) const{
  states.resize(horizon_+1);controls.resize(horizon_);
  for(int k=0;k<=horizon_;++k){
    out_get_(config_,dims_,output_,k,"x",states[k].data());
    if(k<horizon_)out_get_(config_,dims_,output_,k,"u",controls[k].data());
  }
}

TrackSlackSummary AcadosRuntimeSolverChaoche::readSlack(double near_horizon_s,
                                                 double& tire_slack) const{
  TrackSlackSummary summary;tire_slack=0;
  for(int k=1;k<horizon_;++k){
    double sl[4]{},su[4]{};
    out_get_(config_,dims_,output_,k,"sl",sl);
    out_get_(config_,dims_,output_,k,"su",su);
    for(int j=0;j<4;++j){
      const double value=std::max(sl[j],su[j]);
      if(j<2){
        if(k*dt_<=near_horizon_s+1e-9)summary.near=std::max(summary.near,value);
        else summary.far=std::max(summary.far,value);
        if(value>summary.maximum){summary.maximum=value;summary.stage=k;summary.side=j==0?1:-1;}
      }else tire_slack=std::max(tire_slack,value);
    }
  }
  return summary;
}

CostBreakdown AcadosRuntimeSolverChaoche::diagnosticCost(
    const std::vector<State>& states,const std::vector<Control>& controls,
    double target_speed_cap,double progress_scale) const {
  CostBreakdown out;if(states.size()!=static_cast<size_t>(horizon_+1)||
      controls.size()!=static_cast<size_t>(horizon_))return out;
  const auto cost=controller_["cost"];
  const bool debug=formulation_=="baseline"||
      controller_["mode"].as<std::string>()=="debug";
  const double heading_weight=ydouble(cost,debug?"heading_debug":"heading_race");
  const double speed_weight=ydouble(cost,debug?"speed_prior_debug":"speed_prior_race");
  const double reward=ydouble(cost,debug?"progress_reward_debug":"progress_reward_race")*
      clamp(progress_scale,0,1);
  Control previous=previous_control_;
  for(int k=0;k<horizon_;++k){
    const auto& x=states[k];const auto& u=controls[k];
    const auto g=referenceGeometry(x[8]);
    const double dx=x[0]-g.x,dy=x[1]-g.y;
    const double ec=-std::sin(g.yaw)*dx+std::cos(g.yaw)*dy;
    const double el= std::cos(g.yaw)*dx+std::sin(g.yaw)*dy;
    const double eh=wrapAngle(x[2]-g.yaw);
    const double vref=std::min(g.speed_prior,target_speed_cap);
    if(formulation_=="baseline")out.contour+=ydouble(cost,"contour")*(dx*dx+dy*dy)*dt_;
    else {out.contour+=ydouble(cost,"contour")*ec*ec*dt_;
          out.lag+=ydouble(cost,"lag")*el*el*dt_;}
    out.heading+=heading_weight*eh*eh*dt_;
    out.speed_prior+=speed_weight*(x[3]-vref)*(x[3]-vref)*dt_;
    out.progress-=reward*u[2]*dt_;
    out.control+=(ydouble(cost,"speed_command")*u[0]*u[0]+
      ydouble(cost,"steering_command")*x[7]*x[7]+
      ydouble(cost,"virtual_speed")*u[2]*u[2])*dt_;
    const Control du=u-previous;
    out.delta_control+=(ydouble(cost,"speed_command_rate")*du[0]*du[0]+
      ydouble(cost,"steering_command_rate")*u[1]*u[1]+
      ydouble(cost,"virtual_speed_rate")*du[2]*du[2])*dt_;
    previous=u;
    if(k>0){
      double sl[4]{},su[4]{};out_get_(config_,dims_,output_,k,"sl",sl);
      out_get_(config_,dims_,output_,k,"su",su);
      for(int j=0;j<4;++j){
        const double linear=j<2?ydouble(cost,"track_slack_linear"):0.0;
        const double quadratic=j<2?ydouble(cost,"track_slack_quadratic"):
            ydouble(cost,"tire_slack_quadratic");
        const double penalty=(linear*(sl[j]+su[j])+
            .5*quadratic*(sl[j]*sl[j]+su[j]*su[j]))*dt_;
        if(j<2)out.track_slack+=penalty;else out.tire_slack+=penalty;
      }
    }
  }
  return out;
}

SolverResult AcadosRuntimeSolverChaoche::solve(const State& initial,double speed_cap,
                                        double rate_cap,double progress_scale,
                                        double warm_start_advance_s){
  const auto safety=controller_["safety"],solver=controller_["solver"];
  const double near_horizon=ydouble(safety,"near_track_horizon_s");
  const PredictiveSlackPolicy policy{
    ydouble(safety,"emergency_track_slack_m"),
    ydouble(safety,"far_track_slack_hard_m"),
    ydouble(safety,"far_track_retry_speed_scale"),
    ydouble(safety,"far_track_retry_min_speed_mps")};
  const int regular_iterations=std::min(
      std::max(1,yint(solver,"rti_iterations",1)),
      1+std::max(0,yint(solver,"max_corrective_rti_per_cycle",1)));
  const int minimum=std::min(regular_iterations,
      std::max(1,yint(solver,"rti_min_iterations",1)));
  const int retry_iterations=std::max(1,yint(solver,"predictive_retry_rti_iterations",3));
  const double early=solver["rti_early_stop_track_slack_m"]
      ?ydouble(solver,"rti_early_stop_track_slack_m"):policy.emergency_slack;
  auto started=std::chrono::steady_clock::now();
  std::vector<State> states;
  std::vector<Control> controls;
  TrackSlackSummary slack;double tire_slack=0;int status=0,done=0;
  double geometry_theta_shift=0,geometry_heading_shift=0,geometry_curvature_shift=0;
  std::vector<State> parameter_states;

  double target_cap=clamp(speed_cap,0.0,vehicle_.max_speed);
  double input_cap=feasibleInputSpeedCap(initial,target_cap);
  const double warm_advance=warm_start_advance_s<0.0?dt_:warm_start_advance_s;
  advanceWarmStart(warm_advance);
  auto run_rti=[&](int iterations,bool stop_for_far){
    for(int i=0;i<iterations;++i){
      status=solve_(capsule_);++done;
      if(status)break;
      readSolution(states,controls);
      slack=readSlack(near_horizon,tire_slack);
      if(parameter_states.size()==states.size())for(size_t k=0;k<states.size();++k){
        const auto before=referenceGeometry(parameter_states[k][8]);
        const auto after=referenceGeometry(states[k][8]);
        geometry_theta_shift=std::max(geometry_theta_shift,
            std::abs(states[k][8]-parameter_states[k][8]));
        geometry_heading_shift=std::max(geometry_heading_shift,
            std::abs(wrapAngle(after.yaw-before.yaw)));
        geometry_curvature_shift=std::max(geometry_curvature_shift,
            std::abs(after.curvature-before.curvature));
      }
      // Approximate-MPCC geometry is frozen only within one RTI correction.
      // Refresh it from the new theta horizon before the next correction.
      updateStageParameters(states,controls,target_cap,progress_scale);
      parameter_states=states;
      if(i+1>=minimum){
        if(slack.maximum<=early)break;
        if(stop_for_far&&slack.near<=policy.emergency_slack&&
           slack.far>policy.emergency_slack)break;
      }
    }
  };

  seedSolver(initial,target_cap,input_cap,rate_cap,progress_scale);
  parameter_states=warm_states_;
  run_rti(regular_iterations,true);
  bool finite=status==0;
  for(const auto& x:states)finite=finite&&x.allFinite();
  for(const auto& u:controls)finite=finite&&u.allFinite();

  bool retried=false;double used_cap=target_cap;
  const double decision_base=std::min(
      target_cap,std::max(policy.retry_min_speed,std::max(initial[3],0.0)));
  auto decision=evaluateTrackSlack(slack,policy,decision_base,false);
  const TrackSlackSummary trigger_slack=slack;
  if(status==0&&finite&&decision.retry){
    retried=true;used_cap=decision.speed_cap;
    warm_valid_=false;
    target_cap=used_cap;
    input_cap=feasibleInputSpeedCap(initial,target_cap);
    seedSolver(initial,target_cap,input_cap,rate_cap,progress_scale);
    parameter_states=warm_states_;
    status=0;states.clear();controls.clear();slack={};tire_slack=0;
    run_rti(retry_iterations,false);
    finite=status==0;
    for(const auto& x:states)finite=finite&&x.allFinite();
    for(const auto& u:controls)finite=finite&&u.allFinite();
    decision=evaluateTrackSlack(slack,policy,used_cap,true);
  }

  const double elapsed=std::chrono::duration<double>(
      std::chrono::steady_clock::now()-started).count();
  bool success=status==0&&finite&&!decision.reject;
  std::string reason=decision.reason;int failure_kind=0;bool reset=false;
  if(status){success=false;reason="acados_status_"+std::to_string(status);failure_kind=4;}
  else if(!finite){success=false;reason="nonfinite_solution";failure_kind=5;}
  else if(decision.reject){failure_kind=decision.reason=="near_track_slack"?1:2;}
  else if(elapsed>controller_["timing"]["solver_deadline_s"].as<double>()){
    success=false;reason="deadline";failure_kind=3;
  }
  CostBreakdown cost_breakdown;
  if(status==0&&finite)cost_breakdown=diagnosticCost(
      states,controls,used_cap,progress_scale);
  if(!success){
    warm_valid_=false;
    initializeWarmStart(initial,used_cap,input_cap,rate_cap);
    reset=true;
  }else if(success){
    previous_control_=controls[0];
    warm_states_=states;
    warm_controls_=controls;
    warm_valid_=true;
  }
  SolverResult result;
  result.success=success;result.status=status;result.rti_iterations=done;
  if(status==0)nlp_get_(nlp_solver_,"cost_value",&result.objective);
  result.solve_time=elapsed;result.track_slack=slack.maximum;
  result.tire_slack=tire_slack;result.near_track_slack=slack.near;
  result.far_track_slack=slack.far;result.track_slack_stage=slack.stage;
  result.track_slack_time=slack.stage>=0?slack.stage*dt_:0;
  result.track_slack_side=slack.side;result.speed_cap_used=used_cap;
  result.speed_input_cap=input_cap;
  result.geometry_theta_shift=geometry_theta_shift;
  result.geometry_heading_shift=geometry_heading_shift;
  result.geometry_curvature_shift=geometry_curvature_shift;
  result.warm_start_advance_s=warm_advance;
  result.warm_start_shift_stages=warm_advance/dt_;
  result.trigger_track_slack=trigger_slack.maximum;
  result.trigger_track_slack_stage=trigger_slack.stage;
  result.trigger_track_slack_time=trigger_slack.stage>=0?trigger_slack.stage*dt_:0;
  result.trigger_track_slack_side=trigger_slack.side;
  result.failure_kind=failure_kind;result.speed_retry=retried;
  result.warm_start_reset=reset;result.reason=reason;
  result.cost=cost_breakdown;
  result.states=std::move(states);result.controls=std::move(controls);
  return result;
}

}  // namespace f1tenth_dynamic_mpcc
