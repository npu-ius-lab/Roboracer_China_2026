#include "f1tenth_dynamic_mpcc/core.hpp"

#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace f1tenth_dynamic_mpcc {
namespace {
double required(const YAML::Node& node, const char* key) {
  if (!node[key]) throw std::runtime_error(std::string("missing YAML key: ") + key);
  return node[key].as<double>();
}
template <typename T>
T value(const YAML::Node& node, const char* key, const T& fallback) {
  return node[key] ? node[key].as<T>() : fallback;
}
bool namesEqual(const std::vector<std::string>& actual,
                const std::vector<const char*>& expected) {
  if (actual.size() != expected.size()) return false;
  for (std::size_t i = 0; i < expected.size(); ++i)
    if (actual[i] != expected[i]) return false;
  return true;
}
std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream stream(line);
  std::string item;
  while (std::getline(stream, item, ',')) {
    if (!item.empty() && item.back() == '\r') item.pop_back();
    out.push_back(item);
  }
  return out;
}

Eigen::VectorXd makeResidualFeaturesV2(
    const f1tenth_residual_dynamics::ResidualModel& residual,
    const VehicleParameters& p,
    const State& x,
    const Control& u,
    double nominal_vx_dot,
    double nominal_vy_dot,
    double nominal_yaw_rate_dot,
    double delta_target) {
  static const std::vector<const char*> physics_expected{
      "bias", "vx", "vy", "yaw_rate", "delta", "speed_cmd", "steer_cmd",
      "speed_error", "steer_error", "vx_yaw_rate", "vy_yaw_rate",
      "abs_vx_vy", "abs_vx_yaw_rate", "vx_tan_delta", "vx_steer_cmd",
      "vx2_steer_cmd", "beta", "alpha_f", "alpha_r", "vx_alpha_f",
      "vx_alpha_r", "delta_target_delta", "nominal_vx_dot",
      "nominal_vy_dot", "nominal_yaw_rate_dot"};
  static const std::vector<const char*> history_expected{
      "bias", "vx", "vy", "yaw_rate", "delta", "speed_cmd", "steer_cmd",
      "speed_error", "steer_error", "vx_yaw_rate", "vy_yaw_rate",
      "abs_vx_vy", "abs_vx_yaw_rate", "vx_tan_delta", "vx_steer_cmd",
      "vx2_steer_cmd", "beta", "alpha_f", "alpha_r", "vx_alpha_f",
      "vx_alpha_r", "delta_target_delta", "nominal_vx_dot",
      "nominal_vy_dot", "nominal_yaw_rate_dot", "steer_cmd_t050ms",
      "steer_cmd_t100ms", "steer_cmd_t150ms", "steer_cmd_t200ms",
      "speed_cmd_t100ms", "speed_cmd_t200ms"};
  if (namesEqual(residual.featureNames(), history_expected))
    throw std::invalid_argument(
        "markov_history_v2 cannot be deployed causally in the current acados "
        "state; use a validated physics_v2 model");
  if (!namesEqual(residual.featureNames(), physics_expected))
    throw std::invalid_argument("residual v2 feature ordering does not match physics_v2 runtime");
  const double vx = x[3], vy = x[4], yaw_rate = x[5], delta = x[6];
  const double speed_cmd = u[0], steer_cmd = x[7];
  const double safe_vx = std::hypot(vx, p.vx_regularization);
  const double alpha_f = delta - std::atan2(vy + p.lf * yaw_rate, safe_vx);
  const double alpha_r = -std::atan2(vy - p.lr * yaw_rate, safe_vx);
  const double beta = std::atan2(vy, safe_vx);
  std::vector<double> values{
      1.0, vx, vy, yaw_rate, delta, speed_cmd, steer_cmd,
      speed_cmd - vx, steer_cmd - delta, vx * yaw_rate, vy * yaw_rate,
      std::abs(vx) * vy, std::abs(vx) * yaw_rate, vx * std::tan(delta),
      vx * steer_cmd, vx * vx * steer_cmd, beta, alpha_f, alpha_r,
      vx * alpha_f, vx * alpha_r, delta_target - delta, nominal_vx_dot,
      nominal_vy_dot, nominal_yaw_rate_dot};
  return Eigen::Map<Eigen::VectorXd>(values.data(), values.size());
}
}  // namespace

double clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }
double wrapAngle(double v) { return std::atan2(std::sin(v), std::cos(v)); }
double dynamicallyFeasibleSpeedCap(double vx,double target,double gain,
                                   double tau,double max_decel,double margin){
  if(!std::isfinite(vx)||!std::isfinite(target)||!std::isfinite(gain)||
     !std::isfinite(tau)||!std::isfinite(max_decel)||!std::isfinite(margin)||
     target<0||gain<=0||tau<=0||max_decel<=0||margin<0)
    throw std::invalid_argument("invalid dynamic speed cap parameters");
  return std::max(target,(std::max(vx,0.0)-max_decel*tau)/gain+margin);
}

TrackSlackDecision evaluateTrackSlack(const TrackSlackSummary& slack,
                                      const PredictiveSlackPolicy& policy,
                                      double requested_speed_cap,
                                      bool already_retried) {
  TrackSlackDecision out;
  out.speed_cap = requested_speed_cap;
  if (slack.near > policy.emergency_slack) {
    out.reject = true;
    out.reason = "near_track_slack";
    return out;
  }
  if (slack.far <= policy.emergency_slack) return out;

  const double reduced_cap = std::min(
      requested_speed_cap,
      std::max(policy.retry_min_speed,
               requested_speed_cap * policy.retry_speed_scale));
  out.speed_cap = reduced_cap;
  out.speed_limited = reduced_cap < requested_speed_cap - 1e-9;
  if (!already_retried) {
    out.retry = true;
    out.reason = "far_track_slack_retry";
    return out;
  }
  if (slack.far > policy.far_hard_slack) {
    out.reject = true;
    out.reason = "far_track_slack_hard";
    return out;
  }
  out.reason = "predictive_track_slowdown";
  return out;
}

PredictiveSpeedLimiter::PredictiveSpeedLimiter(double hold_s,
                                               double recovery_rate_mps2)
    : hold_s_(hold_s), recovery_rate_(recovery_rate_mps2) {
  if (!std::isfinite(hold_s_) || hold_s_ < 0.0 ||
      !std::isfinite(recovery_rate_) || recovery_rate_ <= 0.0) {
    throw std::invalid_argument("invalid predictive speed limiter parameters");
  }
}

void PredictiveSpeedLimiter::reset() {
  cap_ = std::numeric_limits<double>::infinity();
  hold_until_ = last_update_ = 0.0;
  active_ = false;
}

void PredictiveSpeedLimiter::noteRisk(double speed_cap, double now) {
  if (!std::isfinite(speed_cap) || speed_cap < 0.0 || !std::isfinite(now)) {
    throw std::invalid_argument("invalid predictive speed risk sample");
  }
  cap_ = active_ ? std::min(cap_, speed_cap) : speed_cap;
  hold_until_ = std::max(hold_until_, now + hold_s_);
  last_update_ = now;
  active_ = true;
}

double PredictiveSpeedLimiter::limit(double requested_speed_cap, double now) {
  if (!std::isfinite(requested_speed_cap) || requested_speed_cap < 0.0 ||
      !std::isfinite(now)) {
    throw std::invalid_argument("invalid predictive speed limit query");
  }
  if (!active_) return requested_speed_cap;
  if (now > hold_until_) {
    const double recovery_start = std::max(last_update_, hold_until_);
    cap_ += recovery_rate_ * std::max(0.0, now - recovery_start);
  }
  last_update_ = now;
  if (cap_ >= requested_speed_cap - 1e-9) {
    reset();
    return requested_speed_cap;
  }
  return std::min(requested_speed_cap, cap_);
}

VehicleParameters VehicleParameters::fromYaml(const YAML::Node& cfg,
                                               const YAML::Node& controller) {
  const auto v = cfg["vehicle"], m = cfg["dynamic_model"], l = cfg["limits"];
  const auto a = controller["actuator"], s = cfg["steering_actuator"];
  VehicleParameters p;
  p.wheelbase=required(v,"wheelbase"); p.body_width=required(v,"ego_width");
  p.mass=required(m,"mass"); p.yaw_inertia=required(m,"yaw_inertia");
  p.lf=required(m,"lf"); p.lr=required(m,"lr");
  p.cf=required(m,"tire_cornering_front"); p.cr=required(m,"tire_cornering_rear");
  p.vx_regularization=required(m,"vx_epsilon");
  p.speed_gain=value(a,"speed_static_gain",1.0);
  p.speed_tau=value(a,"speed_time_constant_s",required(l,"speed_time_constant"));
  p.speed_dead_time=value(a,"speed_dead_time_s",0.0);
  p.braking_speed_tau=value(a,"braking_speed_time_constant_s",p.speed_tau);
  p.braking_speed_dead_time=value(a,"braking_speed_dead_time_s",p.speed_dead_time);
  p.steering_gain=value(s,"gain",value(a,"steering_static_gain",1.0));
  p.steering_bias=value(s,"bias",0.0);
  p.steering_tau=value(s,"tau",value(a,"steering_time_constant_s",required(l,"steering_time_constant")));
  p.steering_dead_time=value(s,"delay",value(a,"steering_dead_time_s",0.0));
  p.max_speed=required(l,"max_speed"); p.max_steer=required(l,"max_steer");
  p.max_accel=required(l,"max_accel"); p.max_decel=required(l,"max_decel");
  p.max_steer_rate=required(l,"max_steer_rate");
  p.lateral_accel_limit=required(l,"lateral_accel_limit");
  p.validate(); return p;
}

void VehicleParameters::validate() const {
  if (wheelbase<=0 || body_width<=0 || mass<=0 || yaw_inertia<=0 || lf<=0 || lr<=0 ||
      cf<=0 || cr<=0 || vx_regularization<=0 || speed_tau<=0 ||
      braking_speed_tau<=0 || steering_tau<=0 || max_speed<=0 || max_steer<=0 ||
      speed_dead_time<0 || braking_speed_dead_time<0 || steering_dead_time<0 ||
      std::abs(lf+lr-wheelbase)>1e-6) throw std::runtime_error("invalid vehicle parameters");
}

LowSpeedStateConditioner::LowSpeedStateConditioner(
    const YAML::Node& config, double wheelbase)
    : wheelbase_(wheelbase) {
  enabled_=value(config,"enabled",true);
  enter_vx_=required(config,"stationary_enter_vx_mps");
  exit_vx_=required(config,"stationary_exit_vx_mps");
  command_max_=required(config,"stationary_command_max_mps");
  full_dynamic_vx_=required(config,"full_dynamic_vx_mps");
  if(!std::isfinite(wheelbase_)||wheelbase_<=0||
     !std::isfinite(enter_vx_)||enter_vx_<0||
     !std::isfinite(exit_vx_)||exit_vx_<=enter_vx_||
     !std::isfinite(command_max_)||command_max_<0||
     !std::isfinite(full_dynamic_vx_)||full_dynamic_vx_<=exit_vx_)
    throw std::invalid_argument("invalid low-speed state conditioning parameters");
}

LowSpeedConditioningResult LowSpeedStateConditioner::conditionImpl(
    const State& state, bool entered, bool exited) const {
  LowSpeedConditioningResult out;
  out.state=state;out.stationary=stationary_;out.entered=entered;out.exited=exited;
  if(!enabled_)return out;
  if(stationary_){
    out.state[3]=0.0;out.state[4]=0.0;out.state[5]=0.0;
    out.dynamic_blend=0.0;
    return out;
  }
  out.state[3]=std::max(0.0,out.state[3]);
  const double ratio=clamp((std::abs(out.state[3])-enter_vx_)/
      (full_dynamic_vx_-enter_vx_),0.0,1.0);
  const double blend=ratio*ratio*(3.0-2.0*ratio);
  const double r_kin=out.state[3]*std::tan(out.state[6])/wheelbase_;
  out.state[4]*=blend;
  out.state[5]=blend*out.state[5]+(1.0-blend)*r_kin;
  out.dynamic_blend=blend;
  return out;
}

LowSpeedConditioningResult LowSpeedStateConditioner::updateAndCondition(
    const State& state, double commanded_speed) {
  if(!state.allFinite()||!std::isfinite(commanded_speed))
    throw std::invalid_argument("non-finite low-speed state conditioning input");
  const bool previous=stationary_;
  if(enabled_){
    const double vx=std::abs(state[3]);
    const double command=std::abs(commanded_speed);
    if(stationary_){
      if(vx>=exit_vx_||command>command_max_)stationary_=false;
    }else if(vx<=enter_vx_&&command<=command_max_){
      stationary_=true;
    }
  }
  return conditionImpl(state,!previous&&stationary_,previous&&!stationary_);
}

LowSpeedConditioningResult LowSpeedStateConditioner::condition(
    const State& state) const {
  if(!state.allFinite())
    throw std::invalid_argument("non-finite low-speed state conditioning input");
  return conditionImpl(state,false,false);
}

void LowSpeedStateConditioner::reset(){stationary_=false;}

DynamicBicycleModel::DynamicBicycleModel(
    VehicleParameters p, int substeps,
    std::shared_ptr<const f1tenth_residual_dynamics::ResidualModel> residual)
    : p_(std::move(p)), substeps_(substeps), residual_(std::move(residual)) {
  if (substeps_ < 1) throw std::invalid_argument("integration substeps must be positive");
}

std::array<double,2> DynamicBicycleModel::slipAngles(const State& x) const {
  const double safe=std::hypot(x[3],p_.vx_regularization);
  return {x[6]-std::atan2(x[4]+p_.lf*x[5],safe),
          -std::atan2(x[4]-p_.lr*x[5],safe)};
}

State DynamicBicycleModel::derivatives(const State& x, const Control& u) const {
  State d; const auto slip=slipAngles(x);
  const double fyf=p_.cf*slip[0], fyr=p_.cr*slip[1];
  const double speed=clamp(u[0],0,p_.max_speed);
  const double cmd_rate=clamp(u[1],-p_.max_steer_rate,p_.max_steer_rate);
  // Smoothly select the identified braking response when the requested wheel
  // speed lies below the measured longitudinal speed. The 0.05 m/s blend
  // avoids a derivative discontinuity in the generated SQP dynamics.
  const double braking_blend=0.5*(std::tanh(
      (x[3]-p_.speed_gain*speed)/0.05)+1.0);
  const double speed_tau=(1.0-braking_blend)*p_.speed_tau+
      braking_blend*p_.braking_speed_tau;
  const double vx_dot=clamp((p_.speed_gain*speed-x[3])/speed_tau,
                            -p_.max_decel,p_.max_accel);
  const double blend=0.5*(std::tanh((x[3]-0.45)/0.08)+1.0);
  const double vy_dyn=(fyf+fyr)/p_.mass-x[3]*x[5];
  const double r_dyn=(p_.lf*fyf-p_.lr*fyr)/p_.yaw_inertia;
  const double r_kin=x[3]*std::tan(x[6])/p_.wheelbase;
  const double target=clamp(p_.steering_gain*x[7]+p_.steering_bias,-p_.max_steer,p_.max_steer);
  const double delta_dot=clamp((target-x[6])/p_.steering_tau,-p_.max_steer_rate,p_.max_steer_rate);
  d << x[3]*std::cos(x[2])-x[4]*std::sin(x[2]),
       x[3]*std::sin(x[2])+x[4]*std::cos(x[2]), x[5], vx_dot,
       blend*vy_dyn+(1-blend)*(-x[4]/0.12),
       blend*r_dyn+(1-blend)*((r_kin-x[5])/0.10),
       delta_dot, cmd_rate, std::max(u[2],0.0);
  if(residual_){
    Eigen::VectorXd features;
    if(residual_->schemaVersion()==2){
      features=makeResidualFeaturesV2(
        *residual_,p_,x,u,vx_dot,d[4],d[5],target);
    }else{
      const std::vector<std::array<double,2>> command{{u[0],x[7]}};
      features=residual_->makeFeatures(x[3],x[4],x[5],x[6],command);
    }
    const auto correction=residual_->predict(features);
    d[3]+=correction[0];d[4]+=correction[1];d[5]+=correction[2];
  }
  return d;
}

State DynamicBicycleModel::step(const State& state, const Control& control, double dt) const {
  if (!(dt>0) || !std::isfinite(dt)) throw std::invalid_argument("invalid integration dt");
  State out=state; const double h=dt/substeps_;
  for(int i=0;i<substeps_;++i){
    const State k1=derivatives(out,control), k2=derivatives(out+0.5*h*k1,control);
    const State k3=derivatives(out+0.5*h*k2,control), k4=derivatives(out+h*k3,control);
    out += h*(k1+2*k2+2*k3+k4)/6.0;
  }
  out[3]=std::max(0.0,out[3]); out[6]=clamp(out[6],-p_.max_steer,p_.max_steer);
  out[7]=clamp(out[7],-p_.max_steer,p_.max_steer); return out;
}

void PeriodicSpline::fit(const std::vector<double>& knots, const std::vector<double>& values,
                         double period) {
  if(knots.size()!=values.size() || knots.size()<4 || period<=knots.back())
    throw std::runtime_error("invalid periodic spline data");
  x_=knots; y_=values; period_=period; x_.push_back(period_); y_.push_back(values.front());
  const int n=static_cast<int>(values.size());
  Eigen::MatrixXd a=Eigen::MatrixXd::Zero(n,n); Eigen::VectorXd rhs(n);
  for(int i=0;i<n;++i){
    const int prev=(i+n-1)%n,next=(i+1)%n;
    const double xi=knots[i];
    const double xp=(i==0?knots[n-1]-period:knots[i-1]);
    const double xn=(i==n-1?period:knots[i+1]);
    const double hp=xi-xp, hn=xn-xi;
    a(i,prev)=hp; a(i,i)=2*(hp+hn); a(i,next)=hn;
    rhs[i]=6*((values[next]-values[i])/hn-(values[i]-values[prev])/hp);
  }
  Eigen::VectorXd sol=a.colPivHouseholderQr().solve(rhs);
  second_.resize(n+1); for(int i=0;i<n;++i) second_[i]=sol[i]; second_[n]=sol[0];
}

double PeriodicSpline::eval(double query, int derivative) const {
  double q=std::fmod(query,period_); if(q<0) q+=period_;
  auto it=std::upper_bound(x_.begin(),x_.end(),q);
  int i=std::max(0,static_cast<int>(it-x_.begin())-1); i=std::min(i,static_cast<int>(x_.size())-2);
  const double h=x_[i+1]-x_[i], A=(x_[i+1]-q)/h, B=(q-x_[i])/h;
  if(derivative==0) return A*y_[i]+B*y_[i+1]+((A*A*A-A)*second_[i]+(B*B*B-B)*second_[i+1])*h*h/6;
  if(derivative==1) return (y_[i+1]-y_[i])/h+h*((-3*A*A+1)*second_[i]+(3*B*B-1)*second_[i+1])/6;
  return A*second_[i]+B*second_[i+1];
}

PeriodicTrack::PeriodicTrack(const std::string& path, const YAML::Node& speed_cfg,
                             const VehicleParameters& vehicle) {
  std::ifstream f(path); if(!f) throw std::runtime_error("cannot open raceline: "+path);
  std::string line; std::getline(f,line); const auto header=split(line);
  auto index=[&](const std::string& name){auto it=std::find(header.begin(),header.end(),name); if(it==header.end()) throw std::runtime_error("raceline missing "+name); return static_cast<int>(it-header.begin());};
  auto optional_index=[&](const std::string& name){auto it=std::find(header.begin(),header.end(),name);return it==header.end()?-1:static_cast<int>(it-header.begin());};
  const int is=index("s_m"),ix=index("x_m"),iy=index("y_m"),ir=index("w_tr_right_m"),il=index("w_tr_left_m");
  const int ik=optional_index("kappa_radpm"),iv=optional_index("vx_mps"),ia=optional_index("ax_mps2"),
            ilimit=optional_index("speed_limit_mps");
  while(std::getline(f,line)){if(line.empty())continue;const auto c=split(line);auto get=[&](int n){return std::stod(c.at(n));};s_nodes_.push_back(get(is));x_nodes_.push_back(get(ix));y_nodes_.push_back(get(iy));right_nodes_.push_back(get(ir));left_nodes_.push_back(get(il));if(ik>=0)kappa_nodes_.push_back(get(ik));if(iv>=0)speed_nodes_.push_back(get(iv));if(ia>=0)accel_nodes_.push_back(get(ia));if(ilimit>=0)speed_limit_nodes_.push_back(get(ilimit));}
  if(s_nodes_.size()<8) throw std::runtime_error("raceline has too few rows");
  if(!speed_limit_nodes_.empty()){
    if(speed_limit_nodes_.size()!=s_nodes_.size())
      throw std::runtime_error("speed_limit_mps must be present on every raceline row");
    for(double limit:speed_limit_nodes_)
      if(!std::isfinite(limit)||limit<=0)
        throw std::runtime_error("speed_limit_mps values must be finite and positive");
  }
  const double offset=s_nodes_.front(); for(auto& s:s_nodes_) s-=offset;
  length_=s_nodes_.back()+std::hypot(x_nodes_.front()-x_nodes_.back(),y_nodes_.front()-y_nodes_.back());
  x_.fit(s_nodes_,x_nodes_,length_); y_.fit(s_nodes_,y_nodes_,length_);
  left_.fit(s_nodes_,left_nodes_,length_); right_.fit(s_nodes_,right_nodes_,length_);
  if(kappa_nodes_.empty()){for(double s:s_nodes_)kappa_nodes_.push_back(curvature(s));}
  csv_kappa_.fit(s_nodes_,kappa_nodes_,length_);
  if(speed_cfg && value(speed_cfg,"enabled",false))speed_nodes_=planSpeed(speed_cfg,vehicle);
  else if(speed_nodes_.empty())throw std::runtime_error("raceline missing vx_mps while runtime speed planning is disabled");
  // ax_csv is diagnostic only.  The online profile acceleration always comes
  // from the single runtime speed envelope used by the controller.
  accel_nodes_=profileAcceleration(speed_nodes_);
  speed_.fit(s_nodes_,speed_nodes_,length_); accel_.fit(s_nodes_,accel_nodes_,length_);
  constexpr int count=4096; coarse_s_.reserve(count); coarse_x_.reserve(count); coarse_y_.reserve(count);
  for(int i=0;i<count;++i){double s=length_*i/count;coarse_s_.push_back(s);coarse_x_.push_back(x_.eval(s));coarse_y_.push_back(y_.eval(s));}
}

std::vector<double> PeriodicTrack::profileAcceleration(
    const std::vector<double>& speed) const {
  std::vector<double> out(speed.size());
  for(size_t i=0;i<speed.size();++i){const size_t next=(i+1)%speed.size();const double ds=next? s_nodes_[next]-s_nodes_[i]:length_-s_nodes_[i];out[i]=(speed[next]*speed[next]-speed[i]*speed[i])/(2*std::max(ds,1e-9));}
  return out;
}

std::vector<double> PeriodicTrack::planSpeed(const YAML::Node& c,const VehicleParameters& p) const {
  const double minimum=required(c,"min_speed_mps"), maximum=required(c,"max_speed_mps");
  const double profile_accel=std::min(
      p.max_accel,value(c,"profile_max_accel_mps2",p.max_accel));
  const double profile_decel=std::min(
      p.max_decel,value(c,"profile_max_decel_mps2",p.max_decel));
  if(!std::isfinite(profile_accel)||profile_accel<=0)
    throw std::runtime_error("profile_max_accel_mps2 must be finite and positive");
  if(!std::isfinite(profile_decel)||profile_decel<=0)
    throw std::runtime_error("profile_max_decel_mps2 must be finite and positive");
  const size_t n = s_nodes_.size();
  std::vector<double> out(n), steering(n), ds(n);
  for (size_t i = 0; i < n; ++i) {
    const double node_maximum=speed_limit_nodes_.empty()
      ?maximum:std::min(maximum,speed_limit_nodes_[i]);
    if(node_maximum<minimum)
      throw std::runtime_error("local speed limit is below minimum planned speed");
    const double k = curvature(s_nodes_[i]);
    steering[i] = std::atan(p.wheelbase * k);
    out[i] = clamp(
        std::sqrt(p.lateral_accel_limit / std::max(std::abs(k), 1e-4)),
        minimum, node_maximum);
    if (std::abs(steering[i]) > 0.9 * p.max_steer) out[i] = minimum;
    ds[i] = i + 1 < n ? s_nodes_[i + 1] - s_nodes_[i]
                      : length_ - s_nodes_[i];
  }
  for (size_t i = 0; i < n; ++i) {
    const size_t previous = (i + n - 1) % n;
    const size_t following = (i + 1) % n;
    const double centered_distance = ds[previous] + ds[i];
    const double steering_slope =
        std::abs(steering[following] - steering[previous]) /
        std::max(centered_distance, 1e-9);
    const double node_maximum=speed_limit_nodes_.empty()
      ?maximum:std::min(maximum,speed_limit_nodes_[i]);
    out[i] = std::min(out[i], clamp(
        p.max_steer_rate / std::max(steering_slope, 1e-9), minimum, node_maximum));
  }
  for (int pass = 0; pass < std::max(8, 2 * static_cast<int>(n)); ++pass) {
    const auto previous_profile = out;
    for (size_t i = 0; i < n; ++i) {
      const size_t following = (i + 1) % n;
      out[following] = std::min(
          out[following], std::sqrt(
              out[i] * out[i] + 2 * profile_accel * ds[i]));
    }
    for (size_t i = n; i-- > 0;) {
      const size_t preceding = (i + n - 1) % n;
      out[preceding] = std::min(
          out[preceding], std::sqrt(out[i] * out[i] + 2 * profile_decel * ds[preceding]));
    }
    double change = 0.0;
    for (size_t i = 0; i < n; ++i) change = std::max(change, std::abs(out[i] - previous_profile[i]));
    if (change < 1e-6) break;
  }
  return out;
}

double PeriodicTrack::wrapS(double s) const { double q=std::fmod(s,length_); return q<0?q+length_:q; }
std::array<double,2> PeriodicTrack::position(double s) const {return{x_.eval(s),y_.eval(s)};}
double PeriodicTrack::tangent(double s) const{return std::atan2(y_.eval(s,1),x_.eval(s,1));}
double PeriodicTrack::curvature(double s) const{double dx=x_.eval(s,1),dy=y_.eval(s,1);return(dx*y_.eval(s,2)-dy*x_.eval(s,2))/std::max(std::pow(dx*dx+dy*dy,1.5),1e-9);}
double PeriodicTrack::widthLeft(double s) const{return std::max(0.0,left_.eval(s));}
double PeriodicTrack::widthRight(double s) const{return std::max(0.0,right_.eval(s));}
double PeriodicTrack::speedPrior(double s) const {
  const double q=wrapS(s);
  auto it=std::upper_bound(s_nodes_.begin(),s_nodes_.end(),q);
  const std::size_t i=it==s_nodes_.begin()?0:
      static_cast<std::size_t>(it-s_nodes_.begin()-1);
  const std::size_t next=(i+1)%speed_nodes_.size();
  return clamp(speed_.eval(q),std::min(speed_nodes_[i],speed_nodes_[next]),
               std::max(speed_nodes_[i],speed_nodes_[next]));
}
double PeriodicTrack::csvCurvature(double s) const{return csv_kappa_.eval(s);}
TrackGeometry PeriodicTrack::geometry(double s) const {auto p=position(s);return{p[0],p[1],tangent(s),curvature(s),widthLeft(s),widthRight(s),speedPrior(s)};}
TrackProjection PeriodicTrack::project(double x,double y,std::optional<double> guess) const{
  double s=0; if(guess){double center=wrapS(*guess),best=1e99;for(int i=0;i<129;++i){double q=wrapS(center-std::max(1.5,length_/12.0)+2*std::max(1.5,length_/12.0)*i/128.0);auto p=position(q);double d=(p[0]-x)*(p[0]-x)+(p[1]-y)*(p[1]-y);if(d<best){best=d;s=q;}}}else{double best=1e99;for(size_t i=0;i<coarse_s_.size();++i){double d=(coarse_x_[i]-x)*(coarse_x_[i]-x)+(coarse_y_[i]-y)*(coarse_y_[i]-y);if(d<best){best=d;s=coarse_s_[i];}}}
  for(int i=0;i<12;++i){double q=wrapS(s),xr=x_.eval(q),yr=y_.eval(q),dx=x_.eval(q,1),dy=y_.eval(q,1),ddx=x_.eval(q,2),ddy=y_.eval(q,2);double g=(xr-x)*dx+(yr-y)*dy,h=dx*dx+dy*dy+(xr-x)*ddx+(yr-y)*ddy;if(std::abs(h)<1e-9)break;double step=clamp(g/h,-.5,.5);s-=step;if(std::abs(step)<1e-10)break;}
  double sw=wrapS(s),unwrapped=guess?sw+std::round((*guess-sw)/length_)*length_:sw;auto p=position(sw);double yaw=tangent(sw),ex=x-p[0],ey=y-p[1];return{unwrapped,sw,-std::sin(yaw)*ex+std::cos(yaw)*ey,std::cos(yaw)*ex+std::sin(yaw)*ey,std::hypot(ex,ey),p[0],p[1],yaw};
}

void CommandHistory::push(const CommandSample& s){std::lock_guard<std::mutex>l(mutex_);auto it=std::lower_bound(samples_.begin(),samples_.end(),s.stamp,[](const auto&a,double t){return a.stamp<t;});if(it!=samples_.end()&&it->stamp==s.stamp)*it=s;else samples_.insert(it,s);while(samples_.size()>1&&samples_[1].stamp<samples_.back().stamp-retention_)samples_.pop_front();}
void CommandHistory::clear(){std::lock_guard<std::mutex>l(mutex_);samples_.clear();}
std::optional<CommandSample> CommandHistory::commandAt(double t)const{std::lock_guard<std::mutex>l(mutex_);if(samples_.empty()||t<samples_.front().stamp)return{};auto it=std::upper_bound(samples_.begin(),samples_.end(),t,[](double q,const auto&s){return q<s.stamp;});--it;auto out=*it;out.stamp=t;return out;}
std::optional<CommandSample> CommandHistory::effectiveAt(
    double t,double speed_delay,double steering_delay)const{
  auto speed=commandAt(t-speed_delay+1e-12);
  auto steer=commandAt(t-steering_delay+1e-12);
  if(!speed||!steer)return{};
  return CommandSample{t,speed->speed,steer->steering,
                       steer->virtual_speed,steer->steering_rate};
}

void SensorHistory::pushImu(ImuSample s){std::lock_guard<std::mutex>l(mutex_);auto it=std::lower_bound(imu_.begin(),imu_.end(),s.stamp,[](const auto&a,double t){return a.stamp<t;});imu_.insert(it,s);while(imu_.size()>1&&imu_[1].stamp<imu_.back().stamp-retention_)imu_.pop_front();}
void SensorHistory::pushWheel(WheelSample s){std::lock_guard<std::mutex>l(mutex_);auto it=std::lower_bound(wheel_.begin(),wheel_.end(),s.stamp,[](const auto&a,double t){return a.stamp<t;});wheel_.insert(it,s);while(wheel_.size()>1&&wheel_[1].stamp<wheel_.back().stamp-retention_)wheel_.pop_front();}
std::optional<ImuSample> SensorHistory::imuBefore(double t)const{std::lock_guard<std::mutex>l(mutex_);auto it=std::upper_bound(imu_.begin(),imu_.end(),t,[](double q,const auto&s){return q<s.stamp;});if(it==imu_.begin())return{};return*--it;}
std::optional<WheelSample> SensorHistory::wheelBefore(double t)const{std::lock_guard<std::mutex>l(mutex_);auto it=std::upper_bound(wheel_.begin(),wheel_.end(),t,[](double q,const auto&s){return q<s.stamp;});if(it==wheel_.begin())return{};return*--it;}
void SensorHistory::clear(){std::lock_guard<std::mutex>l(mutex_);imu_.clear();wheel_.clear();}

void StateHistory::push(double t,const State& x){std::lock_guard<std::mutex>l(mutex_);auto it=std::lower_bound(samples_.begin(),samples_.end(),t,[](const auto&a,double q){return a.stamp<q;});if(it!=samples_.end()&&it->stamp==t)it->state=x;else samples_.insert(it,{t,x});while(samples_.size()>1&&samples_[1].stamp<samples_.back().stamp-retention_)samples_.pop_front();}
std::optional<State> StateHistory::at(double t)const{std::lock_guard<std::mutex>l(mutex_);if(samples_.empty()||t<samples_.front().stamp||t>samples_.back().stamp)return{};auto it=std::upper_bound(samples_.begin(),samples_.end(),t,[](double q,const auto&s){return q<s.stamp;});if(it==samples_.end()||std::prev(it)->stamp==t)return std::prev(it)->state;auto b=it,a=std::prev(it);double r=(t-a->stamp)/(b->stamp-a->stamp);State x=a->state+r*(b->state-a->state);x[2]=a->state[2]+r*wrapAngle(b->state[2]-a->state[2]);return x;}
void StateHistory::truncateAfter(double t){std::lock_guard<std::mutex>l(mutex_);auto it=std::upper_bound(samples_.begin(),samples_.end(),t,[](double q,const auto&s){return q<s.stamp;});samples_.erase(it,samples_.end());}
void StateHistory::clear(){std::lock_guard<std::mutex>l(mutex_);samples_.clear();}

LowLatencyPredictor::LowLatencyPredictor(DynamicBicycleModel m,CommandHistory& c,double step):model_(std::move(m)),commands_(c),step_(step){}
void LowLatencyPredictor::pushImu(double t,double r){sensors_.pushImu({t,r});}
void LowLatencyPredictor::pushWheel(double t,double v){sensors_.pushWheel({t,v});}
void LowLatencyPredictor::clear(){std::lock_guard<std::mutex>l(mutex_);sensors_.clear();states_.clear();last_pointlio_stamp_.reset();current_stamp_.reset();current_state_.reset();}
double LowLatencyPredictor::speedDelayFor(
    const State& state,double t,const Control& fallback)const{
  const auto& p=model_.parameters();
  const auto intent=commands_.commandAt(t);
  const double requested=intent?intent->speed:fallback[0];
  return p.speed_gain*requested<state[3]-0.02
      ?p.braking_speed_dead_time:p.speed_dead_time;
}
Control LowLatencyPredictor::controlAt(
    double t,const State& state,const Control& fallback)const{
  const auto& p=model_.parameters();
  auto c=commands_.effectiveAt(
      t,speedDelayFor(state,t,fallback),p.steering_dead_time);
  if(!c)return fallback;
  Control u;u<<c->speed,c->steering_rate,c->virtual_speed;return u;
}
State LowLatencyPredictor::propagate(const State& state,double start,double end,const Control& fallback,bool record){if(end<start)throw std::runtime_error("prediction end precedes start");State out=state;double cursor=start;if(record)states_.push(cursor,out);while(cursor<end-1e-12){double dt=std::min(step_,end-cursor);out=model_.step(out,controlAt(cursor,out,fallback),dt);auto imu=sensors_.imuBefore(cursor+dt);if(imu&&cursor+dt-imu->stamp>=-1e-6&&cursor+dt-imu->stamp<=imu_max_age_){double corrected=imu->yaw_rate-gyro_bias_;out[2]=wrapAngle(out[2]+(corrected-out[5])*dt);out[5]=corrected;}auto wheel=sensors_.wheelBefore(cursor+dt);if(wheel&&cursor+dt-wheel->stamp>=-1e-6&&cursor+dt-wheel->stamp<=wheel_max_age_){double innovation=clamp(wheel->vx-out[3],-wheel_innovation_limit_,wheel_innovation_limit_);out[3]=std::max(0.0,out[3]+std::min(1.0,wheel_gain_*dt/.02)*innovation);}cursor+=dt;if(record)states_.push(cursor,out);}return out;}
Prediction LowLatencyPredictor::correctAndRepropagate(const State& measured,double stamp,double now,double max_age,const Control& fallback){std::lock_guard<std::mutex>l(mutex_);double age=now-stamp;if(age<-.000001||age>max_age)throw std::runtime_error("PointLIO timestamp outside repropagation window");if(!last_pointlio_stamp_||stamp>*last_pointlio_stamp_+1e-9){State corrected=measured;auto historical=states_.at(stamp);if(historical)corrected.segment<2>(6)=historical->segment<2>(6);states_.truncateAfter(stamp);states_.push(stamp,corrected);current_state_=corrected;current_stamp_=stamp;last_pointlio_stamp_=stamp;}if(!current_state_){current_state_=measured;current_stamp_=stamp;}if(*current_stamp_<now-1e-12){current_state_=propagate(*current_state_,*current_stamp_,now,fallback,true);current_stamp_=now;}return{*current_state_,stamp,now,std::max(age,0.0),std::max(age,0.0),std::max(0.0,1-age/std::max(max_age,1e-6)),"REPROPAGATION"};}
Prediction LowLatencyPredictor::committedHorizon(const State& state,double now,const Control& fallback){const double horizon=speedDelayFor(state,now,fallback);double end=now+horizon;return{propagate(state,now,end,fallback,false),now,end,0,horizon,1,"COMMITTED_HORIZON"};}

SteeringObserver::SteeringObserver(const YAML::Node& cfg,const VehicleParameters& p):vehicle_(p){auto a=cfg["steering_actuator"],o=cfg["steering_observer"];enabled_=value(o,"enabled",true);gain_=required(a,"gain");bias_=value(a,"bias",0.0);tau_=required(a,"tau");delay_=value(a,"delay",0.0);delta_min_=required(a,"delta_min");delta_max_=required(a,"delta_max");model_only_speed_=required(o,"model_only_max_speed_mps");dynamic_speed_=required(o,"dynamic_correction_min_speed_mps");kinematic_gain_=required(o,"kinematic_gain_max");dynamic_gain_=required(o,"dynamic_gain_max");vy_cutoff_=required(o,"vy_derivative_cutoff_hz");r_cutoff_=required(o,"yaw_rate_derivative_cutoff_hz");pseudo_cutoff_=value(o,"pseudo_angle_cutoff_hz",2.0);innovation_limit_=required(o,"innovation_limit_rad");timeout_=value(o,"dropout_timeout_s",.25);estimate_.variance=value(o,"initial_variance_rad2",.04);}
double SteeringObserver::target(double c)const{return clamp(gain_*c+bias_,delta_min_,delta_max_);}double SteeringObserver::propagate(double d,double c,double dt)const{return clamp(target(c)+(d-target(c))*std::exp(-dt/tau_),delta_min_,delta_max_);}double SteeringObserver::commandAt(double t)const{double out=0,q=t-delay_;for(const auto&s:commands_){if(s.first>q)break;out=s.second;}return out;}
void SteeringObserver::pushCommand(double t,double d){std::lock_guard<std::mutex>l(mutex_);commands_.emplace_back(t,d);while(commands_.size()>1&&commands_[1].first<t-3)commands_.pop_front();}
ObserverEstimate SteeringObserver::update(double t,double vx,double vy,double r){std::lock_guard<std::mutex>l(mutex_);if(!last_stamp_){last_stamp_=t;last_vy_=vy;last_r_=r;estimate_={t,estimate_.delta,target(commandAt(t)),estimate_.delta,0,estimate_.variance,.2,true,ObserverMode::MODEL_ONLY};return estimate_;}double dt=t-*last_stamp_;if(dt<=0)return estimate_;double pred=propagate(estimate_.delta,commandAt(t),dt),model_pred=pred,innovation=0,g=0;ObserverMode mode=ObserverMode::MODEL_ONLY;bool interval=dt<=timeout_;if(interval&&enabled_){auto low=[](double prev,double val,double cutoff,double h){return prev+(1-std::exp(-2*M_PI*cutoff*h))*(val-prev);};vy_dot_filtered_=low(vy_dot_filtered_,(vy-*last_vy_)/dt,vy_cutoff_,dt);r_dot_filtered_=low(r_dot_filtered_,(r-*last_r_)/dt,r_cutoff_,dt);double ratio=clamp((std::abs(vx)-model_only_speed_)/(dynamic_speed_-model_only_speed_),0,1);ratio=ratio*ratio*(3-2*ratio);double pseudo=pred;if(std::abs(vx)>=dynamic_speed_){g=dynamic_gain_;double safe=std::max(std::abs(vx),vehicle_.vx_regularization),ay=vy_dot_filtered_+vx*r,fyf=(vehicle_.lr*vehicle_.mass*ay+vehicle_.yaw_inertia*r_dot_filtered_)/vehicle_.wheelbase;double raw=clamp(std::atan2(vy+vehicle_.lf*r,safe)+fyf/vehicle_.cf,delta_min_,delta_max_);pseudo_filtered_=pseudo_filtered_?low(*pseudo_filtered_,raw,pseudo_cutoff_,dt):raw;pseudo=*pseudo_filtered_;mode=ObserverMode::DYNAMIC;++valid_samples_;}else if(ratio>0){g=kinematic_gain_*ratio;pseudo=clamp(std::atan(vehicle_.wheelbase*r/std::max(std::abs(vx),.1)),delta_min_,delta_max_);mode=ObserverMode::KINEMATIC;++valid_samples_;}else valid_samples_=0;innovation=clamp(pseudo-pred,-innovation_limit_,innovation_limit_);pred=clamp(pred+g*innovation,delta_min_,delta_max_);}double confidence=mode==ObserverMode::MODEL_ONLY?.2:(mode==ObserverMode::KINEMATIC?std::min(.7,.3+.02*valid_samples_):1.0);double variance=mode==ObserverMode::MODEL_ONLY?std::min(estimate_.variance+2e-4*dt,.25):std::max(estimate_.variance*(1-.5*g),1e-5);estimate_={t,pred,target(commandAt(t)),model_pred,innovation,variance,confidence,true,mode};last_stamp_=t;last_vy_=vy;last_r_=r;return estimate_;}
ObserverEstimate SteeringObserver::estimate()const{std::lock_guard<std::mutex>l(mutex_);return estimate_;}void SteeringObserver::invalidate(){std::lock_guard<std::mutex>l(mutex_);estimate_.valid=false;estimate_.confidence=0;estimate_.mode=ObserverMode::INVALID;}

std::string startupModeName(StartupMode m){switch(m){case StartupMode::BOOT:return"BOOT";case StartupMode::STEER_SETTLE:return"STEER_SETTLE";case StartupMode::CRAWL_OBSERVE:return"CRAWL_OBSERVE";case StartupMode::MPCC_RAMP:return"MPCC_RAMP";case StartupMode::RACE:return"RACE";case StartupMode::SAFE_DECEL:return"SAFE_DECEL";default:return"STOPPED";}}
StartupStateMachine::StartupStateMachine(const YAML::Node& c,double cap,double rate):startup_(c["startup"]),fallback_(c["fallback"]),race_speed_cap_(cap),race_steering_rate_(rate){}
void StartupStateMachine::enter(StartupMode m,double now){mode_=m;entered_at_=now;if(m!=StartupMode::CRAWL_OBSERVE)observer_samples_=0;if(m!=StartupMode::SAFE_DECEL&&m!=StartupMode::STOPPED)recovery_solutions_=0;}
void StartupStateMachine::reset(double now){std::lock_guard<std::mutex>l(mutex_);mode_=StartupMode::BOOT;entered_at_=now;observer_samples_=recovery_solutions_=failures_=0;}
void StartupStateMachine::updateObserver(double now,bool valid,double confidence,bool pointlio){std::lock_guard<std::mutex>l(mutex_);if(mode_==StartupMode::BOOT)enter(StartupMode::STEER_SETTLE,now);if(mode_==StartupMode::STEER_SETTLE&&now-entered_at_>=required(startup_,"steer_settle_time_s"))enter(StartupMode::CRAWL_OBSERVE,now);else if(mode_==StartupMode::CRAWL_OBSERVE){bool q=valid&&pointlio&&confidence>required(startup_,"observer_confidence_threshold");observer_samples_=q?observer_samples_+1:0;if(observer_samples_>=startup_["observer_valid_samples_required"].as<int>())enter(StartupMode::MPCC_RAMP,now);}else if(mode_==StartupMode::MPCC_RAMP&&now-entered_at_>=required(startup_,"mpcc_ramp_time_s"))enter(StartupMode::RACE,now);}
void StartupStateMachine::noteSolver(bool ok,double now){std::lock_guard<std::mutex>l(mutex_);if(ok){failures_=0;if(mode_==StartupMode::SAFE_DECEL||mode_==StartupMode::STOPPED){if(++recovery_solutions_>=startup_["mpcc_valid_solutions_required"].as<int>())enter(StartupMode::MPCC_RAMP,now);}return;}recovery_solutions_=0;++failures_;if(mode_!=StartupMode::SAFE_DECEL&&mode_!=StartupMode::STOPPED&&failures_>=fallback_["enter_safe_decel_after_failures"].as<int>())enter(StartupMode::SAFE_DECEL,now);if(mode_==StartupMode::SAFE_DECEL&&failures_>=fallback_["stop_after_failures"].as<int>())enter(StartupMode::STOPPED,now);}
void StartupStateMachine::forceSafeDecel(double now){std::lock_guard<std::mutex>l(mutex_);if(mode_!=StartupMode::STOPPED)enter(StartupMode::SAFE_DECEL,now);} StartupMode StartupStateMachine::mode()const{std::lock_guard<std::mutex>l(mutex_);return mode_;}int StartupStateMachine::failureCount()const{std::lock_guard<std::mutex>l(mutex_);return failures_;}
StartupProfile StartupStateMachine::profile(double now)const{std::lock_guard<std::mutex>l(mutex_);double crawl=required(startup_,"crawl_speed_mps"),rate=required(startup_,"crawl_delta_rate_max_radps"),lim=required(startup_,"crawl_delta_max_rad");if(mode_==StartupMode::BOOT||mode_==StartupMode::STEER_SETTLE)return{mode_,0,0,rate,0,lim};if(mode_==StartupMode::CRAWL_OBSERVE)return{mode_,0,crawl,rate,0,lim};if(mode_==StartupMode::STOPPED)return{mode_,0,0,required(fallback_,"steering_recenter_rate_radps"),0};if(mode_==StartupMode::SAFE_DECEL)return{mode_,0,race_speed_cap_,required(fallback_,"steering_recenter_rate_radps"),0,std::numeric_limits<double>::infinity()};if(mode_==StartupMode::MPCC_RAMP){double z=clamp((now-entered_at_)/required(startup_,"mpcc_ramp_time_s"),0,1),r=z*z*(3-2*z);return{mode_,r,crawl+r*(race_speed_cap_-crawl),rate+r*(race_steering_rate_-rate),r};}return{StartupMode::RACE,1,race_speed_cap_,race_steering_rate_,1};}

CommandManager::CommandManager(const YAML::Node& c,double steer):publisher_(c["publisher"]),startup_(c["startup"]),fallback_(c["fallback"]),max_steer_(steer){
  const double normal=required(publisher_,"deceleration_limit_mps2");
  const double emergency=value(publisher_,"emergency_deceleration_limit_mps2",normal);
  if(normal<=0.0||emergency<normal)
    throw std::runtime_error(
      "publisher emergency_deceleration_limit_mps2 must be >= normal deceleration_limit_mps2");
}
void CommandManager::setSolution(const std::vector<State>& x,const std::vector<Control>& u,double now,bool emergency_deceleration){std::lock_guard<std::mutex>l(mutex_);desired_<<u[0][0],x[1][7],u[0][2];int si=x.size()>2?2:1,ui=u.size()>1?1:0;shifted_<<u[ui][0],x[si][7],u[ui][2];shifted_available_=true;desired_stamp_=now;solution_valid_=true;failures_=0;desired_source_="mpcc";emergency_deceleration_=emergency_deceleration;}
void CommandManager::setCandidate(double speed,double steering,double virtual_speed,double now,const std::string& source){std::lock_guard<std::mutex>l(mutex_);desired_<<speed,steering,virtual_speed;desired_stamp_=now;solution_valid_=true;failures_=0;shifted_available_=false;desired_source_=source;emergency_deceleration_=source.rfind("pure_pursuit",0)==0||source.rfind("checkpoint_",0)==0;}
void CommandManager::noteFailure(){std::lock_guard<std::mutex>l(mutex_);++failures_;solution_valid_=false;emergency_deceleration_=true;}
std::pair<double,bool> CommandManager::slew(double cur,double target,double rate,double dt){double m=std::max(rate,0.0)*dt,d=clamp(target-cur,-m,m);return{cur+d,std::abs(target-cur)>m+1e-12};}
PublishedCommand CommandManager::tick(double now,const StartupProfile& p){std::lock_guard<std::mutex>l(mutex_);double dt=last_publish_?now-*last_publish_:1.0/required(publisher_,"rate_hz");dt=clamp(dt,.001,required(publisher_,"max_dt_s"));last_publish_=now;bool fresh=now-desired_stamp_<=required(publisher_,"command_timeout_s");Control target=desired_;std::string source=desired_source_;double center=required(startup_,"steering_center_command_rad");if(p.mode==StartupMode::BOOT||p.mode==StartupMode::STEER_SETTLE){target<<0,center,0;source="steer_settle";}else if(p.mode==StartupMode::CRAWL_OBSERVE){target[0]=required(startup_,"crawl_speed_mps");target[1]=clamp(target[1],center-required(startup_,"crawl_delta_max_rad"),center+required(startup_,"crawl_delta_max_rad"));source="crawl_observe";}else if(p.mode==StartupMode::SAFE_DECEL||p.mode==StartupMode::STOPPED||!fresh){target<<0,center,0;source=p.mode==StartupMode::STOPPED?"stopped":"safe_decel";const bool hold=value(fallback_,"hold_last_steering_while_decelerating",true)&&p.mode!=StartupMode::STOPPED&&published_[0]>value(fallback_,"steering_recenter_below_speed_mps",.35);if(hold){target[1]=published_[1];source="safe_decel_hold_steering";}}else if(value(fallback_,"single_failure_use_shifted_solution",true)&&!solution_valid_&&failures_==1&&shifted_available_){target=shifted_;source="shifted_solution";}target[0]=clamp(target[0],0,p.speed_cap);target[1]=clamp(target[1],-max_steer_,max_steer_);if(std::isfinite(p.steering_limit))target[1]=clamp(target[1],-p.steering_limit,p.steering_limit);bool fallback=p.mode==StartupMode::SAFE_DECEL||p.mode==StartupMode::STOPPED||!fresh;const bool emergency_source=emergency_deceleration_||source.rfind("pure_pursuit",0)==0||source.rfind("checkpoint_",0)==0;const double normal_decel=required(publisher_,"deceleration_limit_mps2");const double emergency_decel=value(publisher_,"emergency_deceleration_limit_mps2",normal_decel);double ar=fallback?required(fallback_,"decel_mps2"):(target[0]>=published_[0]?required(publisher_,"acceleration_limit_mps2"):(emergency_source?emergency_decel:normal_decel));auto speed=slew(published_[0],target[0],ar,dt);double sr=std::min(required(publisher_,"steering_slew_rate_limit_radps"),p.steering_rate_cap);if(fallback)sr=std::min(sr,required(fallback_,"steering_recenter_rate_radps"));auto steer=slew(published_[1],target[1],sr,dt);published_<<speed.first,steer.first,std::max(target[2],0.0);return{published_[0],published_[1],published_[2],dt,speed.second||steer.second,source};}
Control CommandManager::published()const{std::lock_guard<std::mutex>l(mutex_);return published_;}

}  // namespace f1tenth_dynamic_mpcc
