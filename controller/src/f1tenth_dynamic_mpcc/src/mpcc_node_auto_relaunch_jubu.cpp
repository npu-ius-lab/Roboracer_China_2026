#include "f1tenth_dynamic_mpcc/acados_runtime_jubu.hpp"
#include "f1tenth_dynamic_mpcc/core.hpp"

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace mpcc = f1tenth_dynamic_mpcc;

namespace {
double yaw(const nav_msgs::Odometry& msg) {
  const auto& q=msg.pose.pose.orientation;
  return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
std::string normalizedFrame(std::string frame) {
  if(!frame.empty()&&frame[0]=='/')frame.erase(0,1);
  return frame;
}
std::string expandHome(std::string path) {
  if(!path.empty()&&path[0]=='~'){const char* home=std::getenv("HOME");if(home)path.replace(0,1,home);}return path;
}
double diagnosticValue(double value) {
  return std::isfinite(value)?value:999.0;
}
double circularProgressDistance(double left,double right,double length) {
  if(!(length>0.0))return std::numeric_limits<double>::infinity();
  return std::abs(std::remainder(left-right,length));
}
mpcc::TrackProjection headingAwareGlobalProject(
    const mpcc::PeriodicTrack& track,double x,double y,double vehicle_yaw) {
  constexpr int samples=4096;
  constexpr double maximum_heading_error=0.70;
  constexpr double heading_weight=0.50;
  bool found=false;double best_score=std::numeric_limits<double>::infinity();
  double best_s=0.0;
  for(int i=0;i<samples;++i){
    const double s=track.length()*i/samples;
    const auto position=track.position(s);
    const double heading=mpcc::wrapAngle(vehicle_yaw-track.tangent(s));
    if(std::abs(heading)>maximum_heading_error)continue;
    const double dx=position[0]-x,dy=position[1]-y;
    const double score=dx*dx+dy*dy+
      heading_weight*heading_weight*heading*heading;
    if(score<best_score){found=true;best_score=score;best_s=s;}
  }
  if(!found)return track.project(x,y);
  const auto refined=track.project(x,y,best_s);
  if(std::abs(mpcc::wrapAngle(vehicle_yaw-refined.psi_ref))>
      maximum_heading_error)return track.project(x,y);
  return refined;
}
}

class MpccNode {
 public:
  MpccNode():private_("~") {
    const std::string package=ros::package::getPath("f1tenth_dynamic_mpcc");
    std::string controller_path,vehicle_path,track_path,generated,formulation,mode;
    private_.param("controller_config",controller_path,package+"/config/controller.yaml");
    private_.param("vehicle_config",vehicle_path,package+"/config/vehicle.yaml");
    private_.param("track_csv",track_path,package+"/data/tracks/virtual_track/raceline.csv");
    private_.param("generated_dir",generated,expandHome("~/.cache/f1tenth_residual_mpcc/acados"));
    private_.param("formulation",formulation,std::string("baseline"));
    controller_=YAML::LoadFile(controller_path); vehicle_cfg_=YAML::LoadFile(vehicle_path);
    double cost_contour_override=-1.0,cost_heading_race_override=-1.0,
      cost_steering_rate_override=-1.0,profile_max_decel_override=-1.0;
    private_.param("cost_contour_override",cost_contour_override,-1.0);
    private_.param("cost_heading_race_override",cost_heading_race_override,-1.0);
    private_.param("cost_steering_command_rate_override",cost_steering_rate_override,-1.0);
    if(cost_contour_override>0.0)controller_["cost"]["contour"]=cost_contour_override;
    if(cost_heading_race_override>0.0)controller_["cost"]["heading_race"]=cost_heading_race_override;
    if(cost_steering_rate_override>0.0)
      controller_["cost"]["steering_command_rate"]=cost_steering_rate_override;
    private_.param("profile_max_decel_override",profile_max_decel_override,-1.0);
    if(profile_max_decel_override>0.0)
      controller_["speed_planning"]["profile_max_decel_mps2"]=profile_max_decel_override;
    private_.param("risk_tiering_enabled",risk_tiering_enabled_,false);
    double near_horizon_override=-1.0,speed_hold_override=-1.0,
      speed_recovery_override=-1.0,current_margin_override=-1.0;
    int confirmation_cycles_override=-1;
    private_.param("near_track_horizon_override",near_horizon_override,-1.0);
    private_.param("predictive_speed_hold_override",speed_hold_override,-1.0);
    private_.param("predictive_speed_recovery_override",speed_recovery_override,-1.0);
    private_.param("pp_current_margin_override",current_margin_override,-1.0);
    private_.param("pp_confirmation_cycles_override",confirmation_cycles_override,-1);
    if(near_horizon_override>0.0)
      controller_["safety"]["near_track_horizon_s"]=near_horizon_override;
    if(speed_hold_override>=0.0)
      controller_["safety"]["predictive_speed_hold_s"]=speed_hold_override;
    if(speed_recovery_override>0.0)
      controller_["safety"]["predictive_speed_recovery_mps2"]=speed_recovery_override;
    if(current_margin_override>=0.0)
      controller_["pure_pursuit_candidate"]["activation_current_margin_m"]=current_margin_override;
    if(confirmation_cycles_override>0)
      controller_["pure_pursuit_candidate"]["prediction_risk_confirmation_cycles"]=confirmation_cycles_override;
    double startup_steering_rate_override=-1.0,publisher_steering_rate_override=-1.0,
      pp_steering_rate_override=-1.0;
    private_.param("startup_steering_rate_override",startup_steering_rate_override,-1.0);
    private_.param("publisher_steering_rate_override",publisher_steering_rate_override,-1.0);
    private_.param("pp_steering_rate_override",pp_steering_rate_override,-1.0);
    if(startup_steering_rate_override>0.0)
      controller_["startup"]["steering_rate_max_radps"]=startup_steering_rate_override;
    if(publisher_steering_rate_override>0.0)
      controller_["publisher"]["steering_slew_rate_limit_radps"]=publisher_steering_rate_override;
    if(pp_steering_rate_override>0.0)
      controller_["pure_pursuit_candidate"]["max_steering_rate_radps"]=pp_steering_rate_override;
    private_.param("oob_recovery_enabled",oob_recovery_enabled_,false);
    std::string residual_model_override,residual_feature_override;
    private_.param("residual_model_path",residual_model_override,std::string());
    private_.param("residual_feature_set",residual_feature_override,std::string());
    if(!residual_model_override.empty())
      controller_["residual_dynamics"]["model_path"]=residual_model_override;
    if(!residual_feature_override.empty())
      controller_["residual_dynamics"]["required_feature_set"]=residual_feature_override;
    private_.param("mode",mode,controller_["mode"].as<std::string>());
    if(mode!="debug"&&mode!="race")throw std::invalid_argument("mode must be debug or race");
    controller_["mode"]=mode;
    const auto residual_cfg=controller_["residual_dynamics"];
    if(residual_cfg&&residual_cfg["enabled"].as<bool>()){
      std::string residual_path=residual_cfg["model_path"].as<std::string>();
      if(residual_path.empty()||residual_path.front()!='/')residual_path=package+"/"+residual_path;
      residual_model_=std::make_shared<f1tenth_residual_dynamics::ResidualModel>(
        f1tenth_residual_dynamics::ResidualModel::load(residual_path));
      const std::string required=residual_cfg["required_feature_set"]
        ?residual_cfg["required_feature_set"].as<std::string>():std::string();
      if(!required.empty()&&residual_model_->featureSet()!=required)
        throw std::runtime_error("residual model feature_set "+residual_model_->featureSet()+
          " does not match required "+required);
      ROS_INFO("Residual dynamics loaded model=%s schema=%d feature_set=%s features=%zu",
               residual_path.c_str(),residual_model_->schemaVersion(),
               residual_model_->featureSet().c_str(),residual_model_->featureCount());
    }
    vehicle_=mpcc::VehicleParameters::fromYaml(vehicle_cfg_,controller_);
    auto speed=controller_["speed_planning"];
    speed["wheelbase_m"]=vehicle_.wheelbase;speed["max_steer_rad"]=vehicle_.max_steer;
    speed["max_steer_rate_radps"]=vehicle_.max_steer_rate;speed["max_accel_mps2"]=vehicle_.max_accel;
    speed["max_decel_mps2"]=vehicle_.max_decel;speed["lateral_accel_limit_mps2"]=vehicle_.lateral_accel_limit;
    track_=std::make_unique<mpcc::PeriodicTrack>(track_path,speed,vehicle_);
    model_=std::make_unique<mpcc::DynamicBicycleModel>(vehicle_,5,residual_model_);
    low_speed_conditioner_=std::make_unique<mpcc::LowSpeedStateConditioner>(
      controller_["low_speed_state_conditioning"],vehicle_.wheelbase);
    private_.param("race_start_enabled",race_start_enabled_,true);
    private_.param("race_start_center_x_m",race_start_center_x_,0.0);
    private_.param("race_start_center_y_m",race_start_center_y_,0.0);
    private_.param("race_start_radius_m",race_start_radius_,2.0);
    private_.param("race_start_s_m",race_start_s_,0.0);
    private_.param("race_start_s_tolerance_m",race_start_s_tolerance_,1.5);
    private_.param("race_start_maximum_contour_error_m",
                   race_start_maximum_contour_error_,0.60);
    double race_start_maximum_heading_error_deg=20.0;
    private_.param("race_start_maximum_heading_error_deg",
                   race_start_maximum_heading_error_deg,20.0);
    race_start_maximum_heading_error_=
      race_start_maximum_heading_error_deg*M_PI/180.0;
    private_.param("race_start_minimum_vehicle_margin_m",
                   race_start_minimum_vehicle_margin_,-0.05);
    private_.param("race_start_merge_lookahead_m",
                   race_start_merge_lookahead_,2.50);
    private_.param("race_start_speed_mps",race_start_speed_,4.0);
    private_.param("race_start_lateral_acceleration_limit_mps2",
                   race_start_lateral_acceleration_limit_,3.20);
    if(race_start_radius_<=0.0||race_start_s_tolerance_<=0.0||
       race_start_maximum_contour_error_<=0.0||
       race_start_maximum_heading_error_<=0.0||
       race_start_merge_lookahead_<=0.0||race_start_speed_<=0.0||
       race_start_lateral_acceleration_limit_<=0.0)
      throw std::invalid_argument("invalid C++ race-start configuration");
    pure_pursuit_candidate_=std::make_unique<mpcc::PurePursuitCandidate>(
      controller_["pure_pursuit_candidate"],*track_,*model_,
      controller_["publisher"]["acceleration_limit_mps2"].as<double>(),
      controller_["publisher"]["deceleration_limit_mps2"].as<double>());
    // The normal checkpoint candidate deliberately slows down for arbitrary
    // carried poses. A recognized competition-grid start uses a separate
    // candidate with the wider 20-degree/0.60 m launch envelope, a 2.5 m
    // merge lookahead and the normal 3.2 m/s^2 race lateral limit. Its rollout
    // still passes the same footprint and corridor safety manager.
    YAML::Node race_start_candidate_config=
      YAML::Clone(controller_["pure_pursuit_candidate"]);
    race_start_candidate_config["max_speed_mps"]=race_start_speed_;
    race_start_candidate_config["lateral_accel_limit_mps2"]=
      race_start_lateral_acceleration_limit_;
    race_start_candidate_config["lookahead_base_m"]=race_start_merge_lookahead_;
    race_start_candidate_config["lookahead_speed_gain_s"]=0.0;
    race_start_candidate_config["lookahead_lateral_gain"]=0.0;
    race_start_candidate_config["lookahead_min_m"]=race_start_merge_lookahead_;
    race_start_candidate_config["lookahead_max_m"]=race_start_merge_lookahead_;
    race_start_candidate_config["lateral_error_soft_m"]=
      race_start_maximum_contour_error_;
    race_start_candidate_config["lateral_error_hard_m"]=
      race_start_maximum_contour_error_+0.10;
    race_start_candidate_config["heading_error_soft_rad"]=
      race_start_maximum_heading_error_;
    race_start_candidate_config["heading_error_hard_rad"]=
      race_start_maximum_heading_error_+0.10;
    race_start_candidate_=std::make_unique<mpcc::PurePursuitCandidate>(
      race_start_candidate_config,*track_,*model_,
      controller_["publisher"]["acceleration_limit_mps2"].as<double>(),
      controller_["publisher"]["deceleration_limit_mps2"].as<double>());
    checkpoint_recovery_=std::make_unique<mpcc::CheckpointRecoveryManager>(
      controller_["pure_pursuit_candidate"]["checkpoint_recovery"]);
    pp_current_margin_=controller_["pure_pursuit_candidate"]
      ["activation_current_margin_m"].as<double>();
    pp_confirmation_cycles_=controller_["pure_pursuit_candidate"]
      ["prediction_risk_confirmation_cycles"].as<int>();
    if(pp_current_margin_<0.0||pp_confirmation_cycles_<1)
      throw std::invalid_argument("invalid tiered PP risk settings");
    oob_recovery_speed_=controller_["pure_pursuit_candidate"]
      ["recovery_speed_min_mps"].as<double>();
    oob_recovery_rate_=controller_["startup"]["steering_rate_max_radps"].as<double>();
    candidate_arbitrator_=std::make_unique<mpcc::CandidateArbitrator>(
      controller_["pure_pursuit_candidate"]["arbitration"]);
    predictor_=std::make_unique<mpcc::LowLatencyPredictor>(*model_,command_history_);
    observer_=std::make_unique<mpcc::SteeringObserver>(vehicle_cfg_,vehicle_);
    const auto safety=controller_["safety"],startup=controller_["startup"];
    double race_cap=std::min(safety["global_speed_max_mps"].as<double>(),
      formulation=="baseline"?safety["baseline_speed_max_mps"].as<double>():vehicle_.max_speed);
    double launch_speed_cap=race_cap;
    private_.param("runtime_speed_cap_mps",launch_speed_cap,race_cap);
    race_cap=std::min(race_cap,std::max(0.0,launch_speed_cap));
    startup_=std::make_unique<mpcc::StartupStateMachine>(controller_,race_cap,
      std::min(vehicle_.max_steer_rate,startup["steering_rate_max_radps"].as<double>()));
    command_manager_=std::make_unique<mpcc::CommandManager>(controller_,vehicle_.max_steer);
    predictive_speed_limiter_=std::make_unique<mpcc::PredictiveSpeedLimiter>(
      safety["predictive_speed_hold_s"].as<double>(),
      safety["predictive_speed_recovery_mps2"].as<double>());
    solver_=std::make_unique<mpcc::AcadosRuntimeSolverJubu>(formulation,expandHome(generated),*track_,vehicle_,controller_,residual_model_);
    formulation_=formulation; runtime_speed_cap_=race_cap;
    const auto topics=controller_["topics"],frames=controller_["frames"],timing=controller_["timing"],prediction=controller_["localization_prediction"];
    std::string telemetry_topic,markers_topic,prediction_topic;
    private_.param("odom_topic",odom_topic_,topics["odom"].as<std::string>());
    private_.param("command_topic",command_topic_,topics["command"].as<std::string>());
    private_.param("collision_topic",collision_topic_,topics["collision"].as<std::string>());
    private_.param("telemetry_topic",telemetry_topic,topics["telemetry"].as<std::string>());
    private_.param("markers_topic",markers_topic,topics["markers"].as<std::string>());
    private_.param("prediction_topic",prediction_topic,topics["prediction_markers"].as<std::string>());
    private_.param("local_reference_topic",local_reference_topic_,
                   std::string("/v5_jubu/local_reference"));
    private_.param("local_speed_cap_topic",local_speed_cap_topic_,
                   std::string("/v5_jubu/local_speed_cap"));
    private_.param("local_state_topic",local_state_topic_,
                   std::string("/v5_jubu/state"));
    private_.param("local_reference_status_topic",local_reference_status_topic_,
                   std::string("/v5_jubu/mpcc_reference_status"));
    private_.param("planner_health_topic",planner_health_topic_,
                   std::string("/v5_jubu/health"));
    private_.param("planner_required",planner_required_,true);
    private_.param("planner_health_timeout_s",planner_health_timeout_,0.50);
    private_.param("local_reference_timeout_s",local_reference_timeout_,0.75);
    private_.param("local_reference_max_start_distance_m",
                   local_reference_max_start_distance_,0.75);
    private_.param("local_reference_min_length_m",local_reference_min_length_,1.5);
    private_.param("local_reference_max_curvature_ratio",
                   local_reference_max_curvature_ratio_,0.90);
    private_.param("local_reference_return_tolerance_m",
                   local_reference_return_tolerance_,0.15);
    private_.param("local_reference_default_speed_cap_mps",
                   local_reference_speed_cap_,2.5);
    if(planner_health_timeout_<=0.0||local_reference_timeout_<=0.0||
       local_reference_max_start_distance_<=0.0||local_reference_min_length_<=0.0||
       local_reference_max_curvature_ratio_<=0.0||
       local_reference_max_curvature_ratio_>1.0||
       local_reference_return_tolerance_<0.0||local_reference_speed_cap_<=0.0)
      throw std::invalid_argument("invalid V5 jubu local-reference parameters");
    private_.param("world_frame",world_frame_,frames["world"].as<std::string>());
    private_.param("body_frame",body_frame_,frames["body"].as<std::string>());
    private_.param("collision_required",collision_required_,safety["collision_required"].as<bool>());
    bool requested=false;private_.param("start_enabled",requested,false);enabled_=safety["start_enabled"].as<bool>()&&requested;
    control_rate_=timing["control_rate_hz"].as<double>(); odom_timeout_=timing["odom_timeout_s"].as<double>();
    max_reprop_=prediction["max_repropagation_s"].as<double>(); warning_age_=prediction["warning_age_s"].as<double>();hard_age_=prediction["hard_age_s"].as<double>();
    benign_reorder_tolerance_=prediction["benign_reorder_tolerance_s"].as<double>();
    benign_reorder_position_tolerance_=prediction["benign_reorder_position_tolerance_m"].as<double>();
    benign_reorder_yaw_tolerance_=prediction["benign_reorder_yaw_tolerance_rad"].as<double>();
    observer_timeout_=vehicle_cfg_["steering_observer"]["dropout_timeout_s"].as<double>();
    latency_speed_limit_=safety["latency_speed_limit_mps"].as<double>(); model_only_limit_=safety["observer_model_only_speed_limit_mps"].as<double>();invalid_limit_=safety["observer_invalid_speed_limit_mps"].as<double>();collision_timeout_=safety["collision_timeout_s"].as<double>();
    command_pub_=node_.advertise<ackermann_msgs::AckermannDriveStamped>(command_topic_,1);
    diagnostic_pub_=node_.advertise<std_msgs::Float32MultiArray>(telemetry_topic,5);
    marker_pub_=node_.advertise<visualization_msgs::MarkerArray>(markers_topic,1,true);
    prediction_pub_=node_.advertise<visualization_msgs::MarkerArray>(prediction_topic,1);
    local_reference_status_pub_=node_.advertise<std_msgs::String>(
      local_reference_status_topic_,1,true);
    odom_sub_=node_.subscribe(odom_topic_,1,&MpccNode::odomCallback,this,ros::TransportHints().tcpNoDelay());
    imu_sub_=node_.subscribe("/livox/imu",100,&MpccNode::imuCallback,this,ros::TransportHints().tcpNoDelay());
    wheel_sub_=node_.subscribe(topics["wheel_odom"].as<std::string>(),20,&MpccNode::wheelCallback,this,ros::TransportHints().tcpNoDelay());
    if(collision_required_)collision_sub_=node_.subscribe(collision_topic_,1,&MpccNode::collisionCallback,this);
    local_reference_sub_=node_.subscribe(local_reference_topic_,1,
      &MpccNode::localReferenceCallback,this);
    local_speed_cap_sub_=node_.subscribe(local_speed_cap_topic_,1,
      &MpccNode::localSpeedCapCallback,this);
    local_state_sub_=node_.subscribe(local_state_topic_,1,
      &MpccNode::localStateCallback,this);
    planner_health_sub_=node_.subscribe(planner_health_topic_,1,
      &MpccNode::plannerHealthCallback,this);
    enable_service_=private_.advertiseService("set_enabled",&MpccNode::enableCallback,this);
    startup_->reset(ros::Time::now().toSec()); publishStop(); publishTrackMarkers();
    control_timer_=node_.createTimer(ros::Duration(1/control_rate_),&MpccNode::controlTick,this);
    publisher_timer_=node_.createTimer(ros::Duration(1/controller_["publisher"]["rate_hz"].as<double>()),&MpccNode::publisherTick,this);
    publishLocalReferenceStatus("GLOBAL",false,"startup");
    ROS_INFO("C++ Residual Dynamic MPCC V5 JUBU ready formulation=%s mode=%s odom=%s command=%s enabled=%s speed_cap=%.2f contour=%.2f heading=%.2f steering_rate_cost=%.2f tiered_risk=%s near=%.2fs hold=%.2fs recovery=%.2fm/s2 steer_rate=%.2frad/s oob_recovery=%s local_ref=%s",
             formulation_.c_str(),mode.c_str(),odom_topic_.c_str(),command_topic_.c_str(),enabled_?"true":"false",race_cap,
             controller_["cost"]["contour"].as<double>(),
             mode=="race"?controller_["cost"]["heading_race"].as<double>():controller_["cost"]["heading_debug"].as<double>(),
             controller_["cost"]["steering_command_rate"].as<double>(),
             risk_tiering_enabled_?"true":"false",
             controller_["safety"]["near_track_horizon_s"].as<double>(),
             controller_["safety"]["predictive_speed_hold_s"].as<double>(),
             controller_["safety"]["predictive_speed_recovery_mps2"].as<double>(),
             std::min(vehicle_.max_steer_rate,
               controller_["startup"]["steering_rate_max_radps"].as<double>()),
             oob_recovery_enabled_?"true":"false",local_reference_topic_.c_str());
  }
  ~MpccNode(){publishStop();}

 private:
  void publishLocalReferenceStatus(const std::string& state,bool active,
                                   const std::string& reason) {
    std_msgs::String message;
    std::ostringstream stream;
    stream<<"state="<<state<<" reference="<<(active?"LOCAL_MPCC":"GLOBAL_MPCC")
          <<" reason="<<reason;
    message.data=stream.str();
    local_reference_status_pub_.publish(message);
  }

  std::shared_ptr<const mpcc::LocalReferenceProfile> buildLocalReference(
      const nav_msgs::Path& path,double progress_guess,
      std::uint64_t& signature) const {
    if(path.poses.size()<8)throw std::invalid_argument("local path has fewer than 8 poses");
    std::string frame=normalizedFrame(path.header.frame_id);
    if(frame.empty())frame=normalizedFrame(path.poses.front().header.frame_id);
    if(frame!=world_frame_)throw std::invalid_argument(
      "local path frame must match "+world_frame_+", got "+frame);

    std::optional<mpcc::State> odom;
    {std::lock_guard<std::mutex> lock(mutex_);odom=odom_;}
    if(!odom)throw std::invalid_argument("local path rejected before valid odometry");
    const double start_dx=path.poses.front().pose.position.x-(*odom)[0];
    const double start_dy=path.poses.front().pose.position.y-(*odom)[1];
    if(std::hypot(start_dx,start_dy)>local_reference_max_start_distance_)
      throw std::invalid_argument("local path starts too far from the vehicle");

    std::vector<mpcc::LocalReferencePoint> points;
    points.reserve(path.poses.size());
    double previous_s=progress_guess;
    constexpr double minimum_ds=0.015;
    signature=1469598103934665603ULL;
    auto mix=[&signature](std::int64_t value){
      signature^=static_cast<std::uint64_t>(value);
      signature*=1099511628211ULL;
    };
    for(const auto& stamped:path.poses){
      const double x=stamped.pose.position.x,y=stamped.pose.position.y;
      if(!std::isfinite(x)||!std::isfinite(y))continue;
      const auto projection=track_->project(x,y,previous_s);
      double s=projection.s;
      while(s<previous_s-track_->length()/2.0)s+=track_->length();
      while(s>previous_s+track_->length()/2.0)s-=track_->length();
      if(!points.empty()&&s<=previous_s+minimum_ds)continue;
      const auto global=track_->geometry(s);
      const double nx=-std::sin(global.yaw),ny=std::cos(global.yaw);
      const double ey=(x-global.x)*nx+(y-global.y)*ny;
      mpcc::TrackGeometry geometry=global;
      geometry.x=x;geometry.y=y;
      geometry.width_left=global.width_left-ey;
      geometry.width_right=global.width_right+ey;
      geometry.speed_prior=global.speed_prior;
      if(geometry.width_left<=0.02||geometry.width_right<=0.02)
        throw std::invalid_argument("local path leaves the physical track corridor");
      points.push_back({s,geometry});
      previous_s=s;
      mix(static_cast<std::int64_t>(std::llround(x*1000.0)));
      mix(static_cast<std::int64_t>(std::llround(y*1000.0)));
    }
    if(points.size()<8)throw std::invalid_argument(
      "local path has fewer than 8 forward-progress poses");
    if(points.back().s-points.front().s<local_reference_min_length_)
      throw std::invalid_argument("local path is shorter than the minimum length");
    for(std::size_t i=0;i<points.size();++i){
      const std::size_t before=i==0?0:i-1;
      const std::size_t after=i+1<points.size()?i+1:points.size()-1;
      const double dx=points[after].geometry.x-points[before].geometry.x;
      const double dy=points[after].geometry.y-points[before].geometry.y;
      points[i].geometry.yaw=std::atan2(dy,dx);
    }
    double maximum_curvature=0.0;
    for(std::size_t i=0;i<points.size();++i){
      const std::size_t before=i==0?0:i-1;
      const std::size_t after=i+1<points.size()?i+1:points.size()-1;
      const double dx=points[after].geometry.x-points[before].geometry.x;
      const double dy=points[after].geometry.y-points[before].geometry.y;
      const double ds=std::hypot(dx,dy);
      points[i].geometry.curvature=ds>1e-6
        ?mpcc::wrapAngle(points[after].geometry.yaw-
                        points[before].geometry.yaw)/ds:0.0;
      maximum_curvature=std::max(
        maximum_curvature,std::abs(points[i].geometry.curvature));
    }
    const double vehicle_curvature_limit=
      std::tan(vehicle_.max_steer)/vehicle_.wheelbase;
    if(maximum_curvature>
       local_reference_max_curvature_ratio_*vehicle_curvature_limit)
      throw std::invalid_argument("local path exceeds Ackermann curvature limit");
    const auto final_global=track_->geometry(points.back().s);
    const double final_nx=-std::sin(final_global.yaw);
    const double final_ny= std::cos(final_global.yaw);
    const double final_ey=(points.back().geometry.x-final_global.x)*final_nx+
                          (points.back().geometry.y-final_global.y)*final_ny;
    if(std::abs(final_ey)>local_reference_return_tolerance_)
      throw std::invalid_argument("local path does not return to the global raceline");
    mix(static_cast<std::int64_t>(points.size()));
    return std::make_shared<const mpcc::LocalReferenceProfile>(
      std::move(points),track_->length());
  }

  void clearLocalReference(const std::string& reason) {
    bool was_active=false;
    {std::lock_guard<std::mutex> lock(mutex_);was_active=local_reference_active_;}
    if(!was_active)return;
    {
      std::lock_guard<std::mutex> solve_lock(solve_mutex_);
      solver_->clearLocalReference();
      solver_->resetWarmStart();
      candidate_arbitrator_->reset();
    }
    std::string state;
    {std::lock_guard<std::mutex> lock(mutex_);
      local_reference_active_=false;local_reference_signature_=0;
      state=local_state_;}
    publishLocalReferenceStatus(state,false,reason);
  }

  void localReferenceCallback(const nav_msgs::Path::ConstPtr& message) {
    if(message->poses.empty()){
      clearLocalReference("manager_cleared_path");
      return;
    }
    double guess=0.0;
    {std::lock_guard<std::mutex> lock(mutex_);
      guess=theta_guess_.value_or(0.0);}
    try{
      std::uint64_t signature=0;
      auto reference=buildLocalReference(*message,guess,signature);
      bool unchanged=false;
      {std::lock_guard<std::mutex> lock(mutex_);
        unchanged=local_reference_active_&&signature==local_reference_signature_;}
      if(!unchanged){
        std::lock_guard<std::mutex> solve_lock(solve_mutex_);
        if(checkpoint_recovery_->armed())
          throw std::invalid_argument(
            "local path rejected during checkpoint recovery");
        solver_->setLocalReference(std::move(reference));
        solver_->resetWarmStart();
        candidate_arbitrator_->reset();
      }
      std::string state;
      {std::lock_guard<std::mutex> lock(mutex_);
        local_reference_active_=true;local_reference_signature_=signature;
        local_reference_stamp_=ros::Time::now().toSec();state=local_state_;}
      publishLocalReferenceStatus(state,true,unchanged?"reference_refreshed":"reference_accepted");
    }catch(const std::exception& error){
      ROS_ERROR_THROTTLE(1,"V5 jubu local path rejected: %s",error.what());
      std::string state;
      {std::lock_guard<std::mutex> lock(mutex_);state=local_state_;}
      publishLocalReferenceStatus(state,solver_->localReferenceActive(),error.what());
    }
  }

  void localSpeedCapCallback(const std_msgs::Float32::ConstPtr& message) {
    if(!std::isfinite(message->data)||message->data<0.0){
      ROS_WARN_THROTTLE(1,"Ignoring invalid V5 jubu speed cap %.3f",message->data);
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    local_reference_speed_cap_=std::min<double>(message->data,vehicle_.max_speed);
  }

  void localStateCallback(const std_msgs::String::ConstPtr& message) {
    static const std::vector<std::string> allowed={
      "GLOBAL","FREE","FOLLOW","PREPARE","PASS","RETURN","ABORT"};
    if(std::find(allowed.begin(),allowed.end(),message->data)==allowed.end()){
      ROS_WARN_THROTTLE(1,"Ignoring invalid V5 jubu state %s",message->data.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      local_state_=message->data;
    }
    if(message->data=="GLOBAL"||message->data=="FREE"||
       message->data=="FOLLOW"||
       message->data=="PREPARE"||message->data=="ABORT")
      clearLocalReference("state_"+message->data);
  }

  void plannerHealthCallback(const std_msgs::Bool::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    planner_healthy_=message->data;
    planner_health_stamp_=ros::Time::now().toSec();
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){double t=msg->header.stamp.toSec(),r=msg->angular_velocity.z;if(std::isfinite(t)&&std::isfinite(r))predictor_->pushImu(t,r);}
  void wheelCallback(const nav_msgs::Odometry::ConstPtr& msg){double t=msg->header.stamp.toSec(),v=msg->twist.twist.linear.x;if(std::isfinite(t)&&std::isfinite(v))predictor_->pushWheel(t,v);}
  void collisionCallback(const std_msgs::Bool::ConstPtr& msg){std::lock_guard<std::mutex>l(mutex_);collision_=msg->data;collision_stamp_=ros::WallTime::now().toSec();}
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    std::string frame=msg->header.frame_id,child=msg->child_frame_id;if(!frame.empty()&&frame[0]=='/')frame.erase(0,1);if(!child.empty()&&child[0]=='/')child.erase(0,1);
    if(frame!=world_frame_||child!=body_frame_){ROS_ERROR_THROTTLE(1,"Rejecting odometry frames %s -> %s",frame.c_str(),child.c_str());return;}
    double stamp=msg->header.stamp.isZero()?ros::Time::now().toSec():msg->header.stamp.toSec();mpcc::State next=mpcc::State::Zero();next<<msg->pose.pose.position.x,msg->pose.pose.position.y,yaw(*msg),msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.angular.z,0,0,0;if(!next.allFinite()){observer_->invalidate();return;}
    std::optional<mpcc::State> previous;std::optional<double> previous_stamp,guess;{std::lock_guard<std::mutex>l(mutex_);previous=odom_;previous_stamp=odom_stamp_;guess=theta_guess_;}
    bool valid=!previous_stamp||stamp>*previous_stamp;bool regression=!valid,jump=false;double distance=0.0,dyaw=0.0;if(previous){distance=(next.segment<2>(0)-previous->segment<2>(0)).norm();dyaw=std::abs(mpcc::wrapAngle(next[2]-(*previous)[2]));jump=distance>controller_["startup"]["localization_jump_distance_m"].as<double>()||dyaw>controller_["startup"]["localization_jump_yaw_rad"].as<double>();valid=valid&&!jump;}
    const double backwards=previous_stamp?*previous_stamp-stamp:-1.0;
    if(regression&&backwards>=0.0&&backwards<=benign_reorder_tolerance_&&distance<=benign_reorder_position_tolerance_&&dyaw<=benign_reorder_yaw_tolerance_){ROS_WARN_THROTTLE(5,"Dropping benign PointLIO reorder dt=%.6fs dpos=%.4fm dyaw=%.4frad",stamp-*previous_stamp,distance,dyaw);return;}
    if(!valid){observer_->invalidate();std::lock_guard<std::mutex>l(mutex_);pointlio_valid_=false;if(jump&&!regression){auto p=headingAwareGlobalProject(*track_,next[0],next[1],next[2]);next[8]=p.s;odom_=next;odom_stamp_=stamp;theta_guess_=p.s;checkpoint_recovery_rearm_.store(true);}if(enabled_)startup_->forceSafeDecel(ros::Time::now().toSec());ROS_WARN_THROTTLE(1,"Rejecting PointLIO regression or pose jump");return;}
    auto estimate=observer_->update(stamp,next[3],next[4],next[5]);auto projection=track_->project(next[0],next[1],guess);next[6]=estimate.delta;next[7]=command_manager_->published()[1];next[8]=projection.s;{
      std::lock_guard<std::mutex>l(mutex_);odom_=next;odom_stamp_=stamp;odom_receive_=ros::Time::now().toSec();theta_guess_=projection.s;pointlio_valid_=true;}
    startup_->updateObserver(ros::Time::now().toSec(),estimate.valid,estimate.confidence,true);
  }
  bool enableCallback(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res){std::lock_guard<std::mutex>l(mutex_);if(req.data&&!odom_){res.success=false;res.message="valid odometry required";return true;}enabled_=req.data;oob_recovery_active_=false;predictive_speed_limiter_->reset();candidate_arbitrator_->reset();near_risk_cycles_=0;checkpoint_recovery_rearm_.store(req.data);if(enabled_)startup_->reset(ros::Time::now().toSec());else{last_control_.setZero();predictor_->clear();}res.success=true;res.message=enabled_?"enabled":"disabled";return true;}
  void controlTick(const ros::TimerEvent&){
    std::unique_lock<std::mutex> solve_lock(solve_mutex_,std::try_to_lock);
    if(!solve_lock.owns_lock()){
      const auto skipped=++skipped_solve_ticks_;
      ROS_WARN_THROTTLE(1,"Skipping overlapping MPCC solve tick count=%lu",
                        static_cast<unsigned long>(skipped));
      return;
    }
    double now=ros::Time::now().toSec();std::optional<mpcc::State> measured;double stamp=0,collision_stamp,planner_health_stamp;bool collision,valid,enabled,local_mpcc_active=false,local_reference_stale=false,planner_healthy=false;double local_speed_cap=0.0;std::string local_state;mpcc::Control fallback;{std::lock_guard<std::mutex>l(mutex_);measured=odom_;stamp=odom_stamp_.value_or(0);collision=collision_;collision_stamp=collision_stamp_;valid=pointlio_valid_;enabled=enabled_;fallback=last_control_;local_mpcc_active=local_reference_active_;local_reference_stale=local_mpcc_active&&now-local_reference_stamp_>local_reference_timeout_;local_speed_cap=local_reference_speed_cap_;local_state=local_state_;planner_healthy=planner_healthy_;planner_health_stamp=planner_health_stamp_;}
    const bool planner_stale=now-planner_health_stamp>planner_health_timeout_;
    if(planner_required_&&(!planner_healthy||planner_stale)){
      if(enabled){command_manager_->noteFailure();startup_->forceSafeDecel(now);}
      if(planner_health_stamp<=0.0){
        ROS_WARN_THROTTLE(1,"V5 JUBU waiting for initial planner health");
      }else{
        ROS_ERROR_THROTTLE(1,
          "V5 JUBU fail-closed: planner health=%s age=%.3f",
          planner_healthy?"true":"false",now-planner_health_stamp);
      }
      return;
    }
    if(local_state=="ABORT"||local_speed_cap<=0.0){
      if(enabled){command_manager_->noteFailure();startup_->forceSafeDecel(now);}
      ROS_ERROR_THROTTLE(1,"V5 JUBU fail-closed: state=%s speed_cap=%.2f",
                         local_state.c_str(),local_speed_cap);
      return;
    }
    if(local_reference_stale){
      solver_->clearLocalReference();solver_->resetWarmStart();candidate_arbitrator_->reset();
      {std::lock_guard<std::mutex>l(mutex_);local_reference_active_=false;local_reference_signature_=0;}
      local_mpcc_active=false;
      publishLocalReferenceStatus(local_state,false,"reference_timeout");
      if(local_state=="PASS"||local_state=="RETURN"){
        if(enabled){command_manager_->noteFailure();startup_->forceSafeDecel(now);}
        ROS_ERROR_THROTTLE(1,
          "V5 JUBU fail-closed: committed local reference timed out");
        return;
      }
      ROS_WARN_THROTTLE(1,"V5 JUBU local reference timed out before commitment");
    }else{
      local_mpcc_active=local_mpcc_active&&solver_->localReferenceActive();
    }
    if(!enabled||!measured)return;
    if(!valid||(collision_required_&&(collision||ros::WallTime::now().toSec()-collision_stamp>collision_timeout_))){command_manager_->noteFailure();startup_->forceSafeDecel(now);return;}double age=now-stamp;if(age>std::min(odom_timeout_,hard_age_)||age<-.02){command_manager_->noteFailure();startup_->forceSafeDecel(now);ROS_WARN_THROTTLE(1,"C++ MPCC stopping: odom age %.3f",age);return;}
    mpcc::SolverResult result;mpcc::State compensated;mpcc::TrackProjection projection;
    const mpcc::State raw_pointlio=*measured;
    mpcc::LowSpeedConditioningResult conditioned_measurement,conditioned_committed;
    mpcc::PurePursuitCandidateResult candidate;
    mpcc::TrajectorySafety mpcc_safety,mpcc_near_safety;
    mpcc::CandidateSelection candidate_selection;
    mpcc::CheckpointRecoveryDecision checkpoint_selection;
    bool prediction_risk=false,pp_trigger_risk=false,
      near_prediction_risk=false,far_prediction_risk=false,
      speed_only_risk=false,checkpoint_selected=false,
      checkpoint_blocked=false;
    double state_prediction_horizon=0.0,committed_horizon=0.0;
    try{
      const double published_speed=command_manager_->published()[0];
      conditioned_measurement=low_speed_conditioner_->updateAndCondition(
        raw_pointlio,published_speed);
      if(conditioned_measurement.entered){
        solver_->resetWarmStart();
        ROS_INFO("Low-speed conditioning ENTER raw_vx=%.3f raw_vy=%.3f raw_r=%.3f command=%.3f",
          raw_pointlio[3],raw_pointlio[4],raw_pointlio[5],published_speed);
      }else if(conditioned_measurement.exited){
        ROS_INFO("Low-speed conditioning EXIT raw_vx=%.3f command=%.3f",
          raw_pointlio[3],published_speed);
      }
      if(conditioned_measurement.stationary&&
         (std::abs(raw_pointlio[4])>.05||std::abs(raw_pointlio[5])>.05))
        ROS_WARN_THROTTLE(2,"Suppressing stationary PointLIO twist noise raw_vx=%.3f raw_vy=%.3f raw_r=%.3f",
          raw_pointlio[3],raw_pointlio[4],raw_pointlio[5]);
      auto predicted=predictor_->correctAndRepropagate(
        conditioned_measurement.state,stamp,now,max_reprop_,fallback);
      auto committed=predictor_->committedHorizon(predicted.state,now,fallback);
      state_prediction_horizon=predicted.horizon;
      committed_horizon=committed.horizon;
      conditioned_committed=low_speed_conditioner_->condition(committed.state);
      compensated=conditioned_committed.state;
      if(checkpoint_recovery_rearm_.exchange(false)){
        checkpoint_recovery_->arm();
        race_start_evaluated_=false;
        race_start_active_=false;
        candidate_arbitrator_->reset();
        if(local_mpcc_active){
          solver_->clearLocalReference();
          {std::lock_guard<std::mutex>l(mutex_);
            local_reference_active_=false;local_reference_signature_=0;}
          local_mpcc_active=false;
          publishLocalReferenceStatus(local_state,false,"checkpoint_recovery_armed");
        }
        solver_->resetWarmStart();
      }
      // A restart after the car has been carried to a checkpoint must acquire
      // the globally nearest track section. Reusing the pre-crash progress
      // guess can latch onto the wrong nearby branch of a closed circuit.
      projection=checkpoint_recovery_->armed()
        ?headingAwareGlobalProject(*track_,compensated[0],compensated[1],compensated[2])
        :track_->project(compensated[0],compensated[1],compensated[8]);
      compensated[7]=command_manager_->published()[1];compensated[8]=projection.s;
      if(checkpoint_recovery_->armed()){
        std::lock_guard<std::mutex>l(mutex_);theta_guess_=projection.s;
      }
      const double pp_footprint=vehicle_.body_width/2+
        controller_["pure_pursuit_candidate"]["boundary_margin_m"].as<double>();
      const auto current_geometry=track_->geometry(projection.s);
      const double current_track_margin=std::min(
        current_geometry.width_left-pp_footprint-projection.e_contour,
        current_geometry.width_right-pp_footprint+projection.e_contour);
      const bool out_of_bounds=oob_recovery_enabled_&&current_track_margin<0.0;
      auto observer=observer_->estimate();auto profile=startup_->profile(now);
      // Enforce the local raceline profile as a command ceiling. Without this,
      // progress reward can exceed 3 m/s outside explicitly named fast zones.
      const double local_track_speed_cap=std::max(0.0,current_geometry.speed_prior);
      double requested_cap=std::min(profile.speed_cap,local_track_speed_cap);
      requested_cap=std::min(requested_cap,local_speed_cap);
      if(checkpoint_recovery_->armed())
        requested_cap=std::min(requested_cap,checkpoint_recovery_->speedCap());
      if(age>warning_age_)requested_cap=std::min(requested_cap,latency_speed_limit_);
      double observer_age=std::isfinite(observer.stamp)?now-observer.stamp:1e9;
      if(!observer.valid||observer_age>observer_timeout_)requested_cap=std::min(requested_cap,invalid_limit_);
      else if(observer.mode==mpcc::ObserverMode::MODEL_ONLY)requested_cap=std::min(requested_cap,model_only_limit_);
      double target_cap;
      {std::lock_guard<std::mutex>l(mutex_);target_cap=predictive_speed_limiter_->limit(requested_cap,now);runtime_speed_cap_=target_cap;}
      double warm_advance=1.0/control_rate_;
      if(last_solver_tick_)warm_advance=std::max(0.0,now-*last_solver_tick_);
      last_solver_tick_=now;
      result=solver_->solve(compensated,target_cap,profile.steering_rate_cap,
                            profile.progress_scale,warm_advance);
      {std::lock_guard<std::mutex>l(mutex_);
        if(result.speed_retry)predictive_speed_limiter_->noteRisk(result.speed_cap_used,now);
        runtime_speed_cap_=predictive_speed_limiter_->limit(requested_cap,now);
      }
      const auto candidate_cfg=controller_["pure_pursuit_candidate"];
      if(!result.states.empty()){
        const double safety_dt=controller_["timing"]["horizon_dt_s"].as<double>();
        const double near_horizon=controller_["safety"]["near_track_horizon_s"].as<double>();
        mpcc_safety=mpcc::evaluateTrajectorySafety(
          result.states,*track_,vehicle_.body_width,
          candidate_cfg["boundary_margin_m"].as<double>(),
          safety_dt);
        mpcc_near_safety=mpcc::evaluateTrajectorySafety(
          result.states,*track_,vehicle_.body_width,
          candidate_cfg["boundary_margin_m"].as<double>(),
          safety_dt,near_horizon);
      }
      const bool track_rejection=result.failure_kind==1||result.failure_kind==2;
      const bool slack_risk=std::max(result.near_track_slack,result.far_track_slack)>
        pure_pursuit_candidate_->triggerSlack();
      const bool rollout_risk=!result.states.empty()&&!mpcc_safety.safe;
      prediction_risk=track_rejection||slack_risk||rollout_risk;
      const bool current_margin_risk=!result.states.empty()&&
        std::isfinite(mpcc_safety.initial_margin)&&
        mpcc_safety.initial_margin<=pp_current_margin_;
      near_prediction_risk=current_margin_risk||
        (!result.states.empty()&&!mpcc_near_safety.safe);
      far_prediction_risk=prediction_risk&&!near_prediction_risk;
      if(risk_tiering_enabled_){
        if(current_margin_risk){
          near_risk_cycles_=pp_confirmation_cycles_;
          pp_trigger_risk=true;
        }else if(near_prediction_risk){
          near_risk_cycles_=std::min(near_risk_cycles_+1,pp_confirmation_cycles_);
          pp_trigger_risk=near_risk_cycles_>=pp_confirmation_cycles_;
        }else{
          near_risk_cycles_=0;
        }
        speed_only_risk=prediction_risk&&!pp_trigger_risk&&
          !candidate_arbitrator_->active();
        if(speed_only_risk){
          const auto safety_cfg=controller_["safety"];
          const double reduced_cap=std::min(target_cap,std::max(
            safety_cfg["far_track_retry_min_speed_mps"].as<double>(),
            target_cap*safety_cfg["far_track_retry_speed_scale"].as<double>()));
          {std::lock_guard<std::mutex>l(mutex_);
            predictive_speed_limiter_->noteRisk(reduced_cap,now);
            runtime_speed_cap_=predictive_speed_limiter_->limit(requested_cap,now);
          }
          ROS_WARN_THROTTLE(.5,
            "Tiered prediction risk: speed-only tier=%s current=%.3f near=%.3f full=%.3f cycles=%d/%d cap=%.2f",
            far_prediction_risk?"far":"near_confirming",
            diagnosticValue(mpcc_safety.initial_margin),
            diagnosticValue(mpcc_near_safety.minimum_margin),
            diagnosticValue(mpcc_safety.minimum_margin),
            near_risk_cycles_,pp_confirmation_cycles_,reduced_cap);
        }
      }else{
        near_risk_cycles_=prediction_risk?pp_confirmation_cycles_:0;
        pp_trigger_risk=prediction_risk;
      }
      // Out-of-bounds recovery channel: MPCC can fail with status=4 and no
      // rollout (so no normal prediction risk) exactly when the car is
      // already outside the corridor. Force an explicit PP re-entry
      // evaluation; the arbitrator still has to prove the rollout returns.
      if(out_of_bounds&&!local_mpcc_active)pp_trigger_risk=true;
      const bool checkpoint_armed=checkpoint_recovery_->armed();
      if(checkpoint_armed&&!race_start_evaluated_){
        const double heading_error=mpcc::wrapAngle(
          compensated[2]-projection.psi_ref);
        race_start_active_=race_start_enabled_&&
          std::hypot(compensated[0]-race_start_center_x_,
                     compensated[1]-race_start_center_y_)<=race_start_radius_&&
          circularProgressDistance(projection.s_wrapped,race_start_s_,
                                   track_->length())<=race_start_s_tolerance_&&
          std::abs(projection.e_contour)<=race_start_maximum_contour_error_&&
          std::abs(heading_error)<=race_start_maximum_heading_error_&&
          current_track_margin>=race_start_minimum_vehicle_margin_;
        race_start_evaluated_=true;
        ROS_WARN("C++ race-start %s s=%.3f lateral=%.3f heading_deg=%.2f margin=%.3f cap=%.2f",
          race_start_active_?"ACTIVE":"not_applicable",projection.s_wrapped,
          projection.e_contour,heading_error*180.0/M_PI,current_track_margin,
          race_start_speed_);
      }
      const double candidate_cap=checkpoint_armed
        ?std::min(target_cap,race_start_active_
            ?race_start_speed_:checkpoint_recovery_->speedCap()):target_cap;
      if(!local_mpcc_active&&
         (pp_trigger_risk||candidate_arbitrator_->active()||checkpoint_armed))
        candidate=(race_start_active_?race_start_candidate_.get():
          pure_pursuit_candidate_.get())->generate(
            compensated,candidate_cap,out_of_bounds,published_speed);
      if(!local_mpcc_active&&checkpoint_armed){
        const double heading_error=mpcc::wrapAngle(
          compensated[2]-projection.psi_ref);
        checkpoint_selection=checkpoint_recovery_->update(
          projection.e_contour,heading_error,current_track_margin,
          candidate.safety,mpcc_safety);
        if(checkpoint_selection.use_candidate&&candidate.valid){
          checkpoint_selected=true;
          command_manager_->setCandidate(candidate.command_speed,
            candidate.command_steering,candidate.virtual_speed,now,
            "checkpoint_pure_pursuit");
          startup_->noteSolver(true,now);
          {std::lock_guard<std::mutex>l(mutex_);
            oob_recovery_active_=false;
            runtime_speed_cap_=std::min(runtime_speed_cap_,
                                        checkpoint_recovery_->speedCap());
          }
          ROS_WARN_THROTTLE(.5,
            "Checkpoint Pure Pursuit ACTIVE lateral=%.3f heading=%.3f margin=%.3f speed=%.2f steer=%.3f",
            projection.e_contour,heading_error,current_track_margin,
            candidate.command_speed,candidate.command_steering);
        }else if(checkpoint_selection.blocked){
          checkpoint_blocked=true;
          command_manager_->setCandidate(0.0,
            command_manager_->published()[1],0.0,now,
            "checkpoint_recovery_wait");
          startup_->noteSolver(true,now);
          {std::lock_guard<std::mutex>l(mutex_);
            oob_recovery_active_=false;runtime_speed_cap_=0.0;
          }
          ROS_ERROR_THROTTLE(1,
            "Checkpoint recovery WAITING: %s lateral=%.3f heading=%.3f margin=%.3f candidate_min=%.3f",
            checkpoint_selection.reason.c_str(),projection.e_contour,
            heading_error,current_track_margin,
            diagnosticValue(candidate.safety.minimum_margin));
        }else if(checkpoint_selection.released){
          if(race_start_active_)
            ROS_INFO("C++ race-start RELEASE to MPCC");
          race_start_active_=false;
          candidate_arbitrator_->reset();
          predictive_speed_limiter_->reset();
          ROS_INFO("Checkpoint Pure Pursuit RELEASE to MPCC: %s",
                   checkpoint_selection.reason.c_str());
        }
      }
      if(!local_mpcc_active&&!checkpoint_selected&&!checkpoint_blocked){
        candidate_selection=candidate_arbitrator_->choose(
          pp_trigger_risk,mpcc_safety,candidate.safety,now);
      }
      if(local_mpcc_active){
        candidate_arbitrator_->reset();
        near_risk_cycles_=0;
        if(result.success&&!result.states.empty()&&mpcc_safety.safe){
          {std::lock_guard<std::mutex>l(mutex_);oob_recovery_active_=false;}
          command_manager_->setSolution(result.states,result.controls,now);
          startup_->noteSolver(true,now);
          ROS_INFO_THROTTLE(1,"V5 jubu LOCAL MPCC active state=%s cap=%.2f",
                            local_state.c_str(),target_cap);
        }else{
          {std::lock_guard<std::mutex>l(mutex_);oob_recovery_active_=false;}
          command_manager_->noteFailure();startup_->noteSolver(false,now);
          ROS_ERROR_THROTTLE(.5,
            "V5 jubu local MPCC rejected; no Pure Pursuit fallback state=%s status=%d safe=%s margin=%.3f",
            local_state.c_str(),result.status,mpcc_safety.safe?"true":"false",
            diagnosticValue(mpcc_safety.minimum_margin));
        }
      }else if(!checkpoint_selected&&!checkpoint_blocked&&
         candidate_selection.use_candidate&&candidate.valid){
        command_manager_->setCandidate(candidate.command_speed,
          candidate.command_steering,candidate.virtual_speed,now,
          candidate_selection.recovery?"pure_pursuit_recovery":
                                       "pure_pursuit_candidate");
        startup_->noteSolver(true,now);
        {std::lock_guard<std::mutex>l(mutex_);
          oob_recovery_active_=out_of_bounds&&candidate_selection.recovery;
          // Latch the takeover speed once. Feeding every candidate command
          // back as the next requested cap recursively drove 1.68 m/s to
          // nearly zero even after the trajectory was safe again.
          if(candidate_selection.activated)
            predictive_speed_limiter_->noteRisk(std::max(
              candidate.command_speed,
              std::min(requested_cap,
                candidate_cfg["recovery_speed_min_mps"].as<double>())),now);
          runtime_speed_cap_=predictive_speed_limiter_->limit(requested_cap,now);
        }
        if(oob_recovery_active_)ROS_WARN_THROTTLE(.5,
          "Pure Pursuit out-of-bounds recovery ACTIVE margin=%.3f speed=%.2f steer=%.3f",
          current_track_margin,candidate.command_speed,candidate.command_steering);
        ROS_WARN_THROTTLE(.5,
          "Pure Pursuit safety candidate active: reason=%s MPCC_min=%.3f candidate_initial=%.3f candidate_final=%.3f reentry=%.2f speed=%.2f steer=%.3f",
          candidate_selection.reason.c_str(),mpcc_safety.minimum_margin,
          candidate.safety.initial_margin,candidate.safety.final_margin,
          candidate.safety.first_safe_time,
          candidate.command_speed,candidate.command_steering);
      }else if(!checkpoint_selected&&!checkpoint_blocked&&result.success){
        {std::lock_guard<std::mutex>l(mutex_);oob_recovery_active_=false;}
        command_manager_->setSolution(result.states,result.controls,now);startup_->noteSolver(true,now);
        if(result.speed_retry)ROS_WARN_THROTTLE(1,"C++ MPCC predictive slowdown: trigger=%.3f stage=%d t=%.2f side=%d target_cap=%.2f input_cap=%.2f",result.trigger_track_slack,result.trigger_track_slack_stage,result.trigger_track_slack_time,result.trigger_track_slack_side,result.speed_cap_used,result.speed_input_cap);
      }else if(!checkpoint_selected&&!checkpoint_blocked){
        {std::lock_guard<std::mutex>l(mutex_);oob_recovery_active_=false;}
        command_manager_->noteFailure();startup_->noteSolver(false,now);
        ROS_WARN_THROTTLE(1,"C++ MPCC solve rejected: reason=%s status=%d kind=%d near=%.3f far=%.3f stage=%d t=%.2f side=%d target_cap=%.2f input_cap=%.2f reset=%s",result.reason.c_str(),result.status,result.failure_kind,result.near_track_slack,result.far_track_slack,result.track_slack_stage,result.track_slack_time,result.track_slack_side,result.speed_cap_used,result.speed_input_cap,result.warm_start_reset?"true":"false");
      }
      if(candidate_selection.activated)ROS_WARN(
        "Pure Pursuit safety candidate TAKEOVER: MPCC margin %.3f -> candidate %.3f",
        mpcc_safety.minimum_margin,candidate.safety.minimum_margin);
      if(candidate_selection.released)ROS_INFO(
        "Pure Pursuit safety candidate RELEASE: %s, MPCC margin %.3f",
        candidate_selection.reason.c_str(),mpcc_safety.minimum_margin);
    }catch(const std::exception& e){command_manager_->noteFailure();startup_->noteSolver(false,now);ROS_ERROR_THROTTLE(1,"C++ MPCC exception: %s",e.what());return;}
    int candidate_reason=local_mpcc_active?12:(pp_trigger_risk?1:(far_prediction_risk?7:(speed_only_risk?8:0)));if(checkpoint_selection.use_candidate)candidate_reason=9;else if(checkpoint_selection.blocked)candidate_reason=10;else if(checkpoint_selection.released)candidate_reason=11;else if(candidate_selection.activated)candidate_reason=candidate_selection.recovery?6:2;else if(candidate_selection.released)candidate_reason=3;else if(candidate_selection.reason=="candidate_unsafe"||candidate_selection.reason=="candidate_became_unsafe")candidate_reason=4;else if(candidate_selection.reason=="insufficient_improvement")candidate_reason=5;
    const bool candidate_active=!local_mpcc_active&&(checkpoint_selected||candidate_arbitrator_->active());
    const bool candidate_activated=checkpoint_selection.activated||candidate_selection.activated;
    const bool candidate_released=checkpoint_selection.released||candidate_selection.released;
    std_msgs::Float32MultiArray diag;auto u=command_manager_->published();auto slips=model_->slipAngles(compensated);double heading=mpcc::wrapAngle(compensated[2]-projection.psi_ref),beta=std::atan2(compensated[4],std::hypot(compensated[3],vehicle_.vx_regularization));diag.data={static_cast<float>(projection.e_contour),static_cast<float>(projection.e_lag),static_cast<float>(heading),static_cast<float>(beta),static_cast<float>(slips[0]),static_cast<float>(slips[1]),static_cast<float>(u[0]),static_cast<float>(u[1]),static_cast<float>(age),static_cast<float>(result.solve_time),static_cast<float>(startup_->failureCount()),static_cast<float>(result.status),result.success?1.0f:0.0f,static_cast<float>(result.track_slack),static_cast<float>(result.tire_slack),static_cast<float>(compensated[3]),static_cast<float>(compensated[4]),static_cast<float>(compensated[5]),static_cast<float>(result.near_track_slack),static_cast<float>(result.far_track_slack),static_cast<float>(result.track_slack_stage),static_cast<float>(result.track_slack_time),static_cast<float>(result.track_slack_side),static_cast<float>(result.speed_cap_used),result.speed_retry?1.0f:0.0f,result.warm_start_reset?1.0f:0.0f,static_cast<float>(result.failure_kind),static_cast<float>(result.rti_iterations),static_cast<float>(result.speed_input_cap),static_cast<float>(result.trigger_track_slack),static_cast<float>(result.trigger_track_slack_stage),static_cast<float>(result.trigger_track_slack_time),static_cast<float>(result.trigger_track_slack_side),static_cast<float>(state_prediction_horizon),static_cast<float>(committed_horizon),static_cast<float>(committed_horizon),static_cast<float>(result.objective),static_cast<float>(result.cost.contour),static_cast<float>(result.cost.lag),static_cast<float>(result.cost.heading),static_cast<float>(result.cost.speed_prior),static_cast<float>(result.cost.progress),static_cast<float>(result.cost.control),static_cast<float>(result.cost.delta_control),static_cast<float>(result.cost.track_slack),static_cast<float>(result.cost.tire_slack),static_cast<float>(result.geometry_theta_shift),static_cast<float>(result.geometry_heading_shift),static_cast<float>(result.geometry_curvature_shift),candidate_active?1.0f:0.0f,candidate_activated?1.0f:0.0f,candidate_released?1.0f:0.0f,prediction_risk?1.0f:0.0f,static_cast<float>(diagnosticValue(mpcc_safety.minimum_margin)),static_cast<float>(diagnosticValue(candidate.safety.minimum_margin)),static_cast<float>(diagnosticValue(candidate_selection.improvement)),static_cast<float>(candidate.command_speed),static_cast<float>(candidate.command_steering),static_cast<float>(candidate.lookahead),static_cast<float>(candidate_reason),static_cast<float>(diagnosticValue(candidate.safety.initial_margin)),static_cast<float>(diagnosticValue(candidate.safety.final_margin)),static_cast<float>(candidate.safety.first_safe_time),(candidate_selection.recovery||checkpoint_selected)?1.0f:0.0f,static_cast<float>(raw_pointlio[3]),static_cast<float>(raw_pointlio[4]),static_cast<float>(raw_pointlio[5]),conditioned_committed.stationary?1.0f:0.0f,static_cast<float>(conditioned_committed.dynamic_blend),static_cast<float>(result.warm_start_advance_s),static_cast<float>(result.warm_start_shift_stages),static_cast<float>(skipped_solve_ticks_.load()),pp_trigger_risk?1.0f:0.0f,near_prediction_risk?1.0f:0.0f,far_prediction_risk?1.0f:0.0f,static_cast<float>(near_risk_cycles_),static_cast<float>(diagnosticValue(mpcc_near_safety.minimum_margin)),checkpoint_recovery_->armed()?1.0f:0.0f};diagnostic_pub_.publish(diag);publishPrediction(result,candidate,candidate_active,checkpoint_blocked);
  }
  void publisherTick(const ros::TimerEvent&){double now=ros::Time::now().toSec();auto p=startup_->profile(now);{std::lock_guard<std::mutex>l(mutex_);p.speed_cap=std::min(p.speed_cap,runtime_speed_cap_);if(oob_recovery_active_){p.mode=mpcc::StartupMode::RACE;p.speed_cap=std::max(p.speed_cap,oob_recovery_speed_);p.steering_rate_cap=std::max(p.steering_rate_cap,oob_recovery_rate_);p.steering_limit=std::numeric_limits<double>::infinity();}if(!enabled_)p={mpcc::StartupMode::STOPPED,0,0,controller_["fallback"]["steering_recenter_rate_radps"].as<double>(),0};}auto out=command_manager_->tick(now,p);if(out.source=="safe_decel_hold_steering")ROS_WARN_THROTTLE(0.5,"C++ MPCC fallback holding steering while braking: speed=%.3f steering=%.3f failures=%d",out.speed,out.steering,startup_->failureCount());double previous=last_published_?last_published_->steering:out.steering,rate=(out.steering-previous)/out.dt;mpcc::Control model;model<<out.speed,rate,out.virtual_speed;{std::lock_guard<std::mutex>l(mutex_);last_control_=model;last_published_=out;}command_history_.push({now,out.speed,out.steering,out.virtual_speed,rate});observer_->pushCommand(now,out.steering);publish(out.speed,out.steering);}
  void publish(double speed,double steering){ackermann_msgs::AckermannDriveStamped msg;msg.header.stamp=ros::Time::now();msg.header.frame_id=body_frame_;msg.drive.speed=speed;msg.drive.steering_angle=steering;command_pub_.publish(msg);}
  void publishStop(){double steering=command_manager_?command_manager_->published()[1]:0;publish(0,steering);}
  visualization_msgs::Marker line(int id,const std::string& name,float r,float g,float b)const{visualization_msgs::Marker m;m.header.frame_id=world_frame_;m.header.stamp=ros::Time::now();m.ns="f1tenth_dynamic_mpcc_cpp";m.id=id;m.type=visualization_msgs::Marker::LINE_STRIP;m.action=visualization_msgs::Marker::ADD;m.pose.orientation.w=1;m.scale.x=.025;m.color.r=r;m.color.g=g;m.color.b=b;m.color.a=1;m.text=name;return m;}
  void publishTrackMarkers(){visualization_msgs::MarkerArray out;out.markers={line(0,"raceline",.2,.8,1),line(1,"track_left",1,.2,.2),line(2,"track_right",1,.2,.2),line(3,"control_left",1,.8,.1),line(4,"control_right",1,.8,.1)};double margin=vehicle_.body_width/2+controller_["safety"]["track_control_margin_m"].as<double>()+.05;for(int i=0;i<=500;++i){auto g=track_->geometry(track_->length()*i/500.0);double nx=-std::sin(g.yaw),ny=std::cos(g.yaw);auto add=[](auto& m,double x,double y,double z){geometry_msgs::Point p;p.x=x;p.y=y;p.z=z;m.points.push_back(p);};add(out.markers[0],g.x,g.y,.02);add(out.markers[1],g.x+nx*g.width_left,g.y+ny*g.width_left,.01);add(out.markers[2],g.x-nx*g.width_right,g.y-ny*g.width_right,.01);add(out.markers[3],g.x+nx*std::max(0.0,g.width_left-margin),g.y+ny*std::max(0.0,g.width_left-margin),.03);add(out.markers[4],g.x-nx*std::max(0.0,g.width_right-margin),g.y-ny*std::max(0.0,g.width_right-margin),.03);}marker_pub_.publish(out);}
  void publishPrediction(const mpcc::SolverResult& result,const mpcc::PurePursuitCandidateResult& candidate,bool candidate_selected,bool checkpoint_blocked){static ros::Time last;if((ros::Time::now()-last).toSec()<.2)return;last=ros::Time::now();visualization_msgs::MarkerArray out;auto m=line(10,"mpcc_prediction",.3,1,.3);for(const auto& x:result.states){geometry_msgs::Point p;p.x=x[0];p.y=x[1];p.z=.06;m.points.push_back(p);}out.markers.push_back(m);auto c=candidate_selected?line(11,"pure_pursuit_selected",1,.2,1):(checkpoint_blocked?line(11,"pure_pursuit_blocked",1,.15,.05):line(11,"pure_pursuit_not_selected",.45,.45,.45));for(const auto& x:candidate.states){geometry_msgs::Point p;p.x=x[0];p.y=x[1];p.z=.08;c.points.push_back(p);}if(candidate.states.empty())c.action=visualization_msgs::Marker::DELETE;out.markers.push_back(c);prediction_pub_.publish(out);}

  ros::NodeHandle node_,private_;
  YAML::Node controller_,vehicle_cfg_;
  mpcc::VehicleParameters vehicle_;
  std::shared_ptr<const f1tenth_residual_dynamics::ResidualModel> residual_model_;
  std::unique_ptr<mpcc::PeriodicTrack>track_;
  std::unique_ptr<mpcc::DynamicBicycleModel>model_;
  std::unique_ptr<mpcc::LowSpeedStateConditioner>low_speed_conditioner_;
  std::unique_ptr<mpcc::PurePursuitCandidate>pure_pursuit_candidate_;
  std::unique_ptr<mpcc::PurePursuitCandidate>race_start_candidate_;
  std::unique_ptr<mpcc::CandidateArbitrator>candidate_arbitrator_;
  std::unique_ptr<mpcc::CheckpointRecoveryManager>checkpoint_recovery_;
  mpcc::CommandHistory command_history_;
  std::unique_ptr<mpcc::LowLatencyPredictor>predictor_;
  std::unique_ptr<mpcc::SteeringObserver>observer_;
  std::unique_ptr<mpcc::StartupStateMachine>startup_;
  std::unique_ptr<mpcc::CommandManager>command_manager_;
  std::unique_ptr<mpcc::PredictiveSpeedLimiter>predictive_speed_limiter_;
  std::unique_ptr<mpcc::AcadosRuntimeSolverJubu>solver_;
  mutable std::mutex mutex_;
  std::mutex solve_mutex_;
  std::optional<mpcc::State>odom_;
  std::optional<double>odom_stamp_,odom_receive_,theta_guess_,last_solver_tick_;
  std::optional<mpcc::PublishedCommand>last_published_;
  mpcc::Control last_control_{mpcc::Control::Zero()};
  std::atomic<unsigned long> skipped_solve_ticks_{0};
  std::atomic<bool> checkpoint_recovery_rearm_{true};
  bool pointlio_valid_{false},collision_{false},collision_required_{false},
    enabled_{false},risk_tiering_enabled_{false},oob_recovery_enabled_{false},
    oob_recovery_active_{false},local_reference_active_{false},
    planner_required_{true},planner_healthy_{false},race_start_enabled_{true},
    race_start_evaluated_{false},race_start_active_{false};
  int pp_confirmation_cycles_{2},near_risk_cycles_{0};
  double pp_current_margin_{0.02};
  double oob_recovery_speed_{1.2},oob_recovery_rate_{1.5};
  double collision_stamp_{-1e99},control_rate_{},odom_timeout_{},max_reprop_{},
    warning_age_{},hard_age_{},benign_reorder_tolerance_{},
    benign_reorder_position_tolerance_{},benign_reorder_yaw_tolerance_{},
    observer_timeout_{},latency_speed_limit_{},model_only_limit_{},invalid_limit_{},
    collision_timeout_{},runtime_speed_cap_{},local_reference_timeout_{0.75},
    local_reference_max_start_distance_{0.75},local_reference_min_length_{1.5},
    local_reference_max_curvature_ratio_{0.90},local_reference_return_tolerance_{0.15},
    local_reference_speed_cap_{2.5},local_reference_stamp_{-1e99},
    planner_health_timeout_{0.50},planner_health_stamp_{-1e99},
    race_start_center_x_{0.0},race_start_center_y_{0.0},race_start_radius_{2.0},
    race_start_s_{0.0},race_start_s_tolerance_{1.5},
    race_start_maximum_contour_error_{0.60},
    race_start_maximum_heading_error_{0.35},
    race_start_minimum_vehicle_margin_{-0.05},
    race_start_merge_lookahead_{2.50},race_start_speed_{4.0},
    race_start_lateral_acceleration_limit_{3.20};
  std::uint64_t local_reference_signature_{0};
  std::string formulation_,odom_topic_,command_topic_,collision_topic_,world_frame_,
    body_frame_,local_reference_topic_,local_speed_cap_topic_,local_state_topic_,
    local_reference_status_topic_,planner_health_topic_,local_state_{"GLOBAL"};
  ros::Publisher command_pub_,diagnostic_pub_,marker_pub_,prediction_pub_,
    local_reference_status_pub_;
  ros::Subscriber odom_sub_,imu_sub_,wheel_sub_,collision_sub_,local_reference_sub_,
    local_speed_cap_sub_,local_state_sub_,planner_health_sub_;
  ros::ServiceServer enable_service_;
  ros::Timer control_timer_,publisher_timer_;
};

int main(int argc,char**argv){ros::init(argc,argv,"f1tenth_dynamic_mpcc_v5_jubu");try{MpccNode node;ros::AsyncSpinner spinner(3);spinner.start();ros::waitForShutdown();}catch(const std::exception&e){ROS_FATAL("C++ MPCC V5 jubu initialization failed: %s",e.what());return 1;}return 0;}
