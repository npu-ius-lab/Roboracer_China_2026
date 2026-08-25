#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include <mutex>
#include <optional>

class CommandGate {
 public:
  CommandGate():private_("~"){
    bool allowed=false;private_.param("allow_real_hardware",allowed,false);if(!allowed)throw std::runtime_error("allow_real_hardware must be true");
    private_.param("input_topic",input_,std::string("/f1tenth_mpcc/real/ackermann_cmd_stamped"));private_.param("output_topic",output_,std::string("/tianracer/ackermann_cmd"));private_.param("collision_topic",collision_topic_,std::string("/tianracer/collision"));private_.param("collision_required",collision_required_,false);private_.param("start_enabled",enabled_,false);private_.param("command_timeout_s",command_timeout_,.20);private_.param("collision_timeout_s",collision_timeout_,.50);double rate;private_.param("publish_rate_hz",rate,20.0);
    publisher_=node_.advertise<ackermann_msgs::AckermannDrive>(output_,1);command_sub_=node_.subscribe(input_,1,&CommandGate::commandCallback,this,ros::TransportHints().tcpNoDelay());if(collision_required_)collision_sub_=node_.subscribe(collision_topic_,1,&CommandGate::collisionCallback,this);service_=private_.advertiseService("set_enabled",&CommandGate::enable,this);timer_=node_.createTimer(ros::Duration(1/rate),&CommandGate::tick,this);stop();ROS_WARN("C++ hardware command gate output=%s enabled=%s",output_.c_str(),enabled_?"true":"false");
  }
  ~CommandGate(){stop();}
 private:
  void commandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg){std::lock_guard<std::mutex>l(mutex_);command_=msg->drive;command_stamp_=ros::WallTime::now().toSec();}
  void collisionCallback(const std_msgs::Bool::ConstPtr& msg){std::lock_guard<std::mutex>l(mutex_);collision_=msg->data;collision_stamp_=ros::WallTime::now().toSec();}
  bool safe(double now)const{return command_&&now-command_stamp_<=command_timeout_&&(!collision_required_||(!collision_&&now-collision_stamp_<=collision_timeout_));}
  bool enable(std_srvs::SetBool::Request&req,std_srvs::SetBool::Response&res){std::lock_guard<std::mutex>l(mutex_);if(req.data&&!safe(ros::WallTime::now().toSec())){res.success=false;res.message="fresh command required";return true;}enabled_=req.data;res.success=true;res.message=enabled_?"enabled":"disabled";return true;}
  void tick(const ros::TimerEvent&){std::optional<ackermann_msgs::AckermannDrive>command;{std::lock_guard<std::mutex>l(mutex_);if(enabled_&&safe(ros::WallTime::now().toSec()))command=command_;}if(command)publisher_.publish(*command);else stop();}
  void stop(){publisher_.publish(ackermann_msgs::AckermannDrive());}
  ros::NodeHandle node_,private_;ros::Publisher publisher_;ros::Subscriber command_sub_,collision_sub_;ros::ServiceServer service_;ros::Timer timer_;mutable std::mutex mutex_;std::optional<ackermann_msgs::AckermannDrive>command_;double command_stamp_{-1e99},collision_stamp_{-1e99},command_timeout_{},collision_timeout_{};bool collision_{true},collision_required_{false},enabled_{false};std::string input_,output_,collision_topic_;
};

int main(int argc,char**argv){ros::init(argc,argv,"f1tenth_mpcc_command_gate");try{CommandGate gate;ros::spin();}catch(const std::exception&e){ROS_FATAL("command gate failed: %s",e.what());return 1;}return 0;}
