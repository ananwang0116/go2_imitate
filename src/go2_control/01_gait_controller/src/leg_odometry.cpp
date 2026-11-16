#include <array>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "go2_gait_controller/planner.hpp"
#include "go2_gait_controller/dynamics.hpp"
using go2_gait::Vec;using go2_gait::Mat;
class LegOdometry:public rclcpp::Node {
  go2_gait::Planner model_;go2_gait::Dynamics dynamics_;
  std::array<double,12> q_{},qd_{};std::array<double,4> forces_{},contact_time_{-1e9,-1e9,-1e9,-1e9};
  std::array<bool,4> planned_{true,true,true,true};double planned_time_=-1e9;
  Mat R_=Mat::Identity();Vec omega_=Vec::Zero(),velocity_=Vec::Zero();
  bool loaded_=false,ready_=false;double imu_time_=-1e9,joint_time_=-1e9,last_time_=-1,yaw_origin_=0,previous_yaw_=0,yaw_unwrapped_=0,x_=0,y_=0,height_=.35;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  static double wrap(double x){return std::atan2(std::sin(x),std::cos(x));}
  void tick(){
    const double t=now().seconds();if(last_time_<0){last_time_=t;return;}double dt=t-last_time_;last_time_=t;
    if(dt<=0||dt>.2||!loaded_||t-joint_time_>.12||t-imu_time_>.12)return;
    double yaw=std::atan2(R_(1,0),R_(0,0)),roll=std::atan2(R_(2,1),R_(2,2)),pitch=std::asin(std::clamp(-R_(2,0),-1.,1.));
    auto state=dynamics_.evaluate(q_);auto J=dynamics_.jacobians(q_);Vec v=Vec::Zero();double z=0;int support=0;
    for(int l=0;l<4;++l)if(t-contact_time_[l]<.12&&forces_[l]>1.5&&(t-planned_time_>.12||planned_[l])){
      v-=R_*(J[l]*Vec(qd_[3*l],qd_[3*l+1],qd_[3*l+2])+omega_.cross(state.feet[l]));
      z+=.022-(R_*state.feet[l]).z();++support;
    }
    if(!ready_){if(support<3)return;yaw_origin_=previous_yaw_=yaw;ready_=true;height_=z/support;}
    yaw_unwrapped_+=wrap(yaw-previous_yaw_);previous_yaw_=yaw;
    if(support){v/=support;velocity_=.7*velocity_+.3*v;height_=.8*height_+.2*z/support;}
    else velocity_*=.8;
    // Local odom axes are aligned to the measured initial yaw. No cmd_vel or Gazebo truth.
    Mat origin=Eigen::AngleAxisd(-yaw_origin_,Vec::UnitZ()).toRotationMatrix();Vec local=origin*velocity_;
    if(support){x_+=local.x()*dt;y_+=local.y()*dt;}
    nav_msgs::msg::Odometry out;out.header.stamp=now();out.header.frame_id="odom";out.child_frame_id="base_footprint";
    out.pose.pose.position.x=x_;out.pose.pose.position.y=y_;
    Eigen::Quaterniond heading(Eigen::AngleAxisd(yaw_unwrapped_,Vec::UnitZ()));out.pose.pose.orientation.z=heading.z();out.pose.pose.orientation.w=heading.w();
    Vec body=Eigen::AngleAxisd(-yaw_unwrapped_,Vec::UnitZ())*local;
    out.twist.twist.linear.x=body.x();out.twist.twist.linear.y=body.y();out.twist.twist.angular.z=(R_*omega_).z();
    out.pose.covariance[0]=out.pose.covariance[7]=.02+.002*(std::abs(x_)+std::abs(y_));out.pose.covariance[14]=out.pose.covariance[21]=out.pose.covariance[28]=1e3;out.pose.covariance[35]=.01;
    out.twist.covariance[0]=out.twist.covariance[7]=support?.01:1.;out.twist.covariance[14]=out.twist.covariance[21]=out.twist.covariance[28]=1e3;out.twist.covariance[35]=.01;publisher_->publish(out);
    geometry_msgs::msg::TransformStamped planar;planar.header=out.header;planar.child_frame_id="base_footprint";planar.transform.translation.x=x_;planar.transform.translation.y=y_;planar.transform.rotation=out.pose.pose.orientation;
    geometry_msgs::msg::TransformStamped tilt;tilt.header.stamp=now();tilt.header.frame_id="base_footprint";tilt.child_frame_id="base";tilt.transform.translation.z=height_;
    Eigen::Quaterniond attitude(Eigen::AngleAxisd(-yaw,Vec::UnitZ()).toRotationMatrix()*R_);tilt.transform.rotation.x=attitude.x();tilt.transform.rotation.y=attitude.y();tilt.transform.rotation.z=attitude.z();tilt.transform.rotation.w=attitude.w();tf_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{planar,tilt});
  }
public:
  LegOdometry():Node("go2_leg_odometry"){
    publisher_=create_publisher<nav_msgs::msg::Odometry>("/odom",10);tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscriptions_.push_back(create_subscription<std_msgs::msg::String>("/robot_description",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){if(loaded_)return;champ::GaitConfig c;c.nominal_height=.33913;model_.base.setGaitConfig(c);model_.model.load(model_.base,m->data,{"FL","FR","RL","RR"},"base");dynamics_.load(m->data,model_.model.names);loaded_=true;}));
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr m){if(!loaded_)return;std::unordered_map<std::string,size_t> names;for(size_t i=0;i<m->name.size();++i)names[m->name[i]]=i;for(int i=0;i<12;++i){auto it=names.find(model_.model.names[i]);if(it==names.end()||it->second>=m->position.size()||it->second>=m->velocity.size())return;auto k=it->second;if(!std::isfinite(m->position[k])||!std::isfinite(m->velocity[k]))return;q_[i]=m->position[k];qd_[i]=m->velocity[k];}joint_time_=now().seconds();}));
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::Imu>("/go2/imu",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Imu::ConstSharedPtr m){auto q=m->orientation;Eigen::Quaterniond r(q.w,q.x,q.y,q.z);if(!r.coeffs().allFinite()||r.norm()<.1)return;R_=r.normalized().toRotationMatrix();omega_=Vec(m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z);imu_time_=now().seconds();}));
    subscriptions_.push_back(create_subscription<std_msgs::msg::UInt8MultiArray>("/go2/gait_contacts",10,[this](std_msgs::msg::UInt8MultiArray::ConstSharedPtr m){if(m->data.size()!=4)return;for(int l=0;l<4;++l)planned_[l]=m->data[l];planned_time_=now().seconds();}));
    for(int l=0;l<4;++l)subscriptions_.push_back(create_subscription<gazebo_msgs::msg::ContactsState>("/go2/"+std::array<std::string,4>{"FL","FR","RL","RR"}[l]+"/contact",rclcpp::SensorDataQoS(),[this,l](gazebo_msgs::msg::ContactsState::ConstSharedPtr m){double f=0;for(auto s:m->states){auto v=s.total_wrench.force;f+=std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z);}forces_[l]=m->states.empty()?0:f/m->states.size();contact_time_[l]=now().seconds();}));
    timer_=rclcpp::create_timer(get_node_base_interface(),get_node_timers_interface(),get_clock(),std::chrono::milliseconds(20),[this]{tick();});
  }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<LegOdometry>());rclcpp::shutdown();}
