// Adapted gait geometry from yanyuze1/ROS2-Gazebo-GO2 (TrotGaitController.py).
// Original gait authors: mike4192, lnotspotl, abutalipovvv.
// Go2 URDF IK and feedback effort adapter are local; this is not a CHAMP trot.
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include "go2_gait_controller/planner.hpp"
#include "go2_gait_controller/dynamics.hpp"
using go2_gait::Vec;using go2_gait::Mat;
class ReferenceTrotController : public rclcpp::Node {
  go2_gait::Planner planner_;go2_gait::Dynamics dynamics_;
  bool loaded_=false,initialized_=false,switched_=false,fault_=false;
  int bad_joint_=-1;double bad_value_=0,calm_since_=-1;bool tilt_fault_=false;
  std::array<double,12> q_{},qd_{};
  std::array<Vec,4> forces_{};
  std::array<double,4> contact_force_{},contact_time_{-1e9,-1e9,-1e9,-1e9};
  Mat R_=Mat::Identity();Vec omega_=Vec::Zero(),velocity_=Vec::Zero(),command_=Vec::Zero(),requested_=Vec::Zero();
  double last_joint_=-1e9,last_imu_=-1e9,last_cmd_=-1e9,last_tick_=-1,active_since_=0,ready_since_=-1,default_height_=0,desired_yaw_=0;
  double max_x_,max_y_,max_yaw_,swing_height_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr feet_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr contacts_pub_,actual_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double positive(const std::string &s,double v){v=declare_parameter(s,v);if(!std::isfinite(v)||v<=0)throw std::runtime_error(s+" must be positive and finite");return v;}
  static double smooth(double u){u=std::clamp(u,0.,1.);return u*u*u*(10+u*(-15+6*u));}
  static double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
  double yaw()const{return std::atan2(R_(1,0),R_(0,0));}
  double roll()const{return std::atan2(R_(2,1),R_(2,2));}
  double pitch()const{return std::asin(std::clamp(-R_(2,0),-1.,1.));}
  bool contact(int i)const{return now().seconds()-contact_time_[i]<.12 && contact_force_[i]>1.5;}
  std::array<Vec,4> feet_{},nominal_{},swing_origin_{};
  std::array<double,12> previous_desired_{};
  double gait_clock_=0;bool running_=false;int last_phase_=-1;
  void tick(){
    double time=now().seconds();if(last_tick_<0){last_tick_=time;return;}double dt=time-last_tick_;last_tick_=time;if(dt<=0||dt>.1)return;
    if(!loaded_||time-last_joint_>.12||time-last_imu_>.12)return;
    for(int l=0;l<4;++l)if(time-contact_time_[l]>.12)return;
    auto measured=dynamics_.evaluate(q_,R_);
    if(!initialized_){double speed=0,error=0;for(int i=0;i<12;++i){speed=std::max(speed,std::abs(qd_[i]));error=std::max(error,std::abs(q_[i]-std::array<double,3>{0,.65,-1.3}[i%3]));}
      if(speed>.15||error>.12){ready_since_=-1;return;}if(ready_since_<0)ready_since_=time;if(time-ready_since_<.5)return;
      feet_=nominal_=measured.feet;default_height_=0;for(auto f:measured.feet)default_height_-= (R_*f).z()/4;previous_desired_=q_;initialized_=true;active_since_=time;desired_yaw_=yaw();}
    if(!switched_){if(!switch_->service_is_ready())return;
      if(!future_.valid()){auto request=std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();request->activate_controllers={"go2_balance_effort_controller"};request->deactivate_controllers={"go2_joint_tracking_controller"};request->strictness=request->STRICT;request->timeout.sec=5;future_=switch_->async_send_request(request).share();}
      if(future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){if(!future_.get()->ok)throw std::runtime_error("effort controller activation failed");switched_=true;active_since_=time;}}
    Vec req=time-last_cmd_<.5?requested_:Vec::Zero();if(!switched_||time-active_since_<1||fault_)req.setZero();
    for(int i=0;i<3;++i)command_[i]+=std::clamp(req[i]-command_[i],-(i==2?.5:.6)*dt,(i==2?.5:.6)*dt);
    // Reference gait: all support, FL/RR swing, all support, FR/RL swing.
    const double settle=.04,swing=.18,period=2*(settle+swing);
    desired_yaw_+=command_.z()*dt;double turn=std::clamp(command_.z()+2*wrap(desired_yaw_-yaw()),-1.2,1.2);bool moving=command_.norm()>1e-4;if(moving)running_=true;
    if(running_){gait_clock_+=dt;if(!moving&&std::fmod(gait_clock_,period)<settle)running_=false;}
    double t=std::fmod(gait_clock_,period);int p=t<settle?0:t<settle+swing?1:t<2*settle+swing?2:3;
    double u=p==1?(t-settle)/swing:p==3?(t-2*settle-swing)/swing:0;
    if(p!=last_phase_){swing_origin_=feet_;last_phase_=p;}
    std::array<bool,4> support;std::array<Vec,4> target;
    for(int l=0;l<4;++l){bool sw=running_&&((p==1&&(l==0||l==3))||(p==3&&(l==1||l==2)));support[l]=!sw;
      if(sw){Vec touchdown=Eigen::AngleAxisd(turn*(settle+swing)/2,Vec::UnitZ())*nominal_[l]+Vec(command_.x(),command_.y(),0)*(settle+swing)/2;
        feet_[l]=swing_origin_[l]+u*(touchdown-swing_origin_[l]);feet_[l].z()=nominal_[l].z()+swing_height_*16*u*u*(1-u)*(1-u);}
      else if(running_){feet_[l]=Eigen::AngleAxisd(-turn*dt,Vec::UnitZ())*feet_[l]-Vec(command_.x(),command_.y(),0)*dt;feet_[l].z()+=std::clamp((nominal_[l].z()-feet_[l].z())*dt/.02,-.005,.005);}
      else feet_[l]+=(nominal_[l]-feet_[l])*std::min(1.,dt/.2);
      target[l]=R_.transpose()*Eigen::AngleAxisd(yaw(),Vec::UnitZ())*feet_[l];}
    std::array<double,12> desired{};bool valid=true;
    for(int l=0;l<4;++l){geometry::Transformation f;f.X()=target[l].x();f.Y()=target[l].y();f.Z()=target[l].z();champ::Kinematics::transformToHip(f,*planner_.base.legs[l]);float h=NAN,a=NAN,b=NAN;champ::Kinematics::inverse(h,a,b,*planner_.base.legs[l],f);std::array<double,3> v{h,a,b};for(int j=0;j<3;++j){int i=3*l+j;desired[i]=v[j];if(!std::isfinite(v[j])||v[j]<planner_.model.lower[i]||v[j]>planner_.model.upper[i]){if(valid){bad_joint_=i;bad_value_=v[j];}valid=false;}}}
    const bool tilt_bad=std::abs(roll())>.6||std::abs(pitch())>.6;
    if(!valid||tilt_bad){if(!fault_)RCLCPP_ERROR(get_logger(),"Trot stopped: unreachable target or excessive tilt (valid=%d joint=%d value=%.3f roll=%.3f pitch=%.3f cmd=%.2f,%.2f,%.2f)",int(valid),bad_joint_,bad_value_,roll(),pitch(),command_.x(),command_.y(),command_.z());fault_=true;if(tilt_bad)tilt_fault_=true;calm_since_=-1;desired=previous_desired_;}
    // Only an IK-reach fault is recoverable: after 1 s of zero command with sane tilt, reset feet to the nominal stance.
    // An excessive-tilt fault stays latched.
    if(fault_&&!tilt_fault_){
      if(valid&&std::abs(roll())<.3&&std::abs(pitch())<.3&&(time-last_cmd_>=.5||requested_.norm()<1e-4)){
        if(calm_since_<0)calm_since_=time;
        if(time-calm_since_>1.){fault_=false;calm_since_=-1;running_=false;gait_clock_=0;last_phase_=-1;feet_=nominal_;command_.setZero();RCLCPP_WARN(get_logger(),"Trot IK fault cleared after 1 s of zero command");}
      }else calm_since_=-1;
    }
    sensor_msgs::msg::JointState out;out.header.stamp=now();out.name.assign(planner_.model.names.begin(),planner_.model.names.end());auto gravity=dynamics_.gravity(q_,R_);
    auto J=dynamics_.jacobians(q_);Vec body_velocity=Vec::Zero();double z=0;int count=0;
    std::array<Vec,4> arms;std::array<double,4> caps;
    for(int l=0;l<4;++l){arms[l]=R_*(measured.feet[l]-measured.com);caps[l]=support[l]&&contact(l)?140:0;
      if(running_&&(p==0||p==2)){double blend=smooth((p==0?t:t-settle-swing)/settle);bool next_swing=p==0?(l==0||l==3):(l==1||l==2);caps[l]*=next_swing?1-blend:blend;}
      if(caps[l]>0){body_velocity-=R_*(J[l]*Vec(qd_[3*l],qd_[3*l+1],qd_[3*l+2])+omega_.cross(measured.feet[l]));z-=(R_*measured.feet[l]).z();++count;}}
    if(count){body_velocity/=count;z/=count;}else z=default_height_;
    velocity_=.85*velocity_+.15*body_velocity;
    Vec goal=Eigen::AngleAxisd(yaw(),Vec::UnitZ())*Vec(command_.x(),command_.y(),0);
    Vec acc=8*(goal-velocity_);acc.z()=80*(default_height_-z)-15*velocity_.z();for(int i=0;i<3;++i)acc[i]=std::clamp(acc[i],-3.,3.);
    Mat desired_rotation=Eigen::AngleAxisd(desired_yaw_,Vec::UnitZ()).toRotationMatrix();
    Vec error=Vec::Zero();for(int i=0;i<3;++i)error+=.5*R_.col(i).cross(desired_rotation.col(i));
    Vec angular=160*error-22*(R_*omega_-Vec(0,0,command_.z()));
    forces_=go2_gait::distribute(arms,dynamics_.mass*(acc+Vec(0,0,9.81)),R_*measured.inertia*R_.transpose()*angular,caps,forces_);
    std::array<double,12> feed=gravity;for(int l=0;l<4;++l){Vec tau=-J[l].transpose()*R_.transpose()*forces_[l];for(int j=0;j<3;++j)feed[3*l+j]+=tau[j];}
    for(int i=0;i<12;++i){out.position.push_back(desired[i]);out.velocity.push_back(std::clamp((desired[i]-previous_desired_[i])/dt,-8.,8.));out.effort.push_back(feed[i]);}previous_desired_=desired;pub_->publish(out);
    geometry_msgs::msg::PoseArray feet;feet.header.stamp=now();feet.header.frame_id="base";for(auto f:target){geometry_msgs::msg::Pose p;p.position.x=f.x();p.position.y=f.y();p.position.z=f.z();p.orientation.w=1;feet.poses.push_back(p);}feet_pub_->publish(feet);
    std_msgs::msg::UInt8MultiArray planned,actual;for(int l=0;l<4;++l){planned.data.push_back(support[l]);actual.data.push_back(contact(l));}contacts_pub_->publish(planned);actual_pub_->publish(actual);
    std_msgs::msg::Float64MultiArray status;status.data={double(p),command_.x(),command_.y(),command_.z(),roll(),pitch(),yaw(),double(fault_)};status_pub_->publish(status);
  }

public:
  ReferenceTrotController():Node("go2_champ_controller"){
    max_x_=positive("max_velocity_x",.55);max_y_=positive("max_velocity_y",.15);max_yaw_=positive("max_yaw_rate",1.);swing_height_=positive("swing_height",.04);
    champ::GaitConfig cfg;cfg.nominal_height=.33913;cfg.com_x_translation=0;planner_.base.setGaitConfig(cfg);
    pub_=create_publisher<sensor_msgs::msg::JointState>("/go2/balance_joint_command",1);feet_pub_=create_publisher<geometry_msgs::msg::PoseArray>("/go2/foot_targets",10);contacts_pub_=create_publisher<std_msgs::msg::UInt8MultiArray>("/go2/gait_contacts",10);actual_pub_=create_publisher<std_msgs::msg::UInt8MultiArray>("/go2/actual_contacts",10);status_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("/go2/balance_status",10);
    switch_=create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
    subscriptions_.push_back(create_subscription<std_msgs::msg::String>("/robot_description",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){if(loaded_)return;planner_.model.load(planner_.base,m->data,{"FL","FR","RL","RR"},"base");dynamics_.load(m->data,planner_.model.names);loaded_=true;}));
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr m){if(!loaded_)return;std::unordered_map<std::string,size_t>map;for(size_t i=0;i<m->name.size();++i)map[m->name[i]]=i;for(int i=0;i<12;++i){auto it=map.find(planner_.model.names[i]);if(it==map.end()||it->second>=m->position.size()||it->second>=m->velocity.size())return;size_t k=it->second;if(!std::isfinite(m->position[k])||!std::isfinite(m->velocity[k]))return;q_[i]=m->position[k];qd_[i]=m->velocity[k];}last_joint_=now().seconds();}));
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::Imu>("/go2/imu",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Imu::ConstSharedPtr m){auto q=m->orientation;Eigen::Quaterniond rot(q.w,q.x,q.y,q.z);if(!rot.coeffs().allFinite()||rot.norm()<.1)return;R_=rot.normalized().toRotationMatrix();omega_=Vec(m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z);last_imu_=now().seconds();}));
    for(int l=0;l<4;++l)subscriptions_.push_back(create_subscription<gazebo_msgs::msg::ContactsState>("/go2/"+std::array<std::string,4>{"FL","FR","RL","RR"}[l]+"/contact",rclcpp::SensorDataQoS(),[this,l](gazebo_msgs::msg::ContactsState::ConstSharedPtr m){double f=0;for(auto &s:m->states){auto v=s.total_wrench.force;f+=std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z);}
      // Gazebo accumulates consecutive 1 ms physics contact samples between sensor updates.
      contact_force_[l]=m->states.empty()?0:f/m->states.size();contact_time_[l]=now().seconds();}));
    subscriptions_.push_back(create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,[this](geometry_msgs::msg::Twist::ConstSharedPtr m){if(!std::isfinite(m->linear.x)||!std::isfinite(m->linear.y)||!std::isfinite(m->angular.z))return;requested_=Vec(std::clamp(m->linear.x,-max_x_,max_x_),std::clamp(m->linear.y,-max_y_,max_y_),std::clamp(m->angular.z,-max_yaw_,max_yaw_));last_cmd_=now().seconds();}));
    timer_=rclcpp::create_timer(get_node_base_interface(),get_node_timers_interface(),get_clock(),std::chrono::milliseconds(5),[this]{tick();});
  }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<ReferenceTrotController>());}catch(const std::exception&e){fprintf(stderr,"Go2 balance failed: %s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
