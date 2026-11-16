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
class ChampController : public rclcpp::Node {
  enum Phase {STAND,TRANSFER,UNLOAD,SWING,LAND,RECOVER};
  go2_gait::Planner planner_;go2_gait::Dynamics dynamics_;
  Phase phase_=STAND;bool loaded_=false,initialized_=false,active_=false,switched_=false,test_done_=false,cleared_=false,fault_=false;
  std::array<double,12> q_{},qd_{};
  std::array<Vec,4> anchors_{},targets_{},forces_{};
  std::array<double,4> contact_force_{},contact_time_{-1e9,-1e9,-1e9,-1e9};
  Mat R_=Mat::Identity();Vec omega_=Vec::Zero(),position_=Vec::Zero(),previous_position_=Vec::Zero(),velocity_=Vec::Zero(),reference_=Vec::Zero(),command_=Vec::Zero(),requested_=Vec::Zero(),pose_offset_=Vec::Zero(),body_target_=Vec::Zero(),transfer_start_=Vec::Zero(),swing_start_=Vec::Zero(),swing_end_=Vec::Zero(),com_goal_=Vec::Zero(),previous_com_=Vec::Zero(),com_velocity_=Vec::Zero();
  double last_joint_=-1e9,last_imu_=-1e9,last_cmd_=-1e9,last_tick_=-1,phase_start_=0,active_since_=0,ready_since_=-1,landing_z_=0,ground_height_=0,default_height_=0,desired_height_=0,desired_yaw_=0,desired_roll_=0,desired_pitch_=0,pose_yaw_=0;
  double max_x_,max_y_,max_yaw_,height_,transfer_time_,swing_time_,swing_height_,margin_,foot_radius_=.022;
  int leg_=0,beat_=0,single_leg_=-1;const std::array<int,4> order_{0,3,1,2};
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
  std::array<Vec,3> triangle(int excluded)const{std::array<Vec,3> p;int k=0;for(int i=0;i<4;++i)if(i!=excluded)p[k++]=anchors_[i];Vec a=p[1]-p[0],b=p[2]-p[0];if(a.x()*b.y()-a.y()*b.x()<0)std::swap(p[1],p[2]);return p;}
  static double stability(const Vec &c,const std::array<Vec,3>&p){double d=1e9;for(int k=0;k<3;++k){Vec e=p[(k+1)%3]-p[k];double len=e.head<2>().norm();if(len<1e-6)return -1;d=std::min(d,(e.x()*(c.y()-p[k].y())-e.y()*(c.x()-p[k].x()))/len);}return d;}
  static Vec project(Vec c,const std::array<Vec,3>&p,double margin){for(int n=0;n<12;++n)for(int k=0;k<3;++k){Vec e=p[(k+1)%3]-p[k];Vec inward(-e.y(),e.x(),0);inward.normalize();double d=inward.dot(c-p[k]);if(d<margin)c+=(margin-d)*inward;}return c;}
  void phase(Phase p){phase_=p;phase_start_=now().seconds();RCLCPP_INFO(get_logger(),"phase=%d leg=%d",int(p),leg_);}
  void recover(const char *why){fault_=true;requested_.setZero();command_.setZero();reference_.head<2>()=position_.head<2>();body_target_=position_;landing_z_=anchors_[leg_].z();if(initialized_){auto state=dynamics_.evaluate(q_,R_);for(int l=0;l<4;++l)if(contact(l)){anchors_[l]=position_+R_*state.feet[l];targets_[l]=anchors_[l];}}if(phase_!=RECOVER){RCLCPP_WARN(get_logger(),"Contact-aware recovery: %s",why);phase(RECOVER);}}
  void tick(){
    double time=now().seconds();if(last_tick_<0){last_tick_=time;return;}double dt=time-last_tick_;last_tick_=time;if(dt<=0)return;
    if(dt>.1){recover("simulation time gap");return;}
    if(!loaded_||time-last_joint_>.12||time-last_imu_>.12)return;
    for(int i=0;i<4;++i)if(time-contact_time_[i]>.12)return;
    auto measured=dynamics_.evaluate(q_,R_);
    if(!initialized_){double speed=0,error=0;for(int i=0;i<12;++i){speed=std::max(speed,std::abs(qd_[i]));error=std::max(error,std::abs(q_[i]-std::array<double,3>{0,.65,-1.3}[i%3]));}
      if(speed>.15||error>.12||!contact(0)||!contact(1)||!contact(2)||!contact(3)){ready_since_=-1;return;}if(ready_since_<0)ready_since_=time;if(time-ready_since_<.5)return;
      position_=Vec(0,0,0);for(auto f:measured.feet)position_.z()+=foot_radius_-(R_*f).z();position_.z()/=4;
      reference_=position_;body_target_=position_;previous_position_=position_;default_height_=position_.z();desired_height_=default_height_;desired_yaw_=yaw();
      for(int l=0;l<4;++l)anchors_[l]=position_+R_*measured.feet[l];targets_=anchors_;for(auto &f:forces_)f=Vec(0,0,dynamics_.mass*9.81/4);
      previous_com_=position_+R_*measured.com;ground_height_=0;for(int l=0;l<4;++l)ground_height_+=(anchors_[l]+R_*measured.foot_rotation[l]*Vec(-.002,0,0)).z()-foot_radius_;ground_height_/=4;initialized_=true;active_since_=time;
    }
    if(!switched_){if(!switch_->service_is_ready())return;
      if(!future_.valid()){auto request=std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();request->activate_controllers={"go2_balance_effort_controller"};request->deactivate_controllers={"go2_joint_tracking_controller"};request->strictness=request->STRICT;request->timeout.sec=5;future_=switch_->async_send_request(request).share();}
      if(future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){if(!future_.get()->ok)throw std::runtime_error("exclusive effort controller activation failed");switched_=true;active_since_=time;RCLCPP_INFO(get_logger(),"Feedback balance active: IMU + real contacts + support force QP + foot IK");}
    }
    Vec estimate=Vec::Zero();int support=0;
    for(int l=0;l<4;++l)if(contact(l)&&!(l==leg_&&(phase_==SWING||phase_==LAND||phase_==RECOVER))){estimate+=anchors_[l]-R_*measured.feet[l];++support;}
    if(support>=3){position_=estimate/support;Vec raw=(position_-previous_position_)/dt;velocity_=.85*velocity_+.15*raw;previous_position_=position_;}
    if(std::abs(roll())>.20||std::abs(pitch())>.20)recover("body attitude outside walking envelope");
    Vec req=time-last_cmd_<.5?requested_:Vec::Zero();if(!switched_||time-active_since_<1||phase_==RECOVER||fault_)req.setZero();
    for(int i=0;i<3;++i)command_[i]+=std::clamp(req[i]-command_[i],-.08*dt,.08*dt);
    Vec world_cmd=R_*Vec(command_.x(),command_.y(),0);world_cmd.z()=0;
    for(int i=0;i<2;++i)reference_[i]=std::clamp(reference_[i],position_[i]-.07,position_[i]+.07);
    desired_yaw_+=command_.z()*dt;
    bool moving=command_.norm()>1e-4||single_leg_>=0&&!test_done_;
    if(phase_==STAND){body_target_=reference_+Eigen::AngleAxisd(desired_yaw_,Vec::UnitZ())*pose_offset_;body_target_.z()=desired_height_;
      if(moving&&switched_&&time-active_since_>1&&support==4){leg_=single_leg_>=0?single_leg_:order_[beat_];transfer_start_=position_;phase(TRANSFER);}}
    double elapsed=time-phase_start_;
    auto tri=triangle(leg_);Vec com_world=position_+R_*measured.com;com_velocity_=.85*com_velocity_+.15*(com_world-previous_com_)/dt;previous_com_=com_world;
    if(phase_==TRANSFER){Vec aim=reference_+Eigen::AngleAxisd(desired_yaw_,Vec::UnitZ())*pose_offset_+R_*measured.com;aim=project(aim,tri,margin_);Vec goal=aim-R_*measured.com;goal.z()=desired_height_;body_target_=transfer_start_+smooth(elapsed/transfer_time_)*(goal-transfer_start_);
      if(elapsed>=transfer_time_&&stability(com_world,tri)>margin_*.9&&std::abs(roll()-desired_roll_)<.04&&std::abs(pitch()-desired_pitch_)<.04&&velocity_.head<2>().norm()<.08){com_goal_=body_target_+R_*measured.com;phase(UNLOAD);}
      else if(elapsed>3)recover("weight transfer did not reach verified support margin");}
    elapsed=time-phase_start_;
    if(phase_==UNLOAD){targets_[leg_]=anchors_[leg_];targets_[leg_].z()+=.004*smooth(elapsed/.25);
      bool others=true;for(int l=0;l<4;++l)if(l!=leg_&&!contact(l))others=false;
      if(elapsed>.25&&contact_force_[leg_]<8&&others&&stability(com_world,tri)>.005){swing_start_=position_+R_*measured.feet[leg_];
        Vec nominal=Vec(planner_.base.legs[leg_]->zero_stance().X(),planner_.base.legs[leg_]->zero_stance().Y(),0);
        swing_end_=reference_+R_*nominal+world_cmd*(1.5*(transfer_time_+.25+swing_time_));swing_end_.z()=anchors_[leg_].z();landing_z_=swing_end_.z();cleared_=false;phase(SWING);}
      else if(elapsed>1)recover("swing foot failed to unload");}
    elapsed=time-phase_start_;
    if(phase_==SWING){double u=std::clamp(elapsed/swing_time_,0.,1.);targets_[leg_]=swing_start_+smooth(u)*(swing_end_-swing_start_);targets_[leg_].z()+=swing_height_*16*u*u*(1-u)*(1-u);
      Vec foot=position_+R_*measured.feet[leg_];if(foot.z()-landing_z_>.015&&!contact(leg_))cleared_=true;
      for(int l=0;l<4;++l)if(l!=leg_&&!contact(l)&&elapsed>.08)recover("lost a supporting foot during swing");
      if(phase_==SWING&&stability(com_world,tri)<.002&&elapsed>.08)recover("COM left the measured support margin");
      if(phase_==SWING&&u>=1){if(!cleared_)recover("actual swing foot clearance not confirmed");else phase(LAND);}}
    elapsed=time-phase_start_;
    if(phase_==LAND||phase_==RECOVER){targets_[leg_].z()=std::max(landing_z_-.02,targets_[leg_].z()-.04*dt);
      if(contact(leg_)&&elapsed>.08){anchors_[leg_]=position_+R_*measured.feet[leg_];anchors_[leg_].z()=ground_height_+foot_radius_-(R_*measured.foot_rotation[leg_]*Vec(-.002,0,0)).z();targets_[leg_]=anchors_[leg_];
        if(phase_==LAND){reference_.head<2>()+=world_cmd.head<2>()*(transfer_time_+.25+swing_time_+.08);beat_=(beat_+1)%4;if(single_leg_>=0)test_done_=true;}else test_done_=true;
        phase(STAND);}}
    for(int l=0;l<4;++l)if(l!=leg_||phase_==STAND||phase_==TRANSFER)targets_[l]=anchors_[l];
    elapsed=time-phase_start_;
    if(phase_==UNLOAD||phase_==SWING||phase_==LAND)body_target_=com_goal_-R_*measured.com;
    // Body wrench feedback. Use the measured full-body COM and composite inertia.
    Vec acceleration=60*(body_target_+R_*measured.com-com_world)-14*com_velocity_;acceleration.z()=80*(desired_height_-position_.z())-15*velocity_.z();for(int i=0;i<3;++i)acceleration[i]=std::clamp(acceleration[i],-2.,2.);
    Vec angular(160*(desired_roll_-roll()),160*(desired_pitch_-pitch()),100*wrap(desired_yaw_+pose_yaw_-yaw()));angular-=22*(R_*omega_);Mat inertia=R_*measured.inertia*R_.transpose();Vec moment=inertia*angular;
    std::array<Vec,4> arms;std::array<double,4> caps;
    for(int l=0;l<4;++l){arms[l]=R_*(measured.feet[l]-measured.com);caps[l]=contact(l)?140:0;}
    if(phase_==UNLOAD)caps[leg_]=140*(1-smooth(elapsed/.25));
    if(phase_==SWING||phase_==LAND||phase_==RECOVER)caps[leg_]=0;
    auto f=go2_gait::distribute(arms,dynamics_.mass*(acceleration+Vec(0,0,9.81)),moment,caps,forces_);forces_=f;
    auto J=dynamics_.jacobians(q_);auto gravity=dynamics_.gravity(q_,R_);
    sensor_msgs::msg::JointState out;out.header.stamp=now();out.name.assign(planner_.model.names.begin(),planner_.model.names.end());
    std::array<double,12> desired{};bool valid=true;
    for(int l=0;l<4;++l){Vec foot=R_.transpose()*(targets_[l]-position_);geometry::Transformation p;p.X()=foot.x();p.Y()=foot.y();p.Z()=foot.z();champ::Kinematics::transformToHip(p,*planner_.base.legs[l]);float h=NAN,a=NAN,b=NAN;champ::Kinematics::inverse(h,a,b,*planner_.base.legs[l],p);std::array<double,3> angles{h,a,b};
      for(int j=0;j<3;++j){int i=l*3+j;desired[i]=angles[j];if(!std::isfinite(desired[i])||desired[i]<planner_.model.lower[i]-1e-4||desired[i]>planner_.model.upper[i]+1e-4)valid=false;else desired[i]=std::clamp(desired[i],planner_.model.lower[i],planner_.model.upper[i]);}}
    if(!valid){recover("unreachable measured-frame foot target");desired=q_;}
    for(int l=0;l<4;++l){Vec tau=-J[l].transpose()*R_.transpose()*forces_[l];for(int j=0;j<3;++j){int i=3*l+j;out.position.push_back(desired[i]);out.velocity.push_back(0);out.effort.push_back(tau[j]+gravity[i]);}}
    pub_->publish(out);
    geometry_msgs::msg::PoseArray feet;feet.header.stamp=now();feet.header.frame_id="local_ground";for(auto p:targets_){geometry_msgs::msg::Pose v;v.position.x=p.x();v.position.y=p.y();v.position.z=p.z();v.orientation.w=1;feet.poses.push_back(v);}feet_pub_->publish(feet);
    std_msgs::msg::UInt8MultiArray planned,actual;for(int l=0;l<4;++l){planned.data.push_back(!(l==leg_&&(phase_==SWING||phase_==LAND||phase_==RECOVER)));actual.data.push_back(contact(l));}contacts_pub_->publish(planned);actual_pub_->publish(actual);
    std_msgs::msg::Float64MultiArray status;status.data={double(phase_),double(leg_),position_.x(),position_.y(),position_.z(),roll(),pitch(),yaw(),stability(com_world,tri),double(cleared_),contact_force_[0],contact_force_[1],contact_force_[2],contact_force_[3]};status_pub_->publish(status);
  }
public:
  ChampController():Node("go2_champ_controller"){
    max_x_=positive("max_velocity_x",.04);max_y_=positive("max_velocity_y",.02);max_yaw_=positive("max_yaw_rate",.10);height_=positive("nominal_height",.33913);transfer_time_=positive("transfer_duration",.5);swing_time_=positive("swing_duration",.4);swing_height_=positive("swing_height",.04);margin_=positive("support_margin",.025);single_leg_=declare_parameter("single_leg_test",-1);if(single_leg_<-1||single_leg_>3)throw std::runtime_error("single_leg_test must be -1 or 0..3");
    champ::GaitConfig cfg;cfg.nominal_height=height_;cfg.com_x_translation=0;planner_.base.setGaitConfig(cfg);
    pub_=create_publisher<sensor_msgs::msg::JointState>("/go2/balance_joint_command",1);feet_pub_=create_publisher<geometry_msgs::msg::PoseArray>("/go2/foot_targets",10);contacts_pub_=create_publisher<std_msgs::msg::UInt8MultiArray>("/go2/gait_contacts",10);actual_pub_=create_publisher<std_msgs::msg::UInt8MultiArray>("/go2/actual_contacts",10);status_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("/go2/balance_status",10);
    switch_=create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
    subscriptions_.push_back(create_subscription<std_msgs::msg::String>("/robot_description",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){if(loaded_)return;planner_.model.load(planner_.base,m->data,{"FL","FR","RL","RR"},"base");dynamics_.load(m->data,planner_.model.names);loaded_=true;}));
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr m){if(!loaded_)return;std::unordered_map<std::string,size_t>map;for(size_t i=0;i<m->name.size();++i)map[m->name[i]]=i;for(int i=0;i<12;++i){auto it=map.find(planner_.model.names[i]);if(it==map.end()||it->second>=m->position.size()||it->second>=m->velocity.size())return;size_t k=it->second;if(!std::isfinite(m->position[k])||!std::isfinite(m->velocity[k]))return;q_[i]=m->position[k];qd_[i]=m->velocity[k];}last_joint_=now().seconds();}));
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::Imu>("/go2/imu",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Imu::ConstSharedPtr m){auto q=m->orientation;Eigen::Quaterniond rot(q.w,q.x,q.y,q.z);if(!rot.coeffs().allFinite()||rot.norm()<.1)return;R_=rot.normalized().toRotationMatrix();omega_=Vec(m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z);last_imu_=now().seconds();}));
    for(int l=0;l<4;++l)subscriptions_.push_back(create_subscription<gazebo_msgs::msg::ContactsState>("/go2/"+std::array<std::string,4>{"FL","FR","RL","RR"}[l]+"/contact",rclcpp::SensorDataQoS(),[this,l](gazebo_msgs::msg::ContactsState::ConstSharedPtr m){double f=0;for(auto &s:m->states){auto v=s.total_wrench.force;f+=std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z);}
      // Gazebo accumulates consecutive 1 ms physics contact samples between sensor updates.
      contact_force_[l]=m->states.empty()?0:f/m->states.size();contact_time_[l]=now().seconds();}));
    subscriptions_.push_back(create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,[this](geometry_msgs::msg::Twist::ConstSharedPtr m){if(!std::isfinite(m->linear.x)||!std::isfinite(m->linear.y)||!std::isfinite(m->angular.z))return;if(std::abs(m->linear.x)+std::abs(m->linear.y)+std::abs(m->angular.z)<1e-5&&phase_==STAND)fault_=false;requested_=Vec(std::clamp(m->linear.x,-max_x_,max_x_),std::clamp(m->linear.y,-max_y_,max_y_),std::clamp(m->angular.z,-max_yaw_,max_yaw_));last_cmd_=now().seconds();}));
    subscriptions_.push_back(create_subscription<geometry_msgs::msg::Pose>("/body_pose",10,[this](geometry_msgs::msg::Pose::ConstSharedPtr m){if(!initialized_||!std::isfinite(m->position.x)||!std::isfinite(m->position.y)||!std::isfinite(m->position.z))return;
      auto q=m->orientation;Eigen::Quaterniond orientation(q.w,q.x,q.y,q.z);if(!orientation.coeffs().allFinite()||orientation.norm()<1e-6)return;
      Mat rot=orientation.normalized().toRotationMatrix();desired_roll_=std::clamp(std::atan2(rot(2,1),rot(2,2)),-.08,.08);desired_pitch_=std::clamp(std::asin(std::clamp(-rot(2,0),-1.,1.)),-.08,.08);pose_yaw_=std::clamp(std::atan2(rot(1,0),rot(0,0)),-.08,.08);
      desired_height_=default_height_+std::clamp(m->position.z,-.02,.02);pose_offset_=Vec(std::clamp(m->position.x,-.03,.03),std::clamp(m->position.y,-.03,.03),0);}));
    timer_=rclcpp::create_timer(get_node_base_interface(),get_node_timers_interface(),get_clock(),std::chrono::milliseconds(5),[this]{tick();});
  }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<ChampController>());}catch(const std::exception&e){fprintf(stderr,"Go2 balance failed: %s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
