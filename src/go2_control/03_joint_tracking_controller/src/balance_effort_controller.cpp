#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace go2_control {
class BalanceEffortController : public controller_interface::ControllerInterface {
  struct Target { std::array<double,12> q{},qd{},tau{};double stamp=-1e9; };
  std::vector<std::string> joints_;
  std::array<double,12> hold_{};
  realtime_tools::RealtimeBuffer<Target> target_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  double kp_hip_=50,kp_leg_=100,kd_=2,timeout_=.1;
public:
  controller_interface::CallbackReturn on_init() override {
    auto_declare<std::vector<std::string>>("joints",{});
    auto_declare<double>("kp_hip",50);auto_declare<double>("kp_leg",100);
    auto_declare<double>("kd",2);auto_declare<double>("command_timeout",.1);
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration c;c.type=controller_interface::interface_configuration_type::INDIVIDUAL;
    for(auto &j:joints_)c.names.push_back(j+"/effort");return c;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    controller_interface::InterfaceConfiguration c;c.type=controller_interface::interface_configuration_type::INDIVIDUAL;
    for(auto &j:joints_){c.names.push_back(j+"/position");c.names.push_back(j+"/velocity");}return c;
  }
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    joints_=get_node()->get_parameter("joints").as_string_array();if(joints_.size()!=12)return controller_interface::CallbackReturn::ERROR;
    kp_hip_=get_node()->get_parameter("kp_hip").as_double();kp_leg_=get_node()->get_parameter("kp_leg").as_double();kd_=get_node()->get_parameter("kd").as_double();timeout_=get_node()->get_parameter("command_timeout").as_double();
    sub_=get_node()->create_subscription<sensor_msgs::msg::JointState>("/go2/balance_joint_command",rclcpp::QoS(1),[this](sensor_msgs::msg::JointState::SharedPtr m){
      if(m->name!=joints_||m->position.size()!=12||m->velocity.size()!=12||m->effort.size()!=12)return;
      Target t;t.stamp=rclcpp::Time(m->header.stamp).seconds();
      for(size_t i=0;i<12;++i){if(!std::isfinite(m->position[i])||!std::isfinite(m->velocity[i])||!std::isfinite(m->effort[i]))return;t.q[i]=m->position[i];t.qd[i]=m->velocity[i];t.tau[i]=m->effort[i];}target_.writeFromNonRT(t);
    });return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    for(size_t i=0;i<12;++i)hold_[i]=state_interfaces_[2*i].get_value();
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    for(auto &c:command_interfaces_)c.set_value(0);return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &time,const rclcpp::Duration &) override {
    const auto t=*target_.readFromRT();bool fresh=time.seconds()-t.stamp<timeout_ && time.seconds()>=t.stamp;
    for(size_t i=0;i<12;++i){double q=state_interfaces_[2*i].get_value(),v=state_interfaces_[2*i+1].get_value();
      if(fresh)hold_[i]=t.q[i];
      double tau=(fresh?t.tau[i]:0)+(i%3==0?kp_hip_:kp_leg_)*(hold_[i]-q)+kd_*((fresh?t.qd[i]:0)-v);
      double limit=i%3==2?45.43:23.7;command_interfaces_[i].set_value(std::clamp(tau,-limit,limit));
    }return controller_interface::return_type::OK;
  }
};
}
PLUGINLIB_EXPORT_CLASS(go2_control::BalanceEffortController, controller_interface::ControllerInterface)
