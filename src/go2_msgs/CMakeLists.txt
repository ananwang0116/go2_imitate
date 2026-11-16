cmake_minimum_required(VERSION 3.8)
project(go2_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Event.msg
  msg/DevState.msg
  msg/LocStatus.msg
  msg/SafeStatus.msg
  msg/TaskCmd.msg
  msg/TaskStatus.msg
  msg/RecoverCmd.msg
  msg/NavGoal.msg
  msg/NavStatus.msg
  srv/NavCheck.srv
  DEPENDENCIES std_msgs diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
