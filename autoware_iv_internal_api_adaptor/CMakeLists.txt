cmake_minimum_required(VERSION 3.14)
project(autoware_iv_internal_api_adaptor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/initial_pose.cpp
  src/iv_msgs.cpp
  src/operator.cpp
  src/route.cpp
  src/velocity.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "internal_api::InitialPose"
  "internal_api::IVMsgs"
  "internal_api::Operator"
  "internal_api::Route"
  "internal_api::Velocity"
)

ament_auto_package()
