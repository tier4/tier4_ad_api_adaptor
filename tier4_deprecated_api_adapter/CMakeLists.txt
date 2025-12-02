cmake_minimum_required(VERSION 3.14)
project(tier4_deprecated_api_adapter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/autoware_engage.cpp
  src/initial_pose.cpp
  src/manual_control.cpp
  src/manual_status.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "tier4_deprecated_api_adapter::InitialPose"
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tier4_deprecated_api_adapter::AutowareEngage"
  EXECUTABLE autoware_engage_node
  EXECUTOR MultiThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tier4_deprecated_api_adapter::ManualControl"
  EXECUTABLE manual_control_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tier4_deprecated_api_adapter::ManualStatus"
  EXECUTABLE manual_status_node
)

ament_auto_package(INSTALL_TO_SHARE launch)
