cmake_minimum_required(VERSION 3.14)
project(tier4_deprecated_api_adapter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/manual/manual_control.cpp"
  "src/manual/manual_mode.cpp"
  "src/manual/manual_status.cpp"
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tier4_deprecated_api_adapter::ManualControl"
  EXECUTABLE "manual_control_node"
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tier4_deprecated_api_adapter::ManualMode"
  EXECUTABLE "manual_mode_node"
  EXECUTOR MultiThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tier4_deprecated_api_adapter::ManualStatus"
  EXECUTABLE "manual_status_node"
  EXECUTOR SingleThreadedExecutor
)

ament_auto_package(INSTALL_TO_SHARE launch)
