cmake_minimum_required(VERSION 3.14)
project(tier4_monitoring)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/monitoring.cpp
  src/operator.cpp
  src/message.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tier4_monitoring::Monitoring"
  EXECUTABLE monitoring_node
)

install(PROGRAMS
  script/debug.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE config launch)
