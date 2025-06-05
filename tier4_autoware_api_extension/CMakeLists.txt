cmake_minimum_required(VERSION 3.14)
project(tier4_autoware_api_extension)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/route_distance.cpp
  src/traffic_light.cpp
  src/planning_factor.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tier4_autoware_api_extension::RouteDistance"
  EXECUTABLE route_distance_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tier4_autoware_api_extension::TrafficLight"
  EXECUTABLE traffic_light_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tier4_autoware_api_extension::PlanningFactor"
  EXECUTABLE planning_factor_node
)

ament_auto_package(INSTALL_TO_SHARE config launch)
