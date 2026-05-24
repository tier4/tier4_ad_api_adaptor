cmake_minimum_required(VERSION 3.14)
project(tier4_autoware_api_host)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(tier4_autoware_api_host plugins.xml)

set(AWAPI_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../awapi_awiv_adapter/src)

# Component library (host node + base class impl + relays)
ament_auto_add_library(tier4_autoware_api_host_component SHARED
  src/tier4_api_adaptor_plugin_base.cpp
  src/tier4_api_host_node.cpp
)

rclcpp_components_register_node(tier4_autoware_api_host_component
  PLUGIN "tier4_autoware_api_host::Tier4ApiHostNode"
  EXECUTABLE tier4_api_host_node
)

# Plugin library
ament_auto_add_library(tier4_autoware_api_host_plugins SHARED
  ${AWAPI_SRC_DIR}/awapi_vehicle_state_publisher.cpp
  ${AWAPI_SRC_DIR}/awapi_autoware_state_publisher.cpp
  ${AWAPI_SRC_DIR}/awapi_lane_change_state_publisher.cpp
  ${AWAPI_SRC_DIR}/awapi_obstacle_avoidance_state_publisher.cpp
  ${AWAPI_SRC_DIR}/awapi_autoware_util.cpp
  ${AWAPI_SRC_DIR}/awapi_velocity_factor_converter.cpp
  ${AWAPI_SRC_DIR}/awapi_v2x_aggregator.cpp
  ${AWAPI_SRC_DIR}/awapi_max_velocity_publisher.cpp
  src/adaptors/rtc_controller_host.cpp
  src/plugins/awapi_coordinator_plugin.cpp
  src/plugins/awapi_vehicle_status_plugin.cpp
  src/plugins/awapi_autoware_status_plugin.cpp
  src/plugins/awapi_lane_change_plugin.cpp
  src/plugins/awapi_obstacle_avoidance_plugin.cpp
  src/plugins/external_calibration_status_plugin.cpp
  src/plugins/external_cpu_usage_plugin.cpp
  src/plugins/external_diagnostics_plugin.cpp
  src/plugins/external_door_plugin.cpp
  src/plugins/external_emergency_plugin.cpp
  src/plugins/external_fail_safe_state_plugin.cpp
  src/plugins/external_initial_pose_plugin.cpp
  src/plugins/external_localization_score_plugin.cpp
  src/plugins/external_map_plugin.cpp
  src/plugins/external_metadata_packages_plugin.cpp
  src/plugins/external_operator_plugin.cpp
  src/plugins/external_rosbag_logging_mode_plugin.cpp
  src/plugins/external_route_plugin.cpp
  src/plugins/external_service_plugin.cpp
  src/plugins/external_start_plugin.cpp
  src/plugins/external_system_monitor_plugin.cpp
  src/plugins/external_vehicle_status_plugin.cpp
  src/plugins/external_velocity_plugin.cpp
  src/plugins/external_version_plugin.cpp
  src/plugins/internal_iv_msgs_plugin.cpp
  src/plugins/internal_operator_plugin.cpp
  src/plugins/internal_velocity_plugin.cpp
  src/plugins/extension_planning_factor_plugin.cpp
  src/plugins/extension_traffic_light_plugin.cpp
  src/plugins/extension_route_distance_plugin.cpp
  src/plugins/extension_velocity_limit_plugin.cpp
  src/plugins/deprecated_engage_plugin.cpp
  src/plugins/deprecated_manual_status_plugin.cpp
  src/plugins/deprecated_manual_control_plugin.cpp
  src/plugins/deprecated_hazard_status_plugin.cpp
  src/plugins/deprecated_initial_pose_plugin.cpp
  src/plugins/rtc_controller_plugin.cpp
)

find_package(glog REQUIRED)
target_link_libraries(tier4_autoware_api_host_plugins glog::glog)

target_include_directories(tier4_autoware_api_host_plugins PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../awapi_awiv_adapter/include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
