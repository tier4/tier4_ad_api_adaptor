cmake_minimum_required(VERSION 3.14)
project(tier4_maintenance_management)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(maintenance_state_store REQUIRED)
foreach(_dep ${${PROJECT_NAME}_BUILD_DEPENDS})
  message(WARNING "${_dep}_INCLUDE_DIRS = ${${_dep}_INCLUDE_DIRS}")
endforeach()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/maintenance_management.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tier4_maintenance_management::MaintenanceManagement"
  EXECUTABLE maintenance_management_node
)

ament_auto_package(INSTALL_TO_SHARE config launch)
