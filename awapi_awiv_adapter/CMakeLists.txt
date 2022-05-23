cmake_minimum_required(VERSION 3.5)
project(awapi_awiv_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(awapi_awiv_adapter
  src/awapi_awiv_adapter_node.cpp
  src/awapi_awiv_adapter_core.cpp
  src/awapi_vehicle_state_publisher.cpp
  src/awapi_autoware_state_publisher.cpp
  src/awapi_stop_reason_aggregator.cpp
  src/awapi_v2x_aggregator.cpp
  src/awapi_lane_change_state_publisher.cpp
  src/awapi_obstacle_avoidance_state_publisher.cpp
  src/awapi_max_velocity_publisher.cpp
  src/awapi_autoware_util.cpp
  src/awapi_motion_factor_aggregator.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(awapi_awiv_adapter PRIVATE
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
