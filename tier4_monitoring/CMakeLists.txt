cmake_minimum_required(VERSION 3.14)
project(tier4_monitoring)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/monitoring.cpp
  src/driving.cpp
  src/lanelet.cpp
  src/message.cpp
  src/operator.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tier4_monitoring::Monitoring"
  EXECUTABLE monitoring_node
)

install(PROGRAMS
  script/debug.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(gtest_ros_${PROJECT_NAME}
    "test/test_main.cpp"
    "test/cases/test_driving_enable.cpp"
    "test/cases/test_operator_responsible.cpp"
    "test/cases/test_operator_status.cpp"
    "test/cases/test_velocity_limit.cpp"
    "test/cases/util/mock.cpp"
    "test/cases/util/fixture.cpp"
  )
  get_filename_component(RESOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/files" ABSOLUTE)
  target_compile_definitions(gtest_ros_${PROJECT_NAME} PRIVATE TEST_RESOURCE_PATH="${RESOURCE_PATH}")
  target_include_directories(gtest_ros_${PROJECT_NAME} PRIVATE "src")
  target_link_libraries(gtest_ros_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
