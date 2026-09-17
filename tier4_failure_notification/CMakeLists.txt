cmake_minimum_required(VERSION 3.14)
project(tier4_failure_notification)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/failure_notification.cpp
  src/core/notification.cpp
  src/core/failure.cpp
  src/core/condition.cpp
  src/core/context.cpp
  src/core/expression.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::failure_notification::FailureNotification"
  EXECUTABLE failure_notification_node
)

install(PROGRAMS
  script/message.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_auto_add_gmock(gtest_unit_${PROJECT_NAME}
    "test/unit/src/test_condition.cpp"
    "test/unit/src/test_expression.cpp"
  )
  target_include_directories(gtest_unit_${PROJECT_NAME} PRIVATE "src")

  ament_add_ros_isolated_gtest(gtest_ros_${PROJECT_NAME}
    "test/ros/src/test_main.cpp"
    "test/ros/src/test_node.cpp"
    "test/ros/src/mock/mock.cpp"
  )
  get_filename_component(RESOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/ros/files" ABSOLUTE)
  target_compile_definitions(gtest_ros_${PROJECT_NAME} PRIVATE TEST_RESOURCE_PATH="${RESOURCE_PATH}")
  target_include_directories(gtest_ros_${PROJECT_NAME} PRIVATE "src")
  target_link_libraries(gtest_ros_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
