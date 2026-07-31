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

if(BUILD_TESTING)
  ament_auto_add_gmock(gtest_${PROJECT_NAME}
    "test/src/test_expression.cpp"
  )
  target_include_directories(gtest_${PROJECT_NAME} PRIVATE "src")
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
