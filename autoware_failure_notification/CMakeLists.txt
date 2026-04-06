cmake_minimum_required(VERSION 3.14)
project(autoware_failure_notification)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/failure_notification.cpp
  src/core/notification.cpp
  src/core/message.cpp
  src/core/condition.cpp
  src/core/context.cpp
  src/core/expression.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::failure_notification::FailureNotification"
  EXECUTABLE failure_notification_node
)

ament_auto_package(INSTALL_TO_SHARE config launch)
