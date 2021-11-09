cmake_minimum_required(VERSION 3.5)
project(autoware_api_generator)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY cmake config resource script template
  DESTINATION share/${PROJECT_NAME}
)

ament_package(CONFIG_EXTRAS "autoware_api_generator-extras.cmake")
