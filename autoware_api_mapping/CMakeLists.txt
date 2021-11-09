cmake_minimum_required(VERSION 3.5)
project(autoware_api_mapping)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

autoware_api_generate_library(${PROJECT_NAME}
  "external/set/engage"
  "external/get/engage"
  "internal/set/engage"
  "internal/get/engage"
  USE_GROUP_PATH
  EXTRA "api/extra.yaml"
  REMAP "api/remap.yaml"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
