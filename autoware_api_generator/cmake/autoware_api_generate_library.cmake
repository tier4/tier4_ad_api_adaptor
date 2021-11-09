# Copyright 2021 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

macro(autoware_api_generate_library target)

  # init path settings
  normalize_path(_build_path "${CMAKE_CURRENT_BINARY_DIR}/autoware_api_generator")
  normalize_path(_share_path "${autoware_api_generator_DIR}/..")

  # parse and check
  cmake_parse_arguments(_ARG "USE_GROUP_PATH" "REMAP;EXTRA" "SPECS" ${ARGN})
  if(NOT _ARG_REMAP)
    set(_remap_file "${_share_path}/config/remap.yaml")
  else()
    set(_remap_file "${CMAKE_CURRENT_SOURCE_DIR}/${_ARG_REMAP}")
  endif()
  if(NOT _ARG_EXTRA)
    set(_extra_file "${_share_path}/config/extra.yaml")
  else()
    set(_extra_file "${CMAKE_CURRENT_SOURCE_DIR}/${_ARG_EXTRA}")
  endif()

  # create specification file list
  set(_spec_files "")
  list(APPEND _spec_files "${_share_path}/resource/external.yaml")
  list(APPEND _spec_files "${_share_path}/resource/internal.yaml")

  # create generating file list
  set(_target_api_names "")
  set(_target_cpp_files "")
  set(_target_py_files "")
  set(_target_cmake_files "")
  foreach(_api ${_ARG_UNPARSED_ARGUMENTS})
    string(REGEX REPLACE "^/+" "" _api "${_api}")
    if(NOT ${_ARG_USE_GROUP_PATH})
      string(REGEX REPLACE "^[^/]+" "" _api "${_api}")
      string(REGEX REPLACE "^/+" "" _api "${_api}")
    endif()
    list(APPEND _target_api_names "${_api}")
    list(APPEND _target_cpp_files "${_build_path}/cpp/${_api}.hpp")
    list(APPEND _target_py_files "${_build_path}/py/${_api}.py")
    list(APPEND _target_cmake_files "${_build_path}/cmake/${_api}.cmake")
  endforeach()

  # set dependencies for main target
  set(autoware_api_generator_TARGET "${target}")
  add_custom_target(${autoware_api_generator_TARGET} ALL SOURCES ${_spec_files} ${_remap_file})
  add_custom_target(${autoware_api_generator_TARGET}__cmake SOURCES ${_target_cmake_files})

  # set dependencies for cpp target
  add_custom_target(${autoware_api_generator_TARGET}__cpp SOURCES ${_target_cpp_files})
  add_dependencies(${autoware_api_generator_TARGET} ${autoware_api_generator_TARGET}__cpp)

  # set dependencies for py target
  add_custom_target(${autoware_api_generator_TARGET}__py SOURCES ${_target_py_files})
  add_dependencies(${autoware_api_generator_TARGET} ${autoware_api_generator_TARGET}__py)

  # use a temporary file to avoid long command
  file(WRITE "${_build_path}/list.txt" "${_target_api_names}")

  # generate configure_file scripts
  add_custom_command(
      OUTPUT ${_target_cmake_files}
      COMMAND python3 ${_share_path}/script/configure.py
        "--package" ${PROJECT_NAME}
        "--build" ${_build_path}/cmake
        "--names" ${_build_path}/list.txt
        "--remap" ${_remap_file}
        "--extra" ${_extra_file}
        "--specs" ${_spec_files}
      DEPENDS
        ${_share_path}/script/configure.py
        ${_build_path}/list.txt
        ${_remap_file}
        ${_spec_files}
  )

  # generate cpp file from configure_file script
  foreach(_api ${_target_api_names})
    set(_api_output "${_build_path}/cpp/${_api}.hpp")
    set(_api_script "${_build_path}/cmake/${_api}.cmake")
    add_custom_command(
      OUTPUT ${_api_output}
      COMMAND ${CMAKE_COMMAND}
        -D INPUT=${_share_path}/template/cpp
        -D OUTPUT=${_api_output}
        -P ${_api_script}
      DEPENDS
        ${_share_path}/template/cpp/service.in
        ${_share_path}/template/cpp/topic.in
        ${autoware_api_generator_TARGET}__cmake
  )
  endforeach()

  # generate py file from configure_file script
  foreach(_api ${_target_api_names})
    set(_api_output "${_build_path}/py/${_api}.py")
    set(_api_script "${_build_path}/cmake/${_api}.cmake")
    add_custom_command(
      OUTPUT ${_api_output}
      COMMAND ${CMAKE_COMMAND}
        -D INPUT=${_share_path}/template/py
        -D OUTPUT=${_api_output}
        -P ${_api_script}
      DEPENDS
        ${_share_path}/template/py/service.in
        ${_share_path}/template/py/topic.in
        ${autoware_api_generator_TARGET}__cmake
  )
  endforeach()

  # install generated cpp files
  if(EXISTS ${_build_path})
    install(
      DIRECTORY ${_build_path}/cpp/
      DESTINATION include/${PROJECT_NAME}
      PATTERN *.hpp
    )
    ament_export_include_directories(include)
  endif()

  # install generated py files
  file(WRITE ${_build_path}/py/__init__.py "")
  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR "${_build_path}/py")

endmacro(autoware_api_generate_library)
