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

import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def _create_api_node(node_name, class_name, **kwargs):
    return ComposableNode(
        namespace="external",
        name=node_name,
        package="autoware_iv_external_api_adaptor",
        plugin="external_api::" + class_name,
        **kwargs
    )


def generate_launch_description():
    # RTCController is launched by tier4_autoware_api_launch because it is used by autoware_universe.
    components1 = [
        _create_api_node("service", "Service"),
    ]
    components2 = [
        _create_api_node("diagnostics", "Diagnostics"),
        _create_api_node("door", "Door"),
        _create_api_node("emergency", "Emergency"),
        _create_api_node("fail_safe_state", "FailSafeState"),
        _create_api_node("initial_pose", "InitialPose"),
        _create_api_node("operator", "Operator"),
        _create_api_node("route", "Route"),
        _create_api_node("start", "Start"),
        _create_api_node("vehicle_status", "VehicleStatus"),
        _create_api_node("velocity", "Velocity"),
    ]
    components3 = [
        _create_api_node("calibration_status", "CalibrationStatus"),
        _create_api_node("cpu_usage", "CpuUsage"),
        _create_api_node("localization_score", "LocalizationScore"),
        _create_api_node("map", "Map"),
        _create_api_node("metadata_packages", "MetadataPackages"),
        _create_api_node("rosbag_logging_mode", "RosbagLoggingMode"),
        _create_api_node("system_monitor", "SystemMonitor"),
        _create_api_node("version", "Version"),
    ]

    container = ComposableNodeContainer(
        namespace="external",
        name="autoware_iv_adaptor",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=components3,
        ros_arguments=["--log-level", "autoware_api.external.autoware_iv_adaptor:=WARN"],
        output="screen",
    )
    loader1 = LoadComposableNodes(
        composable_node_descriptions=components1,
        target_container="/autoware_api/external/autoware_iv_adaptor",
        condition=IfCondition(PythonExpression([LaunchConfiguration("api_mode"), " < 1"])),
    )
    loader2 = LoadComposableNodes(
        composable_node_descriptions=components2,
        target_container="/autoware_api/external/autoware_iv_adaptor",
        condition=IfCondition(PythonExpression([LaunchConfiguration("api_mode"), " < 2"])),
    )

    argument = DeclareLaunchArgument("api_mode", default_value="0")
    return launch.LaunchDescription([argument, container, loader1, loader2])
