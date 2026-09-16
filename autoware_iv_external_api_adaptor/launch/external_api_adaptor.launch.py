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
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# Nodes derived from autoware::agnocast_wrapper::Node, as (node name, class name, executable).
# Composed like any other node under ENABLE_AGNOCAST=0, where that base is backed by rclcpp; run as
# their own process under =1.
AGNOCAST_WRAPPER_NODES = [
    ("cpu_usage", "CpuUsage", "cpu_usage_node"),
    ("localization_score", "LocalizationScore", "localization_score_node"),
    ("system_monitor", "SystemMonitor", "system_monitor_node"),
]


# Usage: If the current namespace is /ros/ns:
#  - Namespace("/", "foo/bar") -> "ros/ns/foo/bar"
#  - Namespace(".", "foo.bar") -> "ros.ns.foo.bar"
class Namespace(Substitution):
    def __init__(self, separator, suffix):
        super().__init__()
        self.separator = separator
        self.suffix = suffix

    def perform(self, context):
        namespace = context.launch_configurations.get("ros_namespace", "")
        namespace = f"{namespace}{self.separator}{self.suffix}"
        return namespace.replace("/", self.separator).lstrip(self.separator)


def _create_api_node(node_name, class_name, **kwargs):
    return ComposableNode(
        namespace="external",
        name=node_name,
        package="autoware_iv_external_api_adaptor",
        plugin="external_api::" + class_name,
        **kwargs,
    )


def _create_standalone_api_node(node_name, executable):
    """Launch one AGNOCAST_WRAPPER_NODES entry as its own process."""
    return Node(
        namespace="external",
        name=node_name,
        package="autoware_iv_external_api_adaptor",
        executable=executable,
        additional_env={"LD_PRELOAD": LaunchConfiguration("ld_preload_value")},
        output="screen",
    )


def _get_agnocast_env():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("autoware_agnocast_wrapper"),
                    "launch",
                    "agnocast_env.launch.py",
                ]
            )
        )
    )


def launch_setup(context, *args, **kwargs):
    use_agnocast = context.perform_substitution(LaunchConfiguration("use_agnocast")) == "1"

    # RTCController is launched by tier4_autoware_api_launch because it is used by autoware_universe.
    components = [
        _create_api_node("calibration_status", "CalibrationStatus"),
        _create_api_node("map", "Map"),
        _create_api_node("metadata_packages", "MetadataPackages"),
        _create_api_node("rosbag_logging_mode", "RosbagLoggingMode"),
        _create_api_node("version", "Version"),
    ]
    nodes = []
    for node_name, class_name, executable in AGNOCAST_WRAPPER_NODES:
        if use_agnocast:
            nodes.append(_create_standalone_api_node(node_name, executable))
        else:
            components.append(_create_api_node(node_name, class_name))

    container = ComposableNodeContainer(
        namespace="external",
        name="autoware_iv_adaptor",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=components,
        ros_arguments=[
            "--log-level",
            Namespace(".", "external.autoware_iv_adaptor:=WARN"),
        ],
        output="screen",
    )
    loader_0_4_3 = LoadComposableNodes(
        target_container=Namespace("/", "external/autoware_iv_adaptor"),
        condition=IfCondition(LaunchConfiguration("launch_api_0_4_3")),
        composable_node_descriptions=[
            _create_api_node("diagnostics", "Diagnostics"),
            _create_api_node("door", "Door"),
            _create_api_node("fail_safe_state", "FailSafeState"),
            _create_api_node("initial_pose", "InitialPose"),
            _create_api_node("operator", "Operator"),
            _create_api_node("route", "Route"),
            _create_api_node("start", "Start"),
            _create_api_node("vehicle_status", "VehicleStatus"),
            _create_api_node("velocity", "Velocity"),
        ],
    )
    loader_0_4_4 = LoadComposableNodes(
        target_container=Namespace("/", "external/autoware_iv_adaptor"),
        condition=IfCondition(LaunchConfiguration("launch_api_0_4_4")),
        composable_node_descriptions=[
            _create_api_node("emergency", "Emergency"),
        ],
    )
    return [container, loader_0_4_3, loader_0_4_4, *nodes]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("launch_api_0_4_3", default_value="false"),
            DeclareLaunchArgument("launch_api_0_4_4", default_value="false"),
            _get_agnocast_env(),
            OpaqueFunction(function=launch_setup),
        ]
    )
