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
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


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
        namespace="internal",
        name=node_name,
        package="autoware_iv_internal_api_adaptor",
        plugin="internal_api::" + class_name,
        **kwargs,
    )


def generate_launch_description():
    container = ComposableNodeContainer(
        namespace="internal",
        name="autoware_iv_adaptor",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            _create_api_node(
                "iv_msgs",
                "IVMsgs",
                parameters=[
                    {"launch_api_0_4_3": LaunchConfiguration("launch_api_0_4_3")},
                ],
            ),
        ],
        ros_arguments=[
            "--log-level",
            Namespace(".", "internal.autoware_iv_adaptor:=WARN"),
        ],
        output="screen",
    )
    loader_0_4_3 = LoadComposableNodes(
        target_container=Namespace("/", "internal/autoware_iv_adaptor"),
        condition=IfCondition(LaunchConfiguration("launch_api_0_4_3")),
        composable_node_descriptions=[
            _create_api_node("operator", "Operator"),
            _create_api_node("velocity", "Velocity"),
        ],
    )
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("launch_api_0_4_3", default_value="false"),
            container,
            loader_0_4_3,
        ]
    )
