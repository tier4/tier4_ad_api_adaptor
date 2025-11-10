from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration


def current_ros_namespace(context):
    namespace = context.launch_configurations.get("ros_namespace", "")
    return [SetLaunchConfiguration("current_ros_namespace", namespace)]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=current_ros_namespace)])
