from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory
import os, yaml


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package").perform(context)
    env_config = LaunchConfiguration("env_config").perform(context)
    env_parameters = PathJoinSubstitution([FindPackageShare(runtime_config_package), env_config])
    environment_node = Node(
        package="lidar_obstacle_detection",
        executable="environment",
        name="lidar_obstacle_detection",
        emulate_tty=True,
        parameters=[env_parameters],
        output="screen",
    )

    rviz_config_file = PathJoinSubstitution([FindPackageShare("lidar_obstacle_detection"), "config", "rviz_conf.rviz"])
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        executable="rviz2",
        name="rviz2_pcl",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    tf_node = Node(package="tf2_ros", executable="static_transform_publisher", arguments=["0", "0", "1.7", "0", "0", "0", "world", "laser3d"])

    nodes = [environment_node, rviz_node, tf_node]

    return nodes


def generate_launch_description():
    # Declare Launch Arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument(
        "runtime_config_package",
        default_value="lidar_obstacle_detection",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "env_config",
        default_value="config/environment.yaml",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
    ))
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
