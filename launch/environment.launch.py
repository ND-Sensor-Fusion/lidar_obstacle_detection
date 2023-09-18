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
        package="lidar_obstacle_detection", executable="environment",
        name="lidar_obstacle_detection",
        emulate_tty=True,
        parameters=[env_parameters],
        output="screen",
    )

    nodes = [environment_node]
    
    return nodes
    
def generate_launch_description():
    # Declare Launch Arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("runtime_config_package", default_value="lidar_obstacle_detection",))
    declared_arguments.append(DeclareLaunchArgument("env_config", default_value="config/environment.yaml",))
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
