from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = Path(get_package_share_directory('robot_arm'))
    default_param_file = str(pkg_share / 'config' / 'arm.yaml')
    params_arg = LaunchConfiguration('params')
    return LaunchDescription([
        DeclareLaunchArgument(
            "params",
            default_value=default_param_file,
            description="Path to parameter YAML file"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(           
                    get_package_share_directory('pose_est'),
                    'launch',
                    'pose_est.launch.py'
                )
            ),
        ),
        Node(
            package="robot_arm",
            executable="robot_arm_node",
            name="robot_arm_node",
            output="screen",
            parameters=[params_arg]
        ),
    ])