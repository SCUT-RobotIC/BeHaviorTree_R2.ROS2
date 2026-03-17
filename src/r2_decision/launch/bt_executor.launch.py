from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = Path(get_package_share_directory("r2_decision"))
    default_params = str(pkg_share / "config" / "bt_executor.yaml")
    params_arg = LaunchConfiguration("params")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params",
            default_value=default_params,
            description="Path to BT executor parameter YAML"
        ),
        Node(
            package="btcpp_ros2_samples",
            executable="sample_bt_executor",
            name="bt_executor",
            output="screen",
            parameters=[params_arg]
        ),
        Node(
            package="camera_stag",
            executable="camera_stag_node",
            name="camera_stag",
            output="screen"
        ),
        Node(
            package="move_to_position",
            executable="move_to_position_node",
            name="move_to_position",
            output="screen"
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
            package="yolo_spearhead_see",
            executable="yolo_spearhead_see",
            name="yolo_spearhead_see",
            output="screen"
        ),
    ])
