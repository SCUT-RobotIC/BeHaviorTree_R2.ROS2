from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    ])
