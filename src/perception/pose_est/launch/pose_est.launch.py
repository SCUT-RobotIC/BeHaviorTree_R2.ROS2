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
    pkg_share = Path(get_package_share_directory('pose_est'))
    default_param_file = str(pkg_share / 'config' / 'pose.yaml')
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
                    get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                )
            ),
            launch_arguments = {
                            'align_depth.enable' : 'true',
                            'enable_sync' : 'true',
                            'rgb_camera.color_profile' : '640x480x30',
                            'depth_module.depth_profile' : '640x480x30'
                        }.items()
        ),
        Node(
            package="pose_est",
            executable="auto_predict_node", #“auto_predict_node”
            name="auto_predict_node",
            output="screen",
            parameters=[LaunchConfiguration('params')]  # 设置debug_level为30（非debug模式）
        ),
    ])