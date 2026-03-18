from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = Path(get_package_share_directory("r2_decision"))
    default_params = str(pkg_share / "config" / "bt_executor.yaml")
    params_arg = LaunchConfiguration("params")
    localization_arg = LaunchConfiguration("localization")
    glim_config = os.path.join(
        get_package_share_directory("glim"),
        "config"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params",
            default_value=default_params,
            description="Path to BT executor parameter YAML"
        ),
        DeclareLaunchArgument(
            "localization",
            default_value="pointlio",
            choices=["pointlio", "glim"],
            description="Localization backend for decision launch"
        ),
        # 上下位机通讯节点
        Node(
            package="stm32_comm",
            executable="stm32_comm_node",
            name="stm32_comm",
            output="screen"
        ),
        # PointLIO定位节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(           
                    get_package_share_directory('point_lio'),
                    'launch',
                    'point_lio.launch.py'
                )
            ),
            condition=LaunchConfigurationEquals("localization", "pointlio"),
            launch_arguments=[("use_sim_time", "false")]
        ),
        # GLIM定位节点
        Node(
            condition=LaunchConfigurationEquals("localization", "glim"),
            package="glim_ros",
            executable="glim_rosnode",
            name="glim_rosnode",
            output="screen",
            parameters=[{"config_path": glim_config}]
        ),
        # 梯田相机驱动节点
        Node(
            package="terraced_camera",
            executable="terraced_camera_node",
            name="terraced_camera",
            output="screen",
            parameters=[{"flip_mode": 1}]
        ),
        # 矛头对齐节点
        Node(
            package="camera_stag",
            executable="camera_stag_node",
            name="camera_stag",
            output="screen"
        ),
        # 移动到位置节点
        Node(
            package="move_to_position",
            executable="move_to_position_node",
            name="move_to_position",
            output="screen"
        ),
        # 机械臂控制节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(           
                    get_package_share_directory('pose_est'),
                    'launch',
                    'pose_est.launch.py'
                )
            ),
        ),
        # 矛头夹取节点
        Node(
            package="yolo_spearhead_see",
            executable="yolo_spearhead_see_node",
            name="yolo_spearhead_see",
            output="screen"
        ),
        # 行为树执行器节点
        Node(
            package="btcpp_ros2_samples",
            executable="sample_bt_executor",
            name="bt_executor",
            output="screen"
        ),
    ])
