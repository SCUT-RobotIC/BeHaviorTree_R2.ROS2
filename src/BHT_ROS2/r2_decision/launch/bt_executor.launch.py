from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = Path(get_package_share_directory("r2_decision"))
    executor_params = str(pkg_share / "config" / "bt_executor.yaml")
    r2_initial_params = str(
        Path(get_package_share_directory("r2_initial_bt")) / "config" / "r2_initial_bt.yaml"
    )
    tf_listen_params = str(
        Path(get_package_share_directory("tf_listen_bt")) / "config" / "tf_listen_bt.yaml"
    )
    camera_stag_params = str(
        Path(get_package_share_directory("camera_stag_bt")) / "config" / "camera_stag_bt.yaml"
    )
    glim_config = os.path.join(
        get_package_share_directory("glim"),
        "config"
    )
    stm32_comm_launch = os.path.join(
        get_package_share_directory('stm32_comm'),
        'launch',
        'stm32_comm.launch.py'
    )
    stm32_control_launch = os.path.join(
        get_package_share_directory('stm32_control'),
        'launch',
        'stm32_control.launch.py'
    )
    livox_launch = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'launch',
        'msg_MID360_side.launch.py'
    )
    pointlio_launch = os.path.join(
        get_package_share_directory('point_lio'),
        'launch',
        'point_lio.launch.py'
    )
    pose_est_launch = os.path.join(
        get_package_share_directory('pose_est'),
        'launch',
        'pose_est.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params",
            default_value=executor_params,
            description="Path to BT executor parameter YAML"
        ),
        DeclareLaunchArgument(
            "r2_initial_params",
            default_value=r2_initial_params,
            description="Path to r2_initial_bt parameter YAML"
        ),
        DeclareLaunchArgument(
            "tf_listen_params",
            default_value=tf_listen_params,
            description="Path to tf_listen_bt parameter YAML"
        ),
        DeclareLaunchArgument(
            "camera_stag_params",
            default_value=camera_stag_params,
            description="Path to camera_stag_bt parameter YAML"
        ),
        DeclareLaunchArgument(
            "localization",
            default_value="pointlio",
            choices=["pointlio", "glim"],
            description="Localization backend for decision launch"
        ),
        # 上下位机通讯节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                stm32_comm_launch
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                stm32_control_launch
            ),
        ),
        # PointLIO定位节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                livox_launch
            ),
            condition=LaunchConfigurationEquals("localization", "pointlio"),
            launch_arguments=[("use_sim_time", "false")]
        ),
        # PointLIO定位节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pointlio_launch
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
                pose_est_launch
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
            output="screen",
            parameters=[
                LaunchConfiguration("params"),
                LaunchConfiguration("r2_initial_params"),
                LaunchConfiguration("tf_listen_params"),
                LaunchConfiguration("camera_stag_params")
            ]
        ),
    ])
