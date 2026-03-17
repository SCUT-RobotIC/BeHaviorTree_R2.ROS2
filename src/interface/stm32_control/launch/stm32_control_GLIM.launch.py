from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        name='config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('stm32_control'),
            'config',
            'stm32_control_GLIM.yaml'
        ]),
        description='Path to stm32_control parameter file'
    )

    node = Node(
        package='stm32_control',
        executable='stm32_control_node',
        name='stm32_control_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        node
    ])
