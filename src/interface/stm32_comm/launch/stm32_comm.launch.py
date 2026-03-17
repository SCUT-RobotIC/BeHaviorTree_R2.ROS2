from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stm32_comm',
            executable='stm32_comm_node',
            name='stm32_comm_node',
            parameters=[
                {'serial_port': '/dev/ttyACM0'},
                {'baud_rate': 115200}
            ],
            output='screen'
        )
    ])
