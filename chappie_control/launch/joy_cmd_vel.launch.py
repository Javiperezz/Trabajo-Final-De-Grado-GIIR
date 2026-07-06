
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chappie_control',
            executable='joy_cmd_vel_node',
            name='joy_cmd_vel',
            output='screen',
            emulate_tty=True,
        ),
    ])
