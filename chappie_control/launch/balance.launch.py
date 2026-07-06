import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    default_params = PathJoinSubstitution([
        FindPackageShare('chappie_control'),
        'config',
        'balance_params.yaml',
    ])

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML file with balance node parameters',
    )

    balance_node = Node(
        package='chappie_control',
        executable='balance_node',
        name='chappie_balance',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True,
    )

    return LaunchDescription([
        params_arg,
        balance_node,
    ])
