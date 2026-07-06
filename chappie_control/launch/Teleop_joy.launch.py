from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    default_params = PathJoinSubstitution([
        FindPackageShare('chappie_control'),
        'config',
        'teleop_joy_params.yaml',
    ])

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML file with joy + teleop_twist_joy parameters',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[('/cmd_vel', '/cmd_vel')],   
    )

    return LaunchDescription([
        params_arg,
        joy_node,
        teleop_node,
    ])
