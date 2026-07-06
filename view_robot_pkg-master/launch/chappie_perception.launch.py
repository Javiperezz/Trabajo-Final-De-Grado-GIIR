import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    urdf_path = PathJoinSubstitution([
        FindPackageShare('view_robot_pkg'),
        'urdf',
        'robot_description.urdf',
    ])

 
    robot_description = {'robot_description': Command(['cat ', urdf_path])}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

 
    lidar_back = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_back',
        output='screen',
        parameters=[{
            'serial_port':     '/dev/rplidar_back',
            'serial_baudrate': 115200,
            'frame_id':        'lidar_link',
            'inverted':        False,
            'angle_compensate': True,
            'scan_mode':       'Standard',
            'range_min':        0.20,

        }],
        remappings=[('scan', 'scan_back')],
    )


    lidar_front = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_front',
        output='screen',
        parameters=[{
            'serial_port':     '/dev/rplidar_front',
            'serial_baudrate': 115200,
            'frame_id':        'lidar1_link',
            'inverted':        False,
            'angle_compensate': True,
            'scan_mode':       'Standard',
            'range_min':        0.20,


        }],
        remappings=[('scan', 'scan_front')],
    )

    # Unimos los dos escaners de los lidars 
    merger = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node',
        name='dual_laser_merger',
        output='screen',
        parameters=[{
            'laser_1_topic':  '/scan_back',
            'laser_2_topic':  '/scan_front',
            'merged_topic':   '/scan',
            'target_frame':   'base_link',
            'use_inf':        True,
            'angle_increment': 0.005,
            'scan_time':      0.1,
            'range_min':      0.10,
            'range_max':      12.0,
            'min_height':     -1.0,
            'max_height':      1.0,
            'angle_min':      -3.14159,
            'angle_max':       3.14159,
            'inf_epsilon':     1.0,
        }],
        remappings=[('merged', 'scan')],

    )

    return LaunchDescription([
        rsp,
        jsp,
        lidar_back,
        lidar_front,
        merger,
    ])
