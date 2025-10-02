#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation time'
    )

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('tetra_gazebo_sim'),
            'worlds',
            'tetra_office.sdf'
        ]),
        description='Path to the Gazebo world file'
    )

    # Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tetra_gazebo_sim'),
                'launch',
                'gazebo_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world_file': LaunchConfiguration('world_file')
        }.items()
    )

    # EKF Localization (modified for simulation)
    ekf_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('tetra_gazebo_sim'),
                'params',
                'ekf_sim.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # Joy node for teleop (optional)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        parameters=[
            {"deadzone": 0.05},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Teleop twist joy for manual control
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'axis_linear.x': 1},
            {'axis_angular.yaw': 0},
            {'scale_linear.x': 0.5},
            {'scale_angular.yaw': 1.0},
            {'enable_button': 0}
        ]
    )

    # rosbridge_server for web interface
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        parameters=[
            {"port": 9090},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # rosapi_node
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_file_arg,
        
        gazebo_launch,
        ekf_localization_node,
        joy_node,
        teleop_twist_joy_node,
        rosbridge_server,
        rosapi_node
    ])