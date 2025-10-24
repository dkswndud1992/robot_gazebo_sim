#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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
            FindPackageShare('robot_gazebo_sim'),
            'worlds',
            'robot_office.sdf'
        ]),
        description='Path to the Gazebo world file'
    )

    slam_arg = DeclareLaunchArgument(
        'slam', 
        default_value='true',
        description='Whether to run SLAM'
    )

    navigation_arg = DeclareLaunchArgument(
        'navigation', 
        default_value='true',
        description='Whether to run Navigation2'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', 
        default_value='true',
        description='Whether to start RViz'
    )

    # Include bringup launch
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_gazebo_sim'),
                'launch',
                'sim_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world_file': LaunchConfiguration('world_file')
        }.items()
    )

    # Include navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_gazebo_sim'),
                'launch',
                'sim_navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam': LaunchConfiguration('slam')
        }.items()
    )

    # RViz launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_gazebo_sim'),
                'launch',
                'rviz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_file_arg,
        slam_arg,
        navigation_arg,
        rviz_arg,
        
        bringup_launch,
        navigation_launch,
        rviz_launch
    ])