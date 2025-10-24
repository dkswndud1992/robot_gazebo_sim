#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Flag to enable use_sim_time for simulation'
    )

    map_name_arg = DeclareLaunchArgument(
        'map_name', 
        default_value='office.yaml',
        description='Name of the map file to use'
    )

    # For simulation, we'll use SLAM instead of localization by default
    slam_launch_arg = DeclareLaunchArgument(
        'slam', 
        default_value='true',
        description='Whether to run SLAM'
    )

    # Path to the Slam Toolbox launch file
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('robot_navigation2'),
        'launch',
        'navigation_launch.py'
    )

    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('robot_navigation2'),
        'launch',
        'localization_launch.py'
    )

    slam_params_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo_sim'),
        'params',
        'slam_toolbox_sim.yaml'
    ])

    localization_params_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo_sim'),
        'params',
        'amcl_localization_sim.yaml'
    ])

    navigation_params_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo_sim'),
        'params',
        'navigation_sim.yaml'
    ])

    # Dynamically constructed paths
    map_file_path = PathJoinSubstitution([
        FindPackageShare('robot_navigation2'), 'maps', LaunchConfiguration('map_name')
    ])

    # SLAM launch (for simulation)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        condition=IfCondition(LaunchConfiguration('slam')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': slam_params_path,
        }.items()
    )

    # Localization launch (when not using SLAM)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        condition=IfCondition(LaunchConfiguration('slam')),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': localization_params_path,
                'map': map_file_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
                'map': map_file_path,
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        slam_launch_arg,
        slam_launch,
        localization_launch,
        navigation_launch
    ])