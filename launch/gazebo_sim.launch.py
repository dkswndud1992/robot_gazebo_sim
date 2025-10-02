#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package paths
    tetra_gazebo_sim_path = get_package_share_directory('tetra_gazebo_sim')
    
    # Set Gazebo resource path to find meshes
    # Gazebo Harmonic looks for model://package_name structure
    # We need to provide the parent of 'share' directory (install/tetra_gazebo_sim)
    # So that Gazebo can find: install/tetra_gazebo_sim/share/tetra_gazebo_sim/meshes
    install_base = os.path.dirname(os.path.dirname(tetra_gazebo_sim_path))  # .../install
    pkg_parent = os.path.dirname(tetra_gazebo_sim_path)  # .../install/tetra_gazebo_sim/share
    
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            install_base,
            pkg_parent,
            tetra_gazebo_sim_path,
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ])
    )

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation time if true'
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

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='tetra',
        description='Name of the robot'
    )

    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position of the robot'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position of the robot'
    )

    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.0',
        description='Z position of base_footprint (0 = on ground, internal URDF offsets handle wheel contact)'
    )

    roll_arg = DeclareLaunchArgument(
        'roll',
        default_value='0.0',
        description='Roll angle of the robot (rotation around X axis)'
    )

    pitch_arg = DeclareLaunchArgument(
        'pitch',
        default_value='0.0',
        description='Pitch angle of the robot (rotation around Y axis)'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Yaw angle of the robot (rotation around Z axis)'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('tetra_gazebo_sim'),
                    'urdf',
                    'tetra_gazebo.xacro'
                ])
            ])
        }]
    )

    # Note: joint_state_publisher is NOT needed for Gazebo simulation
    # Gazebo's JointStatePublisher plugin already publishes joint states
    # and ros_gz_bridge forwards them to ROS2

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', LaunchConfiguration('world_file')],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # ROS-Gazebo bridge for common topics
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # cmd_vel: Bidirectional (use ] for bidirectional bridge)
            '/model/tetra/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # All others: Gazebo -> ROS (use @ for unidirectional Gazebo to ROS)
            '/model/tetra/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/scan2@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/depth/image_rect_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'
        ],
        remappings=[
            ('/model/tetra/odometry', '/odom'),
            ('/model/tetra/cmd_vel', '/cmd_vel')
        ],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # TF bridge for coordinate frames
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
        ],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Fake IMU publisher (사용 시 활성화)
    fake_imu_publisher = Node(
        package='tetra_gazebo_sim',
        executable='fake_imu_publisher.py',
        name='fake_imu_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=LaunchConfigurationNotEquals('use_fake_imu', 'false')
    )

    # Static transform for LiDAR frame (Gazebo uses nested frame names)
    # This creates an identity transform so RViz can find the frame
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['--frame-id', 'laser', '--child-frame-id', 'tetra/base_footprint/lidar'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    static_tf_laser2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser2',
        arguments=['--frame-id', 'laser_link2', '--child-frame-id', 'tetra/base_footprint/lidar2'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        # Set environment variables first
        gz_resource_path,
        
        # Launch arguments
        use_sim_time_arg,
        world_file_arg,
        robot_name_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        DeclareLaunchArgument(
            'use_fake_imu',
            default_value='true',
            description='Use fake IMU publisher if Gazebo IMU does not work'
        ),
        
        # Nodes and processes
        gazebo_launch,
        robot_state_publisher,
        # joint_state_publisher removed - Gazebo plugin handles this
        spawn_robot,
        ros_gz_bridge,
        tf_bridge,
        fake_imu_publisher,
        static_tf_laser,
        static_tf_laser2
    ])