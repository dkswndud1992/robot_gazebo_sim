# Copilot Instructions for Robot Gazebo Simulation

## Overview
This repository provides a complete Gazebo Harmonic simulation environment for the mobile robot using ROS2 Jazzy. It integrates robot models, launch files, world definitions, and parameter configurations for rapid simulation, testing, and development.

## Key Architecture & Components
- **Launch Files (`launch/`)**: Python-based ROS2 launch files orchestrate simulation, navigation, and full-system bringup. Key files:
  - `gazebo_sim.launch.py`: Basic simulation (robot + world)
  - `sim_bringup.launch.py`: Adds EKF, joystick, web interface
  - `full_sim.launch.py`: Full stack (SLAM, Nav2, RViz)
- **Robot Models (`urdf/`)**: Xacro-based URDFs define robot structure, sensors, and plugins. Edit `robot_gazebo.xacro` for physical/sensor changes.
- **Worlds (`worlds/`)**: SDF files for simulation environments. Add or modify for custom scenarios.
- **Parameters (`params/`)**: YAML files for EKF, SLAM, navigation, etc.
- **Bridging**: Uses `ros_gz_bridge` to map Gazebo topics (e.g., `/model/robot/cmd_vel`) to ROS2 topics (e.g., `/cmd_vel`).

## Developer Workflows
- **Build**: Use `colcon build --packages-select robot_gazebo_sim` from your ROS2 workspace root.
- **Source**: Always source both ROS2 and workspace setup scripts before running (`source /opt/ros/jazzy/setup.bash && source install/setup.bash`).
- **Simulation**: Launch with `ros2 launch robot_gazebo_sim gazebo_sim.launch.py`. For full navigation, use `full_sim.launch.py`.
- **Testing**: Use provided shell scripts (e.g., `test_simulation.sh`) or publish to `/cmd_vel` to verify movement. Always ensure Gazebo is unpaused (press ▶️ in GUI).
- **Debugging**: Check topic bridges, odometry, and Gazebo logs. See README for troubleshooting common issues (e.g., robot not moving, mesh not found).

## Project-Specific Conventions
- **No `joint_state_publisher` in simulation**: Gazebo plugin handles joint states; do not add this node in launch files.
- **Topic Remapping**: Rely on `ros_gz_bridge` for all sensor/control topic mapping. Do not hardcode Gazebo topic names in ROS nodes.
- **Resource Paths**: Meshes are resolved via `GZ_SIM_RESOURCE_PATH`, set automatically by launch files. If mesh errors occur, rebuild and source workspace.
- **z_pose Guidance**: Default robot spawn height is 0.055m (wheel radius + margin). Adjust via launch args if robot floats or sinks.

## Integration & Extensibility
- **Add sensors**: Edit `urdf/robot_gazebo.xacro` and update launch/bridge as needed.
- **New worlds**: Place `.sdf` in `worlds/` and set `world_file` launch arg.
- **Navigation tuning**: Edit `params/navigation_sim.yaml`.

## External Dependencies
- **ROS2 Jazzy** and **Gazebo Harmonic** (Ubuntu 24.04)
- System packages: `ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge`, `ros-jazzy-navigation2`, etc. (see README)
- Workspace packages: `robot_description`, `robot_navigation2`, `robot_interface`

## Examples
- Launch simulation: `ros2 launch robot_gazebo_sim gazebo_sim.launch.py`
- Publish velocity: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once`
- Save SLAM map: `ros2 run nav2_map_server map_saver_cli -f ~/my_map`

## References
- See `README.md` for detailed troubleshooting, parameter explanations, and advanced usage.
- Key files: `launch/`, `urdf/`, `params/`, `worlds/`, `test_simulation.sh`

---
For unclear or missing conventions, consult the README or open an issue for clarification.
