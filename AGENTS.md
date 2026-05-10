# Autonomous Ackermann Vehicle (Team 23)

Purpose
- ROS 2 Jazzy + Gazebo Harmonic milestones for MCTR1002 (Team 23 Ackermann vehicle)

Stack
- ROS 2 Jazzy (rclpy, launch)
- Gazebo Harmonic + ros_gz_bridge / ros_gz_sim
- Arduino (L298N + encoder + servo)

Key folders
- `Autonomous_Systems_Project_Team_23/` ROS 2 ament_python package
- `Autonomous_Systems_Project_Team_23/Autonomous_Systems_Project_Team_23/` Python nodes
- `Autonomous_Systems_Project_Team_23/launch/` ROS 2 launch files
- `Autonomous_Systems_Project_Team_23/models/` Gazebo model
- `hardware/` Arduino controller sketch
- `assets/figures/` report figures

Build / run (typical)
- Build: `colcon build --packages-select Autonomous_Systems_Project_Team_23`
- Source: `source /opt/ros/jazzy/setup.bash` then `source install/setup.bash`
- MS2 launch: `ros2 launch Autonomous_Systems_Project_Team_23 Autonomous_Systems_MS_2_Team_23.launch.py`
- MS3 launch: `ros2 launch Autonomous_Systems_Project_Team_23 Autonomous_Systems_MS_3_Team_23.launch.py`

Notes
- Teleop supports optional serial forwarding to Arduino (`/dev/ttyUSB0` typical).
- Joy teleop reads `/dev/input/js0` directly (no `joy` package required).
- Arduino sketch in `hardware/` is the tuned MS3 controller.
