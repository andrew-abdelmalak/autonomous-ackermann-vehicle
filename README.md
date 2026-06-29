# Autonomous Ackermann Vehicle — Team 23

**MCTR 1002 — Autonomous Systems**
Mechatronics Department, German University in Cairo (GUC)

| Name | Student ID | GitHub |
|------|------------|--------|
| Andrew Abdelmalak | 55-22771 | [@andrew-abdelmalak](https://github.com/andrew-abdelmalak) |
| Daniel Boules | 55-5055 | — |
| David Girgis | 55-1481 | — |
| Kirolous Kirolous | 55-18081 | — |
| Samir Yacoub | 55-25111 | — |
| Youssef Salama | 55-0540 | — |

---

ROS 2 Jazzy and Gazebo Harmonic control stack for a 1:10-scale Ackermann vehicle.
Covers Milestones 1–5: platform validation, open-loop response, closed-loop
speed/lateral control, path planning with collision avoidance, and Kalman-filtered
state estimation deployed on Raspberry Pi 4 with an Arduino actuator controller.

![MS2 Simulation OLR Response](results/m2_simulation_olr_response.png)

---

## Repository Structure

```text
.
├── src/
│   └── Autonomous_Systems_Project_Team_23/   # ament_python package
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       ├── launch/                            # MS2 & MS3 launch files
│       ├── models/prius_team23/               # Gazebo SDF model + meshes
│       ├── test/                              # Lint + copyright tests
│       └── Autonomous_Systems_Project_Team_23/  # Python nodes
│           ├── Validation_Printing_Node_Team_23.py
│           ├── Vehicle_Pub_Sub_Node_Team_23.py
│           ├── Autonomous_Systems_MS_2_OLR_Team_23.py
│           ├── Autonomous_Systems_MS_2_Teleop_Team_23.py
│           ├── Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_23.py
│           ├── Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_23.py
│           └── Autonomous_Systems_MS_3_Joy_Teleop_Team_23.py
├── arduino/
│   └── vehicle_controller.ino                # Tuned MS3 actuator controller
├── paper/
│   ├── main.tex                               # IEEE conference paper
│   ├── references.bib
│   ├── IEEEtran.cls
│   └── figures/
│       ├── m1_hardware_ros2_doctor.png
│       ├── m1_hardware_validation_node.png
│       ├── m1_simulation_validation_node.png
│       ├── m2_hardware_actuator_test.png
│       ├── m2_simulation_olr_response.png
│       └── m2_simulation_teleop_drive.png
├── results/
│   ├── m1_hardware_ros2_doctor.png
│   ├── m1_hardware_validation_node.png
│   ├── m1_simulation_validation_node.png
│   ├── m2_hardware_actuator_test.png
│   ├── m2_simulation_olr_response.png
│   └── m2_simulation_teleop_drive.png
├── docs/
│   ├── MS1_Literature_Review.pdf
│   └── MS5_Final_Report.pdf
├── README.md
├── LICENSE
├── .gitignore
├── requirements.txt
├── CONTRIBUTING.md
└── CHANGELOG.md
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Raspberry Pi 4 (Ubuntu 24.04)             │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────────┐  │
│  │ Speed Ctrl   │  │ Lateral Ctrl │  │ Joystick Teleop    │  │
│  │  (P + FF)    │  │   (PID)      │  │  (/dev/input/js0)  │  │
│  └──────┬───────┘  └──────┬───────┘  └────────┬──────────┘  │
│         │                 │                    │             │
│         └─────────┬───────┘                    │             │
│                   ▼                            │             │
│         ┌──────────────────┐                   │             │
│         │  Serial Bridge   │◄──────────────────┘             │
│         │  (/dev/ttyUSB0)  │                                  │
│         └────────┬─────────┘                                  │
└──────────────────┼──────────────────────────────────────────┘
                   │ SPD:<speed>,STR:<steering>
                   ▼
┌──────────────────────────────────────────────────────────────┐
│              Arduino (ATmega328P)                             │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────────┐  │
│  │ Speed PID    │  │ Steering Map │  │ Encoder ISR       │  │
│  │  (50 Hz)     │  │  (servo us)  │  │  (tick counter)   │  │
│  └──────┬───────┘  └──────┬───────┘  └────────┬──────────┘  │
│         │                 │                    │             │
│         ▼                 ▼                    │             │
│  ┌──────────┐     ┌──────────┐         ┌──────┴──────┐      │
│  │ L298N    │     │ Servo    │         │ Encoder     │      │
│  │ Motor    │     │ Steering │         │ (wheel RPM) │      │
│  └──────────┘     └──────────┘         └─────────────┘      │
└──────────────────────────────────────────────────────────────┘
```

---

## Hardware Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Wheelbase | 0.26 | m |
| Track width | 0.18 | m |
| Wheel radius | 0.033 | m |
| Mass | 2.1 | kg |
| Max steering angle | ±0.52 | rad |
| Max speed | 0.5 | m/s |
| Encoder PPR | 20 | pulses/rev |
| Motor gear ratio | 48:1 | — |
| Control loop rate | 50 | Hz |
| Serial baud rate | 115200 | baud |

---

## Controller Gains

| Controller | Parameter | Value |
|-----------|-----------|-------|
| Speed (P) | $K_p$ | 1.2 |
| Speed (feedforward) | $K_{ff}$ | 0.85 |
| Lateral (P) | $K_{p,lat}$ | 0.4 |
| Lateral (I) | $K_{i,lat}$ | 0.02 |
| Lateral (D) | $K_{d,lat}$ | 0.1 |
| Kalman filter | Process noise $Q$ | $\mathrm{diag}(0.1, 0.1, 0.01)$ |
| Kalman filter | Measurement noise $R$ | $\mathrm{diag}(0.5, 0.5)$ |

---

## Key Equations

### Bicycle Model (Ackermann Kinematics)

$$
\begin{aligned}
\dot{x} &= v \cos(\theta) \\
\dot{y} &= v \sin(\theta) \\
\dot{\theta} &= \frac{v}{L} \tan(\delta)
\end{aligned}
$$

where $v$ is longitudinal velocity, $\delta$ is steering angle, and $L$ is wheelbase.

### Speed Controller (P + Feedforward)

$$
u = K_{ff} \cdot v_{des} + K_p \cdot (v_{des} - v_{meas})
$$

### Kalman Filter Prediction Step

$$
\begin{aligned}
\hat{x}_{k|k-1} &= A \hat{x}_{k-1|k-1} + B u_k \\
P_{k|k-1} &= A P_{k-1|k-1} A^T + Q
\end{aligned}
$$

---

## Prerequisites

- Ubuntu Noble 24.04
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic
- `ros-jazzy-ros-gz`, `ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge`
- Arduino IDE (for hardware sketch)

## Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/andrew-abdelmalak/autonomous-ackermann-vehicle.git
mv autonomous-ackermann-vehicle/src/Autonomous_Systems_Project_Team_23 .

source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select Autonomous_Systems_Project_Team_23
source install/setup.bash
```

## Usage

### Milestone 1 — Validation

```bash
ros2 run Autonomous_Systems_Project_Team_23 validation_node
ros2 run Autonomous_Systems_Project_Team_23 pub_sub_node
```

### Milestone 2 — OLR & Teleop

```bash
# Open-loop response
ros2 launch Autonomous_Systems_Project_Team_23 Autonomous_Systems_MS_2_Team_23.launch.py \
  control_mode:=olr desired_speed:=0.25 desired_steering:=0.30 lane:=0.0

# Keyboard teleop
ros2 launch Autonomous_Systems_Project_Team_23 Autonomous_Systems_MS_2_Team_23.launch.py \
  control_mode:=teleop
```

| Key | Action |
|-----|--------|
| ↑ | Increase speed |
| ↓ | Decrease speed |
| ← | Steer left |
| → | Steer right |
| Space | Stop |
| Q | Quit |

### Milestone 3 — Closed-Loop Control

```bash
ros2 launch Autonomous_Systems_Project_Team_23 Autonomous_Systems_MS_3_Team_23.launch.py \
  desired_speed:=0.5 desired_lane:=0.0 desired_heading:=0.0
```

### Joystick Teleop (Hardware)

```bash
ros2 run Autonomous_Systems_Project_Team_23 joy_teleop_team_23 \
  --ros-args -p serial_port:=/dev/ttyUSB0
```

### Arduino

Upload `arduino/vehicle_controller.ino` to the ATmega328P board. The sketch
parses serial commands (`SPD:<speed>,STR:<steering>`) and runs a 50 Hz PID
speed loop with encoder feedback.

---

## ROS 2 Interfaces

| Topic | Message Type | Direction |
|-------|--------------|-----------|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS |
| `/model/vehicle/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo |
| `/model/vehicle/odometry` | `nav_msgs/msg/Odometry` | Gazebo → ROS |
| `joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS |

---

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `control_mode` | `teleop` | `teleop` or `olr` |
| `desired_speed` | `0.25` | Target speed (m/s) |
| `desired_steering` | `0.0` | Target steering angle (rad) |
| `lane` | `0.0` | Vehicle spawn Y offset |
| `use_rqt_graph` | `true` | Launch rqt_graph |
| `serial_forwarding_enabled` | `false` | Forward commands to Arduino |
| `serial_port` | `/dev/ttyUSB0` | Arduino serial port |
| `serial_baudrate` | `115200` | Arduino baud rate |

---

## References

1. R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed. Springer, 2012.
2. S. Thrun, W. Burgard, and D. Fox, *Probabilistic Robotics*. MIT Press, 2005.
3. R. E. Kalman, "A new approach to linear filtering and prediction problems," *J. Basic Eng.*, vol. 82, no. 1, pp. 35–45, 1960.
4. J. Kong, M. Pfeiffer, G. Schildbach, and F. Borrelli, "Kinematic and dynamic vehicle models for autonomous driving control design," in *Proc. IEEE Intell. Veh. Symp.*, 2015, pp. 1094–1099.
5. B. Paden, M. Čáp, S. Z. Yong, D. Yershov, and E. Frazzoli, "A survey of motion planning and control techniques for self-driving urban vehicles," *IEEE Trans. Intell. Veh.*, vol. 1, no. 1, pp. 33–55, 2016.
6. S. Macenski, T. Foote, B. Gerkey, C. Lalancette, and W. Woodall, "Robot Operating System 2: Design, architecture, and uses in the wild," *Sci. Robot.*, vol. 7, no. 66, 2022.
7. N. Koenig and A. Howard, "Design and use paradigms for Gazebo, an open-source multi-robot simulator," in *Proc. IEEE/RSJ IROS*, 2004, pp. 2149–2154.
8. Open Robotics, "ROS 2 Documentation: Jazzy Jalisco," 2024. [Online]. Available: https://docs.ros.org/en/jazzy/
9. Open Robotics, "Gazebo Harmonic Documentation," 2024. [Online]. Available: https://gazebosim.org/docs/harmonic
10. R. Siegwart, I. R. Nourbakhsh, and D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd ed. MIT Press, 2011.

---

## License

MIT — see [LICENSE](LICENSE). Copyright (c) 2026 Team 23.
