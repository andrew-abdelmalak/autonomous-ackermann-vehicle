# Contributing

Team 23 — MCTR 1002 Autonomous Systems
GUC Mechatronics Department

## Team Members

| Name | GitHub |
|------|--------|
| Andrew Abdelmalak | [@andrew-abdelmalak](https://github.com/andrew-abdelmalak) |
| Daniel Boules | — |
| David Girgis | — |
| Kirolous Kirolous | — |
| Samir Yacoub | — |
| Youssef Salama | — |

## Workflow

1. Clone the repo and build the ROS 2 package: `colcon build --packages-select Autonomous_Systems_Project_Team_23`
2. Create a feature branch for your changes.
3. Follow the existing code conventions in `src/Autonomous_Systems_Project_Team_23/`.
4. Test your changes in Gazebo Harmonic before submitting a PR.
5. Open a pull request against `main` with a clear description.

## Code Style

- Python nodes follow `rclpy` conventions with ROS 2 parameter declarations.
- Launch files use the ROS 2 Python launch API.
- Arduino code targets the vehicle hardware platform (ATmega328P).

## Issues

Report bugs or request features via [GitHub Issues](https://github.com/andrew-abdelmalak/autonomous-ackermann-vehicle/issues).
