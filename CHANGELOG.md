# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Deterministic `pytest` unit and paper-number reproduction suite covering angle wrapping, quintic blend, Ackermann kinematics, SDF obstacle parsing and racing clearance planning, city loop projection, PID speed and Stanley lateral control laws, and 5-state Kalman filter RMSE metrics.
- SHA-pinned GitHub Actions CI running Ruff lint/formatting, ROS 2 Jazzy `colcon build` and `colcon test`, and `arduino-cli` builds for Arduino Uno and Raspberry Pi Pico sketches.
- Community health files, issue forms, pull request template, release categorization config, Dependabot configuration, and citation metadata.

### Changed

- Consolidated ROS 2 Python nodes under `src/Autonomous_Systems_Project_Team_23/` and embedded firmware under `arduino/vehicle_controller/` and `arduino/pico_ms4_all_tracks/`.
- Synchronized IEEE paper equations, Kalman noise matrices, controller gain descriptions, firmware specs, and verified bibliography entries with the repository source code.
- Removed duplicate `desired_speed` key in the MS4 launch dictionary and made `matplotlib` import lazy inside `save_artifacts_to_disk()` so headless ROS 2 environments can import and test all nodes cleanly.

## [1.0.0] - 2026-06-29

### Added

- Integrated ROS 2 Jazzy and Gazebo Harmonic Ackermann vehicle stack covering open-loop teleoperation (MS2), longitudinal PID and lateral Stanley controllers (MS3), three-track geometry-aware planner and collision monitor (MS4), and 5-state Kalman localization with CSV/PNG evaluation export (MS5).
- Arduino Uno (`vehicle_controller.ino`) and Raspberry Pi Pico RP2040 (`pico_ms4_all_tracks.ino`) serial actuator and IMU/encoder firmware.
- Compiled IEEE paper (`docs/autonomous_ackermann_vehicle_paper.pdf`), LaTeX source (`paper/main.tex`), and course milestone PDFs (`docs/`).

[Unreleased]: https://github.com/andrew-abdelmalak/autonomous-ackermann-vehicle/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/andrew-abdelmalak/autonomous-ackermann-vehicle/releases/tag/v1.0.0
