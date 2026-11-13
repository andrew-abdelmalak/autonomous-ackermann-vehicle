# Contributing to autonomous-ackermann-vehicle

Bug reports, simulation or math fixes, firmware improvements, and documentation updates are welcome.

## Before you start

- Search [existing issues](https://github.com/andrew-abdelmalak/autonomous-ackermann-vehicle/issues) to avoid duplicates.
- For larger changes, open an issue first to align on the design before editing code.
- By participating, you agree to follow the [Code of Conduct](CODE_OF_CONDUCT.md).

## Set up your environment

Requirements: Python 3.12 (`ruff==0.16.10`), Docker (or a native ROS 2 Jazzy + Gazebo Harmonic installation), and `arduino-cli` (`arduino:avr@1.8.8`, `arduino:mbed_rp2040@4.6.0`, `Servo@1.3.0`).

```bash
git clone https://github.com/andrew-abdelmalak/autonomous-ackermann-vehicle.git
cd autonomous-ackermann-vehicle
```

## Run the checks

CI runs these exact checks on every push and pull request targeting `main`:

```bash
# Python lint and format checks (line length 99, single quotes)
ruff check .
ruff format --check .

# ROS 2 Jazzy build and unit tests
docker run --rm -v "$PWD:/ws" -w /ws \
  ros@sha256:00b8d47a6e6384a2834e3d01fa87ae767876543baa574f2d64a75c04ea33c698 \
  bash -lc "source /opt/ros/jazzy/setup.bash && \
    colcon --log-base /tmp/log build --build-base /tmp/build --install-base /tmp/install --packages-select Autonomous_Systems_Project_Team_23 && \
    colcon --log-base /tmp/log test --build-base /tmp/build --install-base /tmp/install --packages-select Autonomous_Systems_Project_Team_23 && \
    colcon --log-base /tmp/log test-result --test-result-base /tmp/build --verbose"

# Embedded firmware compilation checks
arduino-cli compile --fqbn arduino:avr:uno arduino/vehicle_controller
arduino-cli compile --fqbn arduino:mbed_rp2040:pico arduino/pico_ms4_all_tracks
```

## Make your change

1. Create a branch from `main`: `git switch -c fix/short-description`.
2. Keep each pull request scoped to one problem.
3. Add or update unit tests in `src/Autonomous_Systems_Project_Team_23/test/` for any math, planner, controller, or localization change.
4. Keep `paper/` and `docs/autonomous_ackermann_vehicle_paper.pdf` in sync if algorithm parameters or equations change.
5. Follow [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) (`feat:`, `fix:`, `docs:`, `test:`, `ci:`).

## Team members

MCTR 1002 Autonomous Systems, Team 23 (German University in Cairo):

| Name | GitHub |
|---|---|
| Andrew Abdelmalak | [@andrew-abdelmalak](https://github.com/andrew-abdelmalak) |
| Daniel Boules | [@daniel-boules](https://github.com/daniel-boules) |
| David Girgis | [@davidlouiss](https://github.com/davidlouiss) |
| Kirolous Kirolous | [@Kiro579](https://github.com/Kiro579) |
| Samir Yacoub | [@SamirYacoub](https://github.com/SamirYacoub) |
| Youssef Salama | Not listed |

## Report a bug or request a feature

Use the [issue forms](https://github.com/andrew-abdelmalak/autonomous-ackermann-vehicle/issues/new/choose). Report security issues privately via [SECURITY.md](SECURITY.md).

## License

By contributing, you agree that your contributions are licensed under the project's [MIT License](LICENSE).
