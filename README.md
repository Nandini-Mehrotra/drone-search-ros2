# Autonomous UAV Search and Geofence Fail-Safe Controller

**ROS 2 Jazzy · PX4 SITL · Gazebo X500 · Python · Micro XRCE-DDS**

## Objective

Develop and validate a simulation-based autonomous mission controller that commands takeoff, generates a Figure-8 search path, monitors a configurable geofence, and requests landing when mission completion or a boundary breach occurs.

## Current verification status

- **Verified:** ROS 2–PX4 message transport and live local-position telemetry; controller dry-run received telemetry in a previous session.
- **Implemented but not flight-verified:** mission state machine, Figure-8 setpoints, geofence check, landing requests.
- **Pending:** preflight/GCS resolution, autonomous flight, failsafe landing test, and flight log evidence.

Do not interpret passing unit tests as proof of safe autonomous flight.

## Architecture

See [docs/architecture.md](docs/architecture.md) for the data flow, topic names, and PX4 NED coordinate system.

## Repository layout

- `src/search_pattern_offboard.py` — existing ROS 2 flight controller (preserve your original file).
- `src/search_math.py` — pure Figure-8 and geofence mathematics, independently testable.
- `tests/test_search_math.py` — standard-library unit tests.
- `config/mission.yaml` — reference mission settings; **not yet wired to the controller**.
- `docs/technical_explanation.md` — engineering and interview explanation.
- `docs/testing.md` — test plan and evidence checklist.

## Environment used

Ubuntu 24.04 (WSL), ROS 2 Jazzy, PX4 SITL, Gazebo X500, and Micro XRCE-DDS Agent. PX4 message definitions must match the PX4 checkout used to build the simulator.

## Start the simulation

Terminal 1:

```bash
MicroXRCEAgent udp4 -p 8888
```

Terminal 2:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_px4_ws/install/setup.bash
cd ~/projects/px4-autonomous-search
ros2 topic list | grep /fmu
python3 src/search_pattern_offboard.py
```

The controller starts in dry-run mode. Do not enable `--fly` until the PX4 preflight and controller safety checks are resolved.

## Automated tests

```bash
python3 -m unittest discover -s tests -v
```

These tests cover trajectory math and geofence boundaries; they are **not** end-to-end flight tests.

## Safety and limitations

Simulation only. Current flight-controller code requires additional validation for command acknowledgments, stale telemetry, state timeouts, and failsafe behavior. Do not use on physical aircraft without appropriate engineering review, hardware validation, and safety procedures.

## Project explanation

See [docs/technical_explanation.md](docs/technical_explanation.md) for the engineering rationale, coordinate conventions, and interview Q&A.
