# TensorFleet Robotic Project Templates

This directory hosts two TensorFleet robotic starter projects: a lightweight ground robot template and the SO101 arm teleoperation demo. Each template has its own quickstart guide so you can jump straight into the flow that matches your hardware or simulation.

## Projects

- **Simple Robot** – a ground robot focused on velocity control, LiDAR avoidance, and YOLO vision. See `simple-robot_quickstart.md` for the full walkthrough, dependencies, and example scripts.
- **SO101 Arm** – keyboard teleop for the SO101 arm with dedicated camera feeds. Follow `so101-arm_quickstart.md` for the VM setup, telemetry panels, and key bindings.

## Shared resources

- `requirements.txt` / `requirements.arm.txt` – install the Python dependencies for each template (`requirements.arm.txt` is the SO101 arm bundle).
- `config/robot_config.yaml` – robot and network defaults (used by both templates).
- `launch/` – optional ROS 2 launch files that either template can consume.
- `src/` – contains sample Python nodes (`robot_mover`, `obstacle_avoider`, `vision_yolo`) plus the SO101 keyboard teleop in `teleop_so_arm101.py`.

Pick the quickstart that matches your robot, then follow the linked guide for step-by-step instructions.
