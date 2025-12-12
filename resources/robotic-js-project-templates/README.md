# TensorFleet Robotic JS Project

JavaScript/Node.js template for robot control over rosbridge using `roslib`. Includes obstacle avoidance and YOLO vision examples.

## Quick start
1) Install deps: `bun install` (or `npm install`)
2) Open `Simulation view` and `Image View`.
3) Start your VM, then run:
   - `bun robot:mover` - Drive forward, backward, turn left/right sequence
   - `bun robot:obstacle` - LiDAR-based obstacle avoidance
   - `bun robot:vision` - YOLO object detection on camera feed

## Scripts
- `src/robot_mover.js`: Timed movement sequence (forward, stop, backward, stop, turn left/right).
- `src/obstacle_avoider.js`: LiDAR-based obstacle avoidance state machine. Moves forward by default, escapes when obstacles detected.
- `src/vision_yolo.js`: Subscribe to camera, run YOLO detection, republish annotated images.

## Configuration
Edit `config/robot_config.yaml` or override via env vars:
- `TENSORFLEET_BASE_URL` + `TENSORFLEET_JWT` - primary TensorFleet connection (other URLs are derived automatically when blank)
- `ROSBRIDGE_URL` - rosbridge WebSocket URL
- `ROS_HOST` / `ROS_PORT` - rosbridge host and port (fallback if `ROSBRIDGE_URL` is not set)
- `CMD_VEL_TOPIC` - velocity command topic (default `/cmd_vel_raw`)
- `SCAN_TOPIC` - LiDAR scan topic for obstacle avoidance (default `/scan`)
- `LINEAR_SPEED`, `ANGULAR_SPEED` - motion tuning
- `OBSTACLE_DISTANCE`, `CLEAR_DISTANCE` - avoidance thresholds
- `IMAGE_TOPIC`, `ANNOTATED_IMAGE_TOPIC` - camera topics

## Layout
```
.
|-- src/
|   |-- lib/                # Shared utilities (proxy, config, connection)
|   |-- robot_mover.js      # Basic movement example
|   |-- obstacle_avoider.js # LiDAR avoidance state machine
|   `-- vision_yolo.js      # YOLO detection pipeline
|-- config/                 # Network + robot config
|-- launch/                 # Optional ROS 2 launch files
|-- package.json
`-- README.md
```

## Tips
- rosbridge runs in the VM; no local ROS 2 binaries needed
- Connection auto-selects: uses VM Manager proxy if TensorFleet credentials available, otherwise direct rosbridge
- Adjust speeds and thresholds via environment variables for different robots/maps
