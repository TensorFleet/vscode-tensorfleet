# TensorFleet Robotic JS Project

JavaScript/Node.js template for robot control over rosbridge using `roslib`. Includes obstacle avoidance and vision examples.

## Quick start
1) Install deps: `bun install` (or `npm install`)
2) Open `Simulation view` and `Image View`.
3) Start your VM, then run:
   - `bun robot:mover` - Drive forward, backward, turn left/right sequence
   - `bun robot:obstacle` - LiDAR-based obstacle avoidance
   - `bun robot:vision` - YOLO object detection on camera feed
   - `bun robot:vision:colors` - Color-based detection (best for simulation)

## Scripts

### Movement Scripts
- `src/robot_mover.js` - Timed movement sequence (forward, stop, backward, stop, turn left/right).
- `src/obstacle_avoider.js` - LiDAR-based obstacle avoidance state machine. Moves forward by default, escapes when obstacles detected.

### Vision Scripts

**Setup:**
1. Open the **Image Panel** from the sidebar
2. Open the **Teleop Panel** to manually drive the robot around
3. In the Image Panel dropdown, select `/camera/image_raw` to see the robot's camera feed

**Running vision detection:**
1. Run one of the vision scripts:
   - `bun robot:vision` - YOLO detection (80 real-world object classes like people, cars, animals)
   - `bun robot:vision:colors` - Color-based detection (best for simulation with solid-colored shapes)
2. Switch the Image Panel dropdown to `/camera/image_annotated` to see the detection output with bounding boxes and labels
3. Use the Teleop Panel to drive the robot and see detections update in real-time

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
|   |-- vision_yolo.js      # YOLO detection pipeline (real-world objects)
|   `-- vision_colors.js    # Color-based detection (simulation shapes)
|-- config/                 # Network + robot config
|-- launch/                 # Optional ROS 2 launch files
|-- package.json
`-- README.md
```

## Tips
- rosbridge runs in the VM; no local ROS 2 binaries needed
- Connection auto-selects: uses VM Manager proxy if TensorFleet credentials available, otherwise direct rosbridge
- Adjust speeds and thresholds via environment variables for different robots/maps
