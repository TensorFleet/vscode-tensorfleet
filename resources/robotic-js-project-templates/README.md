# TensorFleet Robotic JS Project

This template is the JavaScript/Node.js sibling of the Python-based `robotic-project-templates`. It targets the same ROS graph and topics, but uses `roslib` over rosbridge instead of `roslibpy`.

The examples assume:

- A TensorFleet VM running ROS 2, Gazebo, PX4, rosbridge, and the Foxglove bridge.
- rosbridge is reachable at `ws://172.16.0.10:9091` (or the URL configured in `config/robot_config.yaml`).

## Project Structure

```
.
├── src/
│   ├── robot_mover.js        # Timed movement sequence over /cmd_vel_raw
│   ├── obstacle_avoider.js   # LiDAR-based obstacle avoidance state machine
│   └── vision_yolo.js        # Image subscribe/republish helper over rosbridge
├── config/
│   └── robot_config.yaml     # Robot & network configuration (VM IP, rosbridge URL, topics)
├── launch/                   # (Optional) ROS 2 launch files
├── package.json              # Node.js project definition (roslib dependency)
└── README.md                 # This guide
```

## Setup

From the project root (the directory containing this README and `package.json`):

```bash
bun install
```

You do not need ROS 2 installed locally; the scripts talk to rosbridge running in your VM.

## Configuration

The default network and topic configuration lives in `config/robot_config.yaml`:

```yaml
network:
  vm_ip: "172.16.0.10"
  rosbridge_url: "ws://172.16.0.10:9091"
  foxglove_bridge_url: "ws://172.16.0.10:8765"

motion:
  cmd_vel_topic: "/cmd_vel_raw"
```

Each script reads from this YAML file for its defaults and also accepts environment variables so you can override settings without editing code:

- `ROSBRIDGE_URL` – full rosbridge WebSocket URL, e.g. `ws://172.16.0.10:9091`
- `ROS_HOST` / `ROS_PORT` – rosbridge host and port (fallback if `ROSBRIDGE_URL` is not set)
- `CMD_VEL_TOPIC` – velocity command topic (default from `motion.cmd_vel_topic`)
- `SCAN_TOPIC` – LiDAR scan topic for obstacle avoidance (default `/scan`)
- `LINEAR_SPEED`, `ANGULAR_SPEED` – motion tuning
- `OBSTACLE_DISTANCE`, `CLEAR_DISTANCE` – avoidance thresholds
- `IMAGE_TOPIC`, `ANNOTATED_IMAGE_TOPIC` – image passthrough topics

## Example: Basic Motion (`robot_mover.js`)

Run a simple movement sequence (forward, stop, backward, stop, turn left/right, stop):

```bash
bun src/robot_mover.js
```

This script:

- Connects to rosbridge via `roslib`
- Publishes `geometry_msgs/Twist` messages on `/cmd_vel_raw`
- Uses a promise-based helper to publish at 20 Hz for a given duration

You can adjust speed and topics via environment variables, for example:

```bash
ROS_HOST=172.16.0.10 ROS_PORT=9091 CMD_VEL_TOPIC=/cmd_vel_raw LINEAR_SPEED=0.3 node src/robot_mover.js
```

## Example: Obstacle Avoidance (`obstacle_avoider.js`)

Run a LiDAR-based obstacle avoidance state machine:

```bash
bun src/obstacle_avoider.js
```

Behavior:

- Moves forward by default
- Monitors a `sensor_msgs/LaserScan` on `/scan`
- When an obstacle is closer than `OBSTACLE_DISTANCE` in front, chooses the best escape direction (left, right, or back)
- Resumes forward motion once the path ahead is clear beyond `CLEAR_DISTANCE`

Logs are printed to the terminal to mirror the Python sample, and velocity commands are sent to the same `/cmd_vel_raw` topic.

## Example: Image Passthrough (`vision_yolo.js`)

This script demonstrates a lightweight image pipeline suitable for pairing with a Python YOLO node:

```bash
bun src/vision_yolo.js
```

By default it:

- Subscribes to `sensor_msgs/Image` on `/camera/image_raw`
- Republishes the same messages on `/camera/image_annotated`

Use this pattern as a starting point for building web-based or Node-based visualization tools that sit alongside a heavier Python perception stack (for example, YOLO in `vision_yolo.py` from the Python template).

## Notes on Python → JS Port

- The ROS design and message types are unchanged: `/cmd_vel_raw` (`geometry_msgs/Twist`), `/scan` (`sensor_msgs/LaserScan`), and camera topics (`sensor_msgs/Image`).
- `rclpy` and `roslibpy` are replaced by `roslib` over a WebSocket connection to rosbridge.
- Blocking loops with `time.sleep` are replaced with `setInterval` and `Promise`-based helpers.
- Python classes and methods are ported to JavaScript classes using the same state-machine logic and topic names.
