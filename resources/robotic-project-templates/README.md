# TensorFleet Robotic Project

Welcome to your TensorFleet robotic workspace! 🤖

This template is focused on ground robots and simple velocity control. It is designed to let you:

- Write Python code locally (no local ROS 2/PX4 install needed for most flows)
- Connect to a VM that runs ROS 2 + Gazebo + bridges (via proxy or direct)
- See results immediately in the TensorFleet VS Code panels (Image, Teleop, Raw Messages)

## Project Structure

```
.
├── src/
│   ├── lib/                    # Shared utilities for remote VM connectivity
│   │   ├── proxy_ws_client.py  # WebSocket proxy client for VM Manager
│   │   ├── tensorfleet_config.py  # Configuration management
│   │   ├── robotic_utils.py    # connect_to_robot() and helpers
│   │   └── url_utils.py        # URL conversion utilities
│   ├── robot_mover.py          # Example ROS 2 velocity node
│   ├── obstacle_avoider.py     # LiDAR-based obstacle avoidance
│   └── vision_yolo.py          # YOLO-based vision node via rosbridge
├── config/
│   └── robot_config.yaml       # Robot & network configuration
├── launch/                     # (Optional) ROS 2 launch files
├── .env.example                # Environment variable template
├── .tensorfleet                # TensorFleet workspace marker
├── requirements.txt            # Python dependencies
└── README.md                   # This guide
```

## Quick Start

1. **Start your VM**: Click the **TensorFleet** status bar at the bottom of VS Code and select **Start VM**.
2. Install dependencies with `uv` .

```bash
# If uv isn't installed: curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv
uv pip install -r requirements.txt
```

5. Run any of the example scripts:

```bash
# Basic movement demo
uv run python src/robot_mover.py

# Obstacle avoidance (requires LiDAR)
uv run python src/obstacle_avoider.py

# YOLO vision (requires camera)
uv run python src/vision_yolo.py
```

## Remote VM Connection

The scripts automatically connect through the TensorFleet VM Manager proxy when:
- `TENSORFLEET_BASE_URL` and `TENSORFLEET_JWT` are set in `.env`
- A VM is connected in the VS Code extension

The extension manages your `.env` file automatically, updating connection parameters when:
- You log in to TensorFleet
- You connect/disconnect from a VM
- You switch regions

### Direct Connection (Fallback)

If proxy settings are not available, scripts fall back to direct connection using:
- `ROS_HOST` / `ROS_PORT` environment variables, or
- Values from `config/robot_config.yaml`

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `TENSORFLEET_BASE_URL` | VM Manager base URL (managed by extension) | - |
| `TENSORFLEET_JWT` | Auth token (managed by extension) | - |
| `ROS_HOST` | Direct rosbridge host | `172.16.0.10` |
| `ROS_PORT` | Direct rosbridge port | `9091` |
| `CMD_VEL_TOPIC` | Velocity command topic | `/cmd_vel_raw` |
| `SCAN_TOPIC` | LiDAR scan topic | `/scan` |
| `IMAGE_TOPIC` | Camera image topic | `/camera/image_raw` |
| `LINEAR_SPEED` | Forward speed (m/s) | `0.2` |
| `ANGULAR_SPEED` | Turning speed (rad/s) | `0.5` |

## Example: Basic Motion (`robot_mover.py`)

This script:
- Connects to rosbridge (via proxy or direct)
- Publishes `geometry_msgs/Twist` on `/cmd_vel_raw`
- Executes a short sequence: forward, backward, turn left, turn right, stop

```bash
uv run python src/robot_mover.py
```

## Example: YOLO Vision (`vision_yolo.py`)

This script:
- Connects to rosbridge via `roslibpy` (Python-only client)
- Subscribes to `sensor_msgs/Image` on `/camera/image_raw`
- Runs YOLO object detection on CPU using `ultralytics` and OpenCV
- Publishes annotated images on `/camera/image_annotated`

**Setup:**
1. Open **Simulation View** from the sidebar to see the robot in Gazebo
2. Open **Image Panel** from the sidebar
3. Open **Teleops Panel** to manually drive the robot around
4. In the Image Panel dropdown, select `/camera/image_raw` to see the robot's camera feed

**Running vision detection:**
1. Run the vision script:
   ```bash
   uv run python src/vision_yolo.py
   ```
2. Switch the Image Panel dropdown to `/camera/image_annotated` to see the detection output with bounding boxes and labels
3. Use the Teleops Panel to drive the robot and see detections update in real-time

YOLO runs on **CPU only**; no GPU is required.

## Example: Obstacle Avoidance (`obstacle_avoider.py`)

This script combines motion and LiDAR data for autonomous navigation:
- Moves forward by default
- Turns or backs up when obstacles are detected
- Adaptive speed based on clearance

```bash
uv run python src/obstacle_avoider.py
```

## VM / Infrastructure Assumptions

Your TensorFleet VM should be running:

- ROS 2 (Humble or newer)
- Gazebo and controllers for your robot
- rosbridge on port 9091
- Foxglove bridge (for the TensorFleet Image/Teleop/Raw panels)

The VM is responsible for:
- Simulating the robot and publishing sensor topics
- Hosting all ROS 2 binaries

Your local machine just runs Python, talks to rosbridge, and uses the VS Code extension.

## Customization

- Adjust topics and model in `vision_yolo.py` or via environment variables
- Replace the YOLO model with your own detector
- Extend `robot_mover.py` into keyboard teleop, state machines, or mission logic
- Add new Python nodes in `src/` that publish/subscribe via rosbridge

Happy hacking on robots! ✨
