# TensorFleet Robotic Project

Welcome to your TensorFleet robotic workspace! 🤖

This template is focused on ground robots and simple velocity control instead of drone flight. It is designed to let you:

- Write Python code locally (no local ROS 2/PX4 install needed for most flows)
- Connect to a VM that runs ROS 2 + Gazebo + PX4 + bridges
- See results immediately in the TensorFleet VS Code panels (Image, Teleop, Raw Messages)

## Project Structure

```
.
├── src/
│   ├── robot_mover.py      # Example ROS 2 velocity node (runs inside ROS 2 env)
│   └── vision_yolo.py      # YOLO-based vision node via rosbridge (Python-only client)
├── config/
│   └── robot_config.yaml   # Robot & network configuration (VM IP, rosbridge URL, topics)
├── launch/                 # (Optional) ROS 2 launch files
├── requirements.txt        # Python dependencies (cv2, ultralytics, roslibpy, etc.)
└── README.md               # This guide (Quick Start + YOLO vision)
```

## Quick Start (Python Environment)

1. Create a **Robotic Project** from the TensorFleet tooling view in VS Code.
2. In the new project folder, install dependencies with `uv` on your **local machine**:

```bash
# If uv isn't installed: curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv
uv pip install -r requirements.txt
```

This installs only Python packages (OpenCV, ultralytics, roslibpy, etc.). You do **not** need ROS 2 or PX4 installed locally.

## VM / Infrastructure Assumptions

Your TensorFleet VM should be running:

- ROS 2 (Humble or newer)
- Gazebo and PX4 as appropriate for your robot
- rosbridge on the URL configured in `config/robot_config.yaml` (default `ws://172.16.0.10:8080`)
- Foxglove bridge (for the TensorFleet Image/Teleop/Raw panels)

The VM is responsible for:

- Simulating the robot and publishing sensor topics (e.g. `/camera/image_raw`, `/cmd_vel_raw`)
- Hosting all ROS 2 / PX4 binaries

Your local machine just runs Python, talks to rosbridge, and uses the VS Code extension.

## Configure Networking

Open `config/robot_config.yaml` and confirm the network block matches your VM:

```yaml
network:
  vm_ip: "172.16.0.10"
  rosbridge_url: "ws://172.16.0.10:8080"
  foxglove_bridge_url: "ws://172.16.0.10:8765"

motion:
  cmd_vel_topic: "/cmd_vel_raw"
```

- `rosbridge_url` is how local Python scripts (like `vision_yolo.py`) connect into the VM.
- `cmd_vel_topic` is where velocity commands are expected.

## Example: Basic Motion (`robot_mover.py`)

If you are inside a ROS 2 Python environment (e.g. directly in the VM or a container with `rclpy` and `geometry_msgs` installed), you can run the simple movement node:

```bash
uv run python src/robot_mover.py
```

This will:

- Initialize a ROS 2 node
- Publish `geometry_msgs/msg/Twist` on `/cmd_vel_raw`
- Execute a short sequence: move forward, rotate, then stop

Use this when you want to prototype behavior **inside** the ROS 2 environment.

In this mode, your VM is still responsible for publishing camera data (for example on `/camera/image_raw`) if you want to pair motion with vision.

## Example: YOLO Vision (`vision_yolo.py`)

For most developers, the more interesting path is running vision locally with no ROS 2 installed on the client. With your VM publishing a camera topic on `/camera/image_raw` and `rosbridge` running at the URL in `robot_config.yaml`, run:

```bash
uv run python src/vision_yolo.py
```

This script:

- Connects to rosbridge via `roslibpy` (Python-only client)
- Subscribes to `sensor_msgs/Image` on `/camera/image_raw` (or a compressed variant if you configure it that way)
- Runs YOLO object detection on CPU using `ultralytics` and OpenCV
- Publishes:
  - Annotated images on an **annotated image topic** (by default `/camera/image_annotated`)

Then, in VS Code (TensorFleet extension, using the existing panels):

- Open the **Raw Message panel** and select `/camera/image_annotated`
- Use the **Teleop panel** to move the robot while detections update

If `ultralytics` is not installed, the node falls back to drawing a single demo bounding box so you can still validate:

- Topic wiring from VM → local Python → VM
- Panel integration (Image/Raw/Teleop)

YOLO is run on **CPU only**; no GPU is required.

## Example: Obstacle Avoidance (`obstacle_avoider.py`)

The `obstacle_avoider.py` script shows how to combine motion and vision/laser data into a simple behavior loop. When running it, make sure that:

- Your VM publishes raw camera images on `/camera/image_raw` (or the topic configured in `robot_config.yaml`).
- Any perception pipeline (e.g. `vision_yolo.py`) publishes an **annotated image topic** if you want to visualize processed images alongside obstacle avoidance.
- The node subscribes to the raw/annotated topics as configured and publishes velocity commands to the motion topic (for example `/cmd_vel_raw`), similar to `robot_mover.py`.

In TensorFleet, you can:

- Use the **Image panel** to view `/camera/image_raw` or your YOLO annotated topic.
- Use the **Raw Messages panel** to inspect detections (e.g. `/camera/yolo/detections`).
- Use the **Teleop panel** or your own logic in `obstacle_avoider.py`/`robot_mover.py` to command motion while observing obstacle avoidance behavior.

## Customization Ideas

- Adjust topics and model name in `vision_yolo.py` (or `robot_config.yaml`) to match your robot.
- Replace the YOLO model with your own detector or segmentation network.
- Extend `robot_mover.py` into keyboard teleop, state machines, or mission logic.
- Add new Python nodes in `src/` that publish/subscribe via rosbridge, using this template as a reference.

Happy hacking on robots! ✨
