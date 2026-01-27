# Simple Robot Quickstart

The simple robot template focuses on ground robots and velocity control. It is a great starting point if you want to run Python-only scripts locally while the TensorFleet VM provides ROS 2, Gazebo, and rosbridge.

## Project layout


```
.
├── src/
│   ├── lib/                    # Shared helpers for remote connectivity
│   │   ├── proxy_ws_client.py  # WebSocket proxy client for the VM Manager
│   │   ├── tensorfleet_config.py  # Configuration management
│   │   ├── robotic_utils.py    # connect_to_robot() and helpers
│   │   └── url_utils.py        # URL conversion utilities
│   ├── robot_mover.py          # Simple ROS 2 velocity node demo
│   ├── obstacle_avoider.py     # LiDAR-based obstacle avoidance demo
│   └── vision_yolo.py          # YOLO vision node via rosbridge
├── config/
│   └── robot_config.yaml       # Robot & network configuration
├── launch/                     # Optional ROS 2 launch files
├── .env.example                # Environment variable template
├── .tensorfleet                # TensorFleet workspace marker
├── requirements.txt            # Python dependencies
└── simple-robot_quickstart.md  # This guide
```

## Quick start

1. **Start the VM** via the **TensorFleet** status indicator in VS Code and choose **Start VM**.
2. **Install dependencies**

   ```bash
   # install uv if missing; then build the venv
   uv venv
   uv pip install -r requirements.txt
   ```

3. **Open TensorFleet panels**: Simulation View, Image Panel, and Teleops.
4. **Run a sample script**

   ```bash
   uv run python src/robot_mover.py
   ```

   Try `src/obstacle_avoider.py` for LiDAR-based navigation or `src/vision_yolo.py` to run a YOLO vision node that publishes annotated images.

## Connecting to ROS

Scripts prefer the TensorFleet proxy when `TENSORFLEET_BASE_URL` and `TENSORFLEET_JWT` are set in `.env`, and when the extension is connected to a VM. If proxy variables are missing, the helpers fall back to direct rosbridge using `ROS_HOST` / `ROS_PORT` or values from `config/robot_config.yaml`.

## Environment variables

| Variable | Purpose | Default |
|----------|---------|---------|
| `TENSORFLEET_BASE_URL` | VM Manager base URL (managed by the extension) | – |
| `TENSORFLEET_JWT` | Auth token managed by the extension | – |
| `ROS_HOST` | Direct rosbridge hostname | `172.16.0.10` |
| `ROS_PORT` | Direct rosbridge port | `9091` |
| `CMD_VEL_TOPIC` | Velocity command topic | `/cmd_vel_raw` |
| `SCAN_TOPIC` | LiDAR scan topic | `/scan` |
| `IMAGE_TOPIC` | Camera stream topic | `/camera/image_raw` |
| `LINEAR_SPEED` | Forward speed (m/s) | `0.2` |
| `ANGULAR_SPEED` | Turn speed (rad/s) | `0.5` |

## Example scripts

### `src/robot_mover.py`

Publishes `geometry_msgs/Twist` to `/cmd_vel_raw` and cycles through a small motion sequence (forward, backward, turns, stop). Run it with:

```bash
uv run python src/robot_mover.py
```

### `src/vision_yolo.py`

Connects to rosbridge with `roslibpy`, subscribes to `/camera/image_raw`, runs YOLO on every frame, and republishes annotated images on `/camera/image_annotated`.

1. Open Simulation View, Image Panel, and Teleops.
2. Select `/camera/image_raw` in the Image Panel.
3. Run the script:

   ```bash
   uv run python src/vision_yolo.py
   ```

4. Switch to `/camera/image_annotated` to see detections.

YOLO runs on CPU so no GPU is required.

### `src/obstacle_avoider.py`

Uses `/scan` to detect obstacles, slows down or turns, and backs up depending on clearance. Launch it as:

```bash
uv run python src/obstacle_avoider.py
```

## Customization ideas

- Change `vision_yolo.py` topics or models via the environment variables above.
- Replace the YOLO model with your own detector or tuning.
- Build new Python nodes in `src/` that publish/subscribe through rosbridge, extend `robot_mover.py`, or implement state machines/mission logic.

Happy hacking! ✨
