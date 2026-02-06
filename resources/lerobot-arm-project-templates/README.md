# TensorFleet LeRobot Arm Project

Control the SO-ARM101 robotic arm in simulation, with real hardware, or both.

| Track | What You Need |
|-------|---------------|
| 🎮 **A: Simulation Only** | Keyboard |
| 🦾 **B: Real Arm** | SO101 leader arm + USB |
| 🔗 **C: Sim + Real** | Leader + follower arms |

---

## Prerequisites
- **uv** package manager [install](https://docs.astral.sh/uv/getting-started/installation/): 
```
curl -LsSf https://astral.sh/uv/install.sh | sh
```
---

## Quick Start (All Tracks)

Complete these 3 steps before choosing your track.

### Step 1: Start the VM

1. Open the **TensorFleet** extension in the sidebar
2. Click **Open Server Settings** at the top
3. In **VM Controls**, click **Start VM**

> Wait for the VM status to show "Running" before proceeding.

![Start VM](assets/start_vm.gif)

### Step 2: Install Dependencies

```bash
uv venv && uv pip install -r requirements.txt
```

### Step 3: Open Simulation View

1. Open the **TensorFleet** extension in the sidebar
2. Click **Open Drone Views** at the top
3. Click **Simulation View**

**Or via Command Palette:** `TensorFleet: Open Simulation View`

![Open Simulation View](assets/simulation_view.gif)

> [!TIP]
> If the simulation doesn't appear after a while, close and re-open the panel. If the problem persists, restart the VM.

---

## 🎮 Track A: Simulation Only (Keyboard)

Run the teleoperation script:

```bash
uv run python src/teleop_so_arm101.py
```

Use [keyboard controls](#keyboard-controls) to move the arm. **The terminal must be focused** for keys to work.
✅ **Done!** You're controlling the simulated arm.

---

## 🦾 Track B: Real Arm Setup



> [!IMPORTANT]
> **Already set up your SO101?** Skip to [Configure Environment Variables](#configure-environment-variables) if you've completed motor setup and calibration.

### Prerequisites: SO101 Hardware Setup

Before using this project, your arm must have motors configured and calibrated.

👉 **Follow the official guide**: [HuggingFace SO101 Setup](https://huggingface.co/docs/lerobot/en/so101)

Complete these sections:
1. **Motor IDs & Baudrates** — `lerobot-setup-motors`
2. **Calibration** — `lerobot-calibrate`

### Find Your USB Ports

> [!TIP]
> **Using two arms?** Identify ports one at a time to avoid confusion.

```bash
source .venv/bin/activate
lerobot-find-port
```

**For two-arm setups (leader + follower):**
1. **Unplug both arms**
2. **Plug in leader arm only** → run `lerobot-find-port` → note as `LEADER_PORT`
3. **Plug in follower arm** (keep leader plugged) → run again → the new port is `FOLLOWER_PORT`

| Arm | Example Port (macOS) | .env Variable |
|-----|----------------------|---------------|
| Leader | `/dev/tty.usbmodem58760431551` | `SO101_LEADER_PORT` |
| Follower | `/dev/tty.usbmodem58760431552` | `SO101_FOLLOWER_PORT` |

> [!NOTE]
> See [HuggingFace: Find Port](https://huggingface.co/docs/lerobot/en/so101#1-find-the-usb-ports-for-each-arm) for more details.

### Find Your Calibration IDs

The calibration ID is the name you gave when running `lerobot-calibrate`. If you forgot it, check:

```bash
# Leader calibration
ls ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/
# → my_awesome_leader_arm.json

# Follower calibration  
ls ~/.cache/huggingface/lerobot/calibration/robots/so_follower/
# → my_awesome_follower_arm.json
```

The `.json` filename (without extension) is your ID.

> [!NOTE]
> **Need to calibrate?** See [HuggingFace Calibration Guide](https://huggingface.co/docs/lerobot/en/so101#3-calibrate). Run `lerobot-calibrate` and follow the prompts.

### Configure Environment Variables

```bash
cp .env.example .env
```

Edit `.env` with your values:

```bash
# Required for leader arm
SO101_LEADER_PORT=/dev/tty.usbmodem58760431551
SO101_LEADER_ID=my_awesome_leader_arm

# For follower mirroring (Track C only)
SO101_FOLLOWER_PORT=/dev/tty.usbmodem58760431552
SO101_FOLLOWER_ID=my_awesome_follower_arm
```

### Run Leader Teleop

```bash
uv run python src/teleop_so_arm101.py --input leader
```

The simulated arm now mirrors your leader arm movements!

---

## 🔗 Track C: Sim + Real (Leader → Sim → Follower)

**Prerequisite**: Complete Track B for both leader and follower arms.

This mode chains: **Leader arm → Simulator → Follower arm**

```bash
uv run python src/teleop_so_arm101.py --input leader --use-follower-env
```

The leader drives the simulation, which mirrors to the real follower.

---

## Keyboard Controls

| Key | Action |
|-----|--------|
| `q` / `a` | Joint 1 +/- |
| `w` / `s` | Joint 2 +/- |
| `e` / `d` | Joint 3 +/- |
| `r` / `f` | Joint 4 +/- |
| `t` / `g` | Joint 5 +/- |
| `y` / `h` | Gripper open/close |
| `0` | Return to home position |
| `Space` | Hold current position |
| `x` / `Ctrl-C` | Exit |

---

## Environment Variables

| Variable | Track | Description |
|----------|-------|-------------|
| `SO101_LEADER_PORT` | Real Arm, Sim + Real | USB port for leader arm |
| `SO101_LEADER_ID` | Real Arm, Sim + Real | Calibration ID for leader |
| `SO101_FOLLOWER_PORT` | Sim + Real | USB port for follower arm |
| `SO101_FOLLOWER_ID` | Sim + Real | Calibration ID for follower |

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| VM won't start | Check TensorFleet status bar → Restart VM |
| `lerobot-find-port` shows nothing | Check USB cable and power supply |
| Calibration file not found | Re-run `lerobot-calibrate` with matching `--teleop.id` |
| Connection refused (port 9091) | Wait for VM to fully start, check Simulation View |
| Keys don't work | Click inside the terminal to focus it |

---

## Project Contents

```
.
├── src/teleop_so_arm101.py    # Main teleoperation script
├── src/lib/                   # Hardware bridge and utilities
├── requirements.txt       # Python dependencies
├── .env.example               # Environment variable template
└── assets/                    # Screenshots for this guide
```

---

## Next Steps

- **Record demonstrations**: Collect training data for imitation learning
- **Train models**: Use LeRobot to train policies from your data
- **[LeRobot documentation](https://huggingface.co/docs/lerobot)**: Full SDK reference

## 3. Record a LeRobot dataset (sim)

This recorder writes a LeRobot dataset by subscribing to `/joint_states`, action commands, and the SO-ARM101 cameras over rosbridge.

### A. Run the recorder

```bash
uv run python src/record_so_arm101_dataset.py --input keyboard
```

> [!IMPORTANT]
> Always include `--input keyboard` or `--input leader` when recording. Without teleop, the arm sits idle and you'll record a static dataset that isn't useful for training.

Defaults:
- Dataset root: `./datasets/so_arm101_sim_<timestamp>`
- Repo id: `local/so_arm101_sim_<timestamp>`
- FPS: 5 (matches sim cameras)
- Cameras: wrist, agent, side
- Video codec: h264

> [!NOTE]
> Recording videos requires `ffmpeg` to be available in `PATH`. If not, pass `--no-videos` to store PNGs.

### B. Controls

- `n`: end episode and start next
- `p`: pause/resume recording
- `x` / `Ctrl-C`: quit (saves current episode if it has frames)

### C. Common overrides

```bash
# Record 20 episodes at 10 seconds each
uv run python src/record_so_arm101_dataset.py \
  --episodes 20 \
  --episode-seconds 10 \
  --task "pick-and-place"

# Disable images and log only joint/action data
uv run python src/record_so_arm101_dataset.py --no-images

# Record just wrist + agent cameras
uv run python src/record_so_arm101_dataset.py --cameras wrist,agent

# Record even if action topics are missing (actions mirror joint states until commands arrive)
uv run python src/record_so_arm101_dataset.py --no-wait-for-action

# Record + keyboard teleop in the same process
uv run python src/record_so_arm101_dataset.py --input keyboard

# Record + leader teleop in the same process
uv run python src/record_so_arm101_dataset.py \
  --input leader \
  --leader-port /dev/ttyACM1 \
  --leader-id my_awesome_leader_arm

# Lerobot-style flags (mirrors lerobot-record)
uv run python src/record_so_arm101_dataset.py \
  --robot.type=so101_follower \
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 1920, height: 1080, fps: 30}}" \
  --ros.camera_topics="{front: /so_arm101/agent_camera/image_raw}" \
  --teleop.type=so101_leader \
  --teleop.port /dev/ttyACM1 \
  --teleop.id my_awesome_leader_arm \
  --dataset.repo_id=local/record-test \
  --dataset.num_episodes=5 \
  --dataset.single_task="Grab the black cube"

# Append to an existing dataset root
uv run python src/record_so_arm101_dataset.py --resume --root ./datasets/so_arm101_sim_20250130_120000
```

> [!IMPORTANT]
> `--resume` is strict: it requires `--root`, and your current settings (fps, cameras, image/video mode)
> must match the existing dataset metadata.

### D. Preflight check (optional)

```bash
uv run python src/record_so_arm101_dataset.py --check
```

Use `--check` to confirm topics, images, and ffmpeg before recording.

## 4. Test + train the dataset

### A. Quick sanity check

```bash
python - <<'PY'
from lerobot.datasets.lerobot_dataset import LeRobotDataset

repo_id = "local/so_arm101_sim_YYYYMMDD_HHMMSS"
root = "./datasets/so_arm101_sim_YYYYMMDD_HHMMSS"

ds = LeRobotDataset(repo_id, root=root)
print("episodes:", ds.meta.total_episodes, "frames:", ds.meta.total_frames)
print("features:", list(ds.meta.features.keys()))
print("sample keys:", list(ds[0].keys()))
PY
```

### B. Visualize an episode (optional)

```bash
# HF_HUB_OFFLINE=1 prevents any HuggingFace hub access
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id local/so_arm101_sim_YYYYMMDD_HHMMSS \
  --root ./datasets/so_arm101_sim_YYYYMMDD_HHMMSS \
  --episode-index 0 \
  --display-compressed-images true
```

> [!NOTE]
> If you recorded with an older recorder and see `meta/tasks.parquet` missing, recreate it once (replace `teleop` with your `--task` label):
> ```bash
> python - <<'PY'
> from pathlib import Path
> import pandas as pd
>
> root = Path("./datasets/so_arm101_sim_YYYYMMDD_HHMMSS")
> tasks = pd.DataFrame({"task_index": [0]}, index=["teleop"])
> tasks.to_parquet(root / "meta" / "tasks.parquet")
> print("Wrote", root / "meta" / "tasks.parquet")
> PY
> ```

### C. Start a minimal training run

```bash
# HF_HUB_OFFLINE=1 prevents any HuggingFace hub access
HF_HUB_OFFLINE=1 python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=local/so_arm101_sim_YYYYMMDD_HHMMSS \
  --dataset.root=./datasets/so_arm101_sim_YYYYMMDD_HHMMSS \
  --policy.type=act \
  --policy.repo_id=local/so_arm101_policy \
  --steps=1000 \
  --batch_size=4 \
  --num_workers=0 \
  --eval_freq=0 \
  --wandb.enable=false \
  --save_checkpoint=true \
  --output_dir=./outputs
```

> [!TIP]
> - `--policy.repo_id` is required even for local training (just use any local name)
> - `--output_dir` specifies where checkpoints are saved
> - Add `--policy.device=cpu` for CPU-only training

---

## Appendix: Recorder Argument Reference

Complete list of arguments for `record_so_arm101_dataset.py`.

### Dataset Options

| Argument | Default | Description |
|----------|---------|-------------|
| `--repo-id` | `local/so_arm101_sim_<timestamp>` | LeRobot dataset repo ID |
| `--root` | `./datasets/so_arm101_sim_<timestamp>` | Dataset root directory |
| `--fps` | `5` | Recording FPS |
| `--episodes` | `0` (unlimited) | Number of episodes to record |
| `--episode-seconds` | `0` (manual) | Auto-end episode after N seconds |
| `--reset-seconds` | `2.0` | Pause between episodes |
| `--task` | `teleop` | Task label for dataset |
| `--no-videos` | - | Store images as PNGs instead of videos |
| `--vcodec` | `h264` | Video codec |
| `--resume` | - | Resume an existing dataset (requires `--root`) |

### Teleop Input Options

| Argument | Default | Description |
|----------|---------|-------------|
| `--input` | `none` | Teleop mode: `none`, `keyboard`, or `leader` |
| `--leader-port` | - | USB serial port for leader arm (required for `--input leader`) |
| `--leader-id` | `awesome_leader` | Leader arm ID for calibration lookup |
| `--calibration-dir` | `~/.cache/.../so_leader/` | Path to leader calibration directory |
| `--calibration-file` | - | Path to specific calibration JSON file |
| `--leader-rate` | `100.0` | Leader state publish rate in Hz |
| `--trajectory-time` | `0.2` (keyboard) / `0.02` (leader) | Trajectory time_from_start in seconds |
| `--follower-port` | - | USB serial port for real follower arm (optional) |
| `--follower-id` | `my_awesome_follower_arm` | Follower arm ID for calibration |
| `--follower-calibration-dir` | `~/.cache/.../so_follower/` | Path to follower calibration directory |

### ROS / Rosbridge Options

| Argument | Default | Description |
|----------|---------|-------------|
| `--host` | - | Rosbridge host (or use `ROS_HOST` env) |
| `--port` | `9091` | Rosbridge port |
| `--rosbridge-url` | - | Full rosbridge URL (e.g., `ws://127.0.0.1:9091`) |
| `--state-topic` | `/joint_states` | Joint state topic |
| `--arm-action-topic` | `/arm_controller/joint_trajectory` | Arm action topic |
| `--gripper-action-topic` | `/gripper_controller/joint_trajectory` | Gripper action topic |
| `--clock-topic` | `/clock` | Clock topic for timestamps |

### Camera Options

| Argument | Default | Description |
|----------|---------|-------------|
| `--cameras` | `wrist,agent,side` | Comma-separated camera list |
| `--no-images` | - | Disable image recording |
| `--wrist-topic` | `/so_arm101/wrist_camera/image_raw` | Wrist camera topic |
| `--agent-topic` | `/so_arm101/agent_camera/image_raw` | Agent camera topic |
| `--side-topic` | `/so_arm101/side_camera/image_raw` | Side camera topic |
| `--ros.camera_topics` | - | YAML mapping of camera names to topics |
| `--display_data` | `false` | Preview camera feeds in OpenCV windows |

### LeRobot-style Aliases

These mirror `lerobot-record` flags:

| Argument | Maps to |
|----------|---------|
| `--dataset.repo_id` | `--repo-id` |
| `--dataset.root` | `--root` |
| `--dataset.fps` | `--fps` |
| `--dataset.num_episodes` | `--episodes` |
| `--dataset.episode_time_s` | `--episode-seconds` |
| `--dataset.reset_time_s` | `--reset-seconds` |
| `--dataset.single_task` | `--task` |
| `--dataset.video` | inverse of `--no-videos` |
| `--dataset.vcodec` | `--vcodec` |
| `--robot.type` | Sets robot type metadata |
| `--robot.port` | `--follower-port` |
| `--robot.id` | `--follower-id` |
| `--robot.cameras` | `--cameras` (YAML format) |
| `--teleop.type` | `--input` |
| `--teleop.port` | `--leader-port` |
| `--teleop.id` | `--leader-id` |

### Debug Options

| Argument | Default | Description |
|----------|---------|-------------|
| `--debug-leader` | - | Log leader input values |
| `--debug-raw` | - | Include raw servo ticks in debug output |
| `--debug-limits` | - | Warn when values exceed URDF limits |
| `--debug-state` | - | Log sim joint states vs commanded targets |
| `--debug-all` | - | Enable all debug options |
| `--debug-interval` | `1.0` | Seconds between debug logs |
| `--debug-log-file` | - | Append debug logs to a file |

### Utilities

| Argument | Default | Description |
|----------|---------|-------------|
| `--check` | - | Preflight check: verify topics and exit |
| `--check-timeout` | `15.0` | Seconds to wait during preflight check |
| `--wait-for-action` | auto | Wait for action commands before recording |
| `--no-wait-for-action` | - | Start recording without action commands |
