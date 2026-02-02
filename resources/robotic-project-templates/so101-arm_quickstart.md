# SO-ARM101 Quick Start

Follow this guide any time you work with the SO-ARM101 example. The first section covers the simulated keyboard teleop you have today, and the second introduces the leader arm workflow that streams real controller input into the simulated robot.

## 1. Keyboard teleop (simulated arm)

1. **Start VM**: Click the **TensorFleet** status bar in VS Code and select **Start VM**.
2. **Setup Environment**: Install dependencies using `uv`.
   ```bash
   uv venv && uv pip install -r requirements.arm.txt
   ```
3. **Open Simulation**: Launch the **Simulation View** and zoom in on the table as shown.
   ![Initial View](assets/simulation_view_01.png)
   ![Zoomed View](assets/simulation_view_02.png)
4. **Monitor Camera Feeds**: Open several **Image Panels** and subscribe to these camera topics:
   - `/so_arm101/agent_camera/image_raw`
   - `/so_arm101/side_camera/image_raw`
   - `/so_arm101/wrist_camera/image_raw`
   ![Image Panels](assets/image_panels.png)

   **Tip: Using Raw Messages**
   For a more compact view of multiple topics, use the **Raw Messages** panel:
   - Open the **Raw Messages** panel.
   - Search for your topic in **Search Topics**.
   - Select the topic and click **Subscribe**.

   ![Raw Messages](assets/raw_messages.png)

5. **Run Teleoperation**: Start the keyboard control script:
   ```bash
   uv run python src/teleop_so_arm101.py
   ```

   To mirror keyboard commands to a real follower arm, add the follower port:
   ```bash
   uv run python src/teleop_so_arm101.py --input keyboard --follower-port /dev/ttyACM0
   ```

   > [!IMPORTANT]
   > Keyboard controls will only work when the **terminal window is focused**.

### Keyboard Controls

| Key | Action |
| --- | --- |
| `q` / `a` | Joint 1 +/- |
| `w` / `s` | Joint 2 +/- |
| `e` / `d` | Joint 3 +/- |
| `r` / `f` | Joint 4 +/- |
| `t` / `g` | Joint 5 +/- |
| `y` / `h` | Gripper open/close |
| `space` | Hold current position |
| `0` | Return to home position |
| `x` / `Ctrl-C` | Exit teleoperation |

## 2. Leader arm teleop (real input into simulated arm)

This mode injects live rotations from a real SO101 leader arm into the simulator. Keep clear notes of the USB ports you use so you can reuse them later (save the port path in your notes or `.env` file). If you disconnect/reconnect cables, repeat the port discovery step.

### A. Find the leader arm USB port

1. Activate the Python environment:
   ```bash
   source .venv/bin/activate
   ```
2. Run the port discovery helper:
   ```bash
   lerobot-find-port
   ```

   **Example output:**
   ```
   Finding all available ports for the MotorBus.
   ['/dev/tty.usbmodem575E0032081', '/dev/tty.usbmodem575E0031751']
   Remove the USB cable from your MotorBus and press Enter when done.

   [...Disconnect corresponding leader or follower arm and press Enter...]

   The port of this MotorBus is /dev/tty.usbmodem575E0032081
   Reconnect the USB cable.
   ```

   > Tip: jot down the port you plan to reuse (e.g., `/dev/ttyACM1`) so calibration and teleop steps stay consistent.

### B. Calibrate the leader arm

Follow the Hugging Face calibration guide (includes video): https://huggingface.co/docs/lerobot/en/so101  

Then run:

```bash
lerobot-calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 --teleop.id=my_awesome_leader_arm
```

Replace `--teleop.port` with the port you saved and `--teleop.id` with a memorable name. If calibration data lives outside the default cache, add `--calibration-dir /path/to/calibration/teleoperators/so_leader`.

### C. Calibrate the follower arm (for mirroring)

If you want to drive a real follower arm (from the keyboard or leader), calibrate it once:

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_awesome_follower_arm
```

Replace the port and ID with your saved values.

### D. Set up environment variables (recommended)

To avoid typing long port names and IDs every time, set up environment variables:

**Option A: Create a .env file (recommended)**
```bash
cp .env.example .env
# Edit .env with your actual port and ID values
```

The script automatically loads `.env` on startup - no need to run `source .env`.

**Option B: Export directly in terminal**
```bash
export SO101_LEADER_PORT="/dev/tty.usbmodem58760431551"
export SO101_LEADER_ID="my_awesome_leader_arm"
export SO101_FOLLOWER_PORT="/dev/tty.usbmodem58760431552"
export SO101_FOLLOWER_ID="my_awesome_follower_arm"
```

> [!TIP]
> If using Option B, add the export commands to your `~/.zshrc` or `~/.bashrc` to persist them across terminal sessions.

### E. Run teleoperation

Pick the script that matches your desired input. With environment variables set (see step D), the commands are much simpler:

- **Simulated arm via keyboard (default)**:
  ```bash
  uv run python src/teleop_so_arm101.py --input keyboard
  ```
  The simulator resets to the starting (home) position each time the script launches.

- **Simulated arm via keyboard, mirrored to a real follower**:
  ```bash
  # With environment variables set:
  uv run python src/teleop_so_arm101.py --input keyboard --use-follower-env

  # Or with explicit flags:
  uv run python src/teleop_so_arm101.py --input keyboard --follower-port /dev/ttyACM0
  ```

- **Simulated arm driven by leader arm input (lowest latency)**:
  ```bash
  # With environment variables set:
  uv run python src/teleop_so_arm101.py --input leader

  # Or with explicit flags:
  uv run python src/teleop_so_arm101.py --input leader \
      --leader-port /dev/ttyACM1 \
      --leader-id my_awesome_leader_arm
  ```

- **Leader arm driving a real follower (relative mirroring, no jump-to-home)**:
  ```bash
  # With environment variables set:
  uv run python src/teleop_so_arm101.py --input leader

  # Or with explicit flags:
  uv run python src/teleop_so_arm101.py --input leader \
      --leader-port /dev/ttyACM1 \
      --follower-port /dev/ttyACM0
  ```

Feel free to reconnect the leader arm and rerun the leader command whenever you need live control; the simulator reads from the saved teleop ID and port every time.

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
