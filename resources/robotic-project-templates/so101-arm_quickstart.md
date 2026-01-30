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

### D. Run teleoperation

Pick the script that matches your desired input:

- **Simulated arm via keyboard (default, still available)**:
  ```bash
  uv run python src/teleop_so_arm101.py --input keyboard
  ```
  The simulator resets to the starting (home) position each time the script launches.

- **Simulated arm via keyboard, mirrored to a real follower**:
  ```bash
  uv run python src/teleop_so_arm101.py --input keyboard --follower-port /dev/ttyACM0
  ```

- **Simulated arm driven by leader arm input (lowest latency)**:
  ```bash
  uv run python src/teleop_so_arm101.py --input leader --leader-port /dev/ttyACM1 --leader-id my_awesome_leader_arm
  ```
  Swap `/dev/ttyACM1` and `my_awesome_leader_arm` with the port and nickname you recorded earlier. Pass `--calibration-dir /path/to/calibration/teleoperators/so_leader` if needed.

- **Leader arm driving a real follower (relative mirroring, no jump-to-home)**:
  ```bash
  uv run python src/teleop_so_arm101.py --input leader --leader-port /dev/ttyACM1 --follower-port /dev/ttyACM0
  ```

Feel free to reconnect the leader arm and rerun the leader command whenever you need live control; the simulator reads from the saved teleop ID and port every time.

## 3. Record a LeRobot dataset (sim)

This recorder writes a LeRobot dataset by subscribing to `/joint_states`, action commands, and the SO-ARM101 cameras over rosbridge.

### A. Run the recorder

```bash
uv run python src/record_so_arm101_dataset.py
```

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
- `q`: quit (saves current episode if it has frames)

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
lerobot-dataset-viz \
  --repo-id local/so_arm101_sim_YYYYMMDD_HHMMSS \
  --root ./datasets/so_arm101_sim_YYYYMMDD_HHMMSS \
  --episode-index 0
```

### C. Start a minimal training run

```bash
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=local/so_arm101_sim_YYYYMMDD_HHMMSS \
  --dataset.root=./datasets/so_arm101_sim_YYYYMMDD_HHMMSS \
  --policy.type=act \
  --steps=1000 \
  --batch_size=4 \
  --num_workers=0 \
  --eval_freq=0 \
  --wandb.enable=false
```

If you want CPU only, add `--policy.device=cpu`.
