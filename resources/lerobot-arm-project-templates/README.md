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
├── src/teleop_so_arm101.py         # Teleoperation script
├── src/record_so_arm101_dataset.py # Dataset recorder
├── src/deploy_act.py               # ACT policy deployment
├── src/lib/                        # Hardware bridge and utilities
├── scripts/train_act.sh            # Train ACT policy
├── scripts/train_act_overfit.sh    # Quick overfit test
├── requirements.txt                # Python dependencies
├── .env.example                    # Environment variable template
└── assets/                         # Screenshots for this guide
```

---

## 3. Record a dataset

```bash
# Keyboard teleop
uv run python src/record_so_arm101_dataset.py --input keyboard

# Leader arm teleop
uv run python src/record_so_arm101_dataset.py --input leader
```

> [!IMPORTANT]
> Always pass `--input keyboard` or `--input leader`. Without teleop the arm sits idle and the dataset is useless for training.

### Controls

| Key | Action |
|-----|--------|
| `r` | Start recording episode |
| `s` | Save episode and start next |
| `d` | Discard current episode |
| `f` | Finalize dataset and quit (saves pending episode) |
| `q` / `Ctrl-C` | Quit without finalizing |

### Resume an existing dataset

```bash
uv run python src/record_so_arm101_dataset.py --resume --root ./datasets/so_arm101_sim_YYYYMMDD_HHMMSS
```

> [!NOTE]
> `--resume` requires `--root`, and your settings (fps, cameras, image/video mode) must match the existing dataset.

---

## 4. Train the policy

### Full training run

```bash
./scripts/train_act.sh ./datasets/so_arm101_sim_YYYYMMDD_HHMMSS local/so_arm101_sim_YYYYMMDD_HHMMSS
```

Checkpoints are saved to `outputs/train/act_so_arm101_sim_YYYYMMDD_HHMMSS/`.

### Quick overfit test (verify pipeline on a small dataset)

```bash
./scripts/train_act_overfit.sh ./datasets/so_arm101_sim_YYYYMMDD_HHMMSS local/so_arm101_sim_YYYYMMDD_HHMMSS
```

### Resume training

```bash
./scripts/train_act.sh --resume outputs/train/act_so_arm101_sim_YYYYMMDD_HHMMSS
```

Pass extra args to override config, e.g. `--steps=50000`.

> [!TIP]
> Set `TRAIN_STEPS`, `BATCH_SIZE`, `SAVE_FREQ`, or `NUM_WORKERS` as environment variables to override defaults without editing the script.

---

## 5. Deploy the policy

Run a trained checkpoint in the simulation. The script loads the policy, subscribes to camera and joint state topics, and publishes trajectory commands in a loop.

### Run deployment

```bash
uv run python src/deploy_act.py \
  --policy-path outputs/train/act_so_arm101_sim_YYYYMMDD_HHMMSS/checkpoints/last/pretrained_model
```

### Smoke test (load + one inference, no continuous loop)

```bash
uv run python src/deploy_act.py \
  --policy-path outputs/train/act_so_arm101_sim_YYYYMMDD_HHMMSS/checkpoints/last/pretrained_model \
  --check
```

### Controls

| Key | Action |
|-----|--------|
| `Space` | Pause / resume |
| `q` / `Ctrl-C` | Quit |

### Common options

| Flag | Default | Description |
|------|---------|-------------|
| `--policy-path` | *(required)* | Path to `pretrained_model` directory |
| `--fps` | `5` | Control loop frequency (Hz) |
| `--action-steps` | `10` | Actions to execute per inference |
| `--device` | `cuda` | Inference device: `cuda`, `cpu`, or `mps` |
| `--cameras` | `wrist,agent,side` | Comma-separated camera list |
| `--trajectory-time` | `0.2` | `time_from_start` for each trajectory point (s) |
| `--debug` | — | Log commands and joint states |
| `--check` | — | Smoke test: connect, infer once, exit |

---

## Appendix: Recorder Reference

### Common overrides

```bash
# Record 20 episodes at 10 seconds each
uv run python src/record_so_arm101_dataset.py \
  --episodes 20 \
  --episode-seconds 10 \
  --task "pick-and-place"

# Disable images (log joint/action data only)
uv run python src/record_so_arm101_dataset.py --no-images

# Record just wrist + agent cameras
uv run python src/record_so_arm101_dataset.py --cameras wrist,agent

# Start recording without waiting for action commands
uv run python src/record_so_arm101_dataset.py --no-wait-for-action

# Leader arm teleop with explicit port/id
uv run python src/record_so_arm101_dataset.py \
  --input leader \
  --leader-port /dev/ttyACM1 \
  --leader-id my_awesome_leader_arm
```

### Preflight check

```bash
uv run python src/record_so_arm101_dataset.py --check
```

Verifies topics, cameras, and ffmpeg before recording.

### Defaults

- Dataset root: `./datasets/so_arm101_sim_<timestamp>`
- Repo id: `local/so_arm101_sim_<timestamp>`
- FPS: 5
- Cameras: wrist, agent, side
- Video codec: h264 (pass `--no-videos` to store PNGs instead)

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
