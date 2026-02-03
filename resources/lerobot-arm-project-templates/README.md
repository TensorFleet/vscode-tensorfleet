# TensorFleet LeRobot Arm Project

Control the SO-ARM101 robotic arm in simulation, with real hardware, or both.

| Track | What You Need | Time to First Command |
|-------|---------------|----------------------|
| 🎮 **A: Simulation Only** | Keyboard | ~2 minutes |
| 🦾 **B: Real Arm** | SO101 leader arm + USB | ~30 min (first-time setup) |
| 🔗 **C: Sim + Real** | Leader + follower arms | ~30 min (first-time setup) |

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

Click the **TensorFleet** status bar at the bottom of VS Code → **Start VM**.

> Wait for the VM status to show "Running" before proceeding (~30 seconds).

### Step 2: Install Dependencies

```bash
uv venv && uv pip install -r requirements.arm.txt
```

### Step 3: Open Webview Panels

**Simulation View**: Shows the 3D arm in Gazebo
   - Command Palette → "TensorFleet: Open Simulation View"

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

**Time**: ~30 minutes (first-time only)

> [!IMPORTANT]
> Complete all steps before running scripts. After first-time setup, you only need to run the final command.

### Find Your USB Port

```bash
source .venv/bin/activate
lerobot-find-port
```

Disconnect the arm when prompted. Note the port (e.g., `/dev/ttyACM0` on Linux, `/dev/tty.usbmodem...` on macOS).

### Setup Motors (First-Time Only)

> [!WARNING]
> This step is **required** for new motors. Takes ~15 minutes.

Follow the [HuggingFace motor setup guide](https://huggingface.co/docs/lerobot/en/so101#2-set-the-motors-ids-and-baudrates). You'll run `lerobot-setup-motors` and connect each motor one at a time.

### Calibrate the Arm

**Leader arm:**
```bash
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=my_leader
```

**Follower arm** (if using):
```bash
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=my_follower
```

### Configure Environment Variables

```bash
cp .env.example .env
```

Edit `.env` with your values:

```bash
# Required for leader arm input
SO101_LEADER_PORT=/dev/ttyACM0
SO101_LEADER_ID=my_leader

# Required for follower mirroring (optional)
SO101_FOLLOWER_PORT=/dev/ttyACM1
SO101_FOLLOWER_ID=my_follower
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
├── requirements.arm.txt       # Python dependencies
├── .env.example               # Environment variable template
└── assets/                    # Screenshots for this guide
```

---

## Next Steps

- **Record demonstrations**: Collect training data for imitation learning
- **Train models**: Use LeRobot to train policies from your data
- **[LeRobot documentation](https://huggingface.co/docs/lerobot)**: Full SDK reference
