# TensorFleet Robotic Project

Welcome to your TensorFleet robotic workspace! 🤖

This template is focused on ground robots and simple velocity control instead of drone flight.

## Project Structure

```
.
├── src/                       # Robotic control scripts
│   └── robot_mover.py        # Basic cmd_vel_raw publisher
├── config/                    # Robot & network configuration
│   └── robot_config.yaml
├── launch/                    # (Optional) ROS 2 launch files
├── requirements.txt           # Python dependencies
└── GETTING_STARTED.md         # This guide
```

## Quick Start

1. Create a **Robotic Project** from the TensorFleet tooling view in VS Code.
2. In the new project folder, install deps with uv:

```bash
# If uv isn't installed: curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv
uv pip install -r requirements.txt
```

3. Make sure your ROS 2 environment is sourced and that your VM at `172.16.0.10` is running the bridges:

- rosbridge at port `8080`
- foxglove bridge at port `8765`

4. Run the example movement script:

```bash
uv run python src/robot_mover.py
```

This will publish velocity commands to `cmd_vel_raw` so you can see basic movement on your robot (or in your simulator).

## Next Steps

- Edit `config/robot_config.yaml` to match your robot.
- Extend `src/robot_mover.py` with your own motion primitives or behaviors.
- Wire the VM’s rosbridge / Foxglove endpoints into your UI or monitoring tools.

Happy hacking on robots! ✨
