# TensorFleet Drone Project

Welcome to your new TensorFleet drone development project! 🚁

## Project Structure

```
.
├── src/                    # Your drone control scripts
│   └── main.py            # Main entry point
├── missions/              # Mission plans and waypoints
│   └── example_mission.plan
├── config/                # Configuration files
│   └── drone_config.yaml
├── launch/                # ROS 2 launch files
│   └── drone_sim.launch.py
└── README.md             # This file
```

## Quick Start

### 1. Launch Simulation

Open the TensorFleet panel in VS Code and click "Open Gazebo Workspace" to start the simulator.

Or use the command palette:

```
TensorFleet: Open Gazebo Workspace
```

### 2. Start ROS 2

Click "Launch ROS 2 & Stable Baselines Lab" to open the ROS 2 environment.

### 3. Run Your First Flight

```bash
# Source your ROS 2 environment
source /opt/ros/humble/setup.bash  # Adjust for your ROS 2 installation

# Run the example script
python src/main.py
```

## Available Tools

### AI-Powered Development (with Cursor/Claude)

If you've configured the TensorFleet MCP server, you can use natural language commands:

- "Start a Gazebo simulation with iris drone"
- "Show me the current drone telemetry"
- "Run YOLOv8 detection on the camera feed"
- "Create a waypoint mission"

See [QUICK_START.md](../QUICK_START.md) for MCP setup instructions.

### Manual Commands

All TensorFleet commands are available in the Command Palette (`Cmd+Shift+P`):

- `TensorFleet: Open QGroundControl Workspace`
- `TensorFleet: Open Gazebo Workspace`
- `TensorFleet: Open AI Ops Workspace`
- `TensorFleet: Open ROS 2 & Stable Baselines Lab`

## Next Steps

1. **Configure your drone:** Edit `config/drone_config.yaml`
2. **Create missions:** Add waypoint files to `missions/`
3. **Write control code:** Implement your logic in `src/`
4. **Test in simulation:** Use Gazebo for safe testing

## Resources

- [PX4 Documentation](https://docs.px4.io/)
- [ROS 2 Documentation](https://docs.ros.org/)
- [QGroundControl User Guide](https://docs.qgroundcontrol.com/)

Happy flying! ✨
