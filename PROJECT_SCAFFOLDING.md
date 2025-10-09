# TensorFleet Project Scaffolding

## Overview

The TensorFleet extension now includes a **"New Project"** feature that scaffolds a complete drone development project with pre-configured templates, example code, and best practices.

## How to Use

### Method 1: Using the TensorFleet Tools Panel

1. Open VS Code
2. Click on the **TensorFleet** icon in the Activity Bar (left sidebar)
3. In the "TensorFleet Tools" panel at the bottom, click **"🚀 New Project"**
4. Enter your project name (e.g., `my-drone-project`)
5. Select the location where you want to create the project
6. Choose how to open the project:
   - **Open Project** - Opens in current window
   - **Open in New Window** - Opens in a new VS Code window
   - **Close** - Just creates the project without opening

### Method 2: Using Command Palette

1. Press `Cmd+Shift+P` (Mac) or `Ctrl+Shift+P` (Windows/Linux)
2. Type "TensorFleet: Create New Project"
3. Press Enter
4. Follow the prompts

## What Gets Created

When you create a new project, you get a complete drone development workspace with:

```
my-drone-project/
├── README.md                    # Project documentation
├── requirements.txt             # Python dependencies
├── .gitignore                   # Git ignore rules
├── src/
│   └── main.py                  # Example ROS 2 drone controller
├── config/
│   └── drone_config.yaml        # Drone configuration
├── missions/
│   └── example_mission.plan     # QGroundControl mission plan
└── launch/
    └── drone_sim.launch.py      # ROS 2 launch file
```

## Included Templates

### 1. Main Controller (`src/main.py`)

A complete ROS 2 drone controller example that:

- Publishes position setpoints
- Follows a pre-defined waypoint mission
- Demonstrates basic autonomous flight patterns
- Uses proper ROS 2 node structure

### 2. Drone Configuration (`config/drone_config.yaml`)

Pre-configured settings for:

- Drone model selection (iris, typhoon_h480, plane)
- Simulation world (empty, warehouse, outdoor)
- Sensor configuration (camera, lidar, GPS)
- Flight parameters (max altitude, speed, geofence)
- AI model settings (YOLOv8, depth estimation)

### 3. Mission Plan (`missions/example_mission.plan`)

A sample QGroundControl mission plan with:

- Takeoff procedure
- Multiple waypoints
- Return-to-launch (RTL) operation
- Compatible with QGroundControl

### 4. ROS 2 Launch File (`launch/drone_sim.launch.py`)

Launch configuration that starts:

- PX4 SITL (Software In The Loop)
- Gazebo simulation
- Drone model spawning
- ROS 2 bridge for PX4

### 5. Dependencies (`requirements.txt`)

All necessary Python packages:

- ROS 2 client libraries
- Computer vision (OpenCV, YOLOv8)
- Robotics utilities
- Development tools

## Next Steps After Creating a Project

### 1. Install Dependencies

```bash
cd my-drone-project
pip install -r requirements.txt
```

### 2. Configure Your Environment

Edit `config/drone_config.yaml` to match your setup:

```yaml
drone:
  model: "iris" # Your drone model

simulation:
  world: "empty" # Your preferred world
```

### 3. Launch Simulation

Use TensorFleet panels:

- Click "Open Gazebo Workspace" to start simulation
- Click "Launch ROS 2 & Stable Baselines Lab" for ROS 2 terminal

### 4. Run Your Code

```bash
# In the ROS 2 terminal
python src/main.py
```

### 5. Use AI Assistant (if MCP configured)

Ask Cursor or Claude:

- "Start Gazebo simulation with iris drone"
- "Launch the ROS 2 environment"
- "Show me the current telemetry"

## Customization

### Adding New Mission Plans

Create new `.plan` files in the `missions/` folder:

```json
{
  "fileType": "Plan",
  "version": 1,
  "mission": {
    "items": [
      // Your waypoints here
    ]
  }
}
```

### Extending the Controller

Add new behaviors in `src/main.py`:

```python
class DroneController(Node):
    def __init__(self):
        # Add your custom subscribers
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
```

### Custom Launch Configurations

Modify `launch/drone_sim.launch.py` to add:

- Additional nodes
- Custom parameters
- Different simulation scenarios

## Integration with TensorFleet Features

The scaffolded project is fully integrated with:

✅ **Gazebo Simulation** - Pre-configured launch files
✅ **ROS 2** - Ready-to-use ROS 2 nodes
✅ **QGroundControl** - Compatible mission plans
✅ **AI Ops** - Configuration for YOLOv8 and other models
✅ **MCP Integration** - Works seamlessly with AI assistants

## Tips

1. **Start Simple**: The example controller demonstrates basic concepts. Build on it gradually.

2. **Use Configuration**: Keep settings in `drone_config.yaml` instead of hardcoding.

3. **Version Control**: The `.gitignore` is pre-configured for Python, ROS 2, and simulation files.

4. **Test in Simulation**: Always test in Gazebo before real hardware.

5. **Leverage AI**: If MCP is configured, use natural language to explore and modify your project.

## Troubleshooting

### Project Creation Fails

- Ensure you have write permissions to the target directory
- Check that the project name is valid (letters, numbers, hyphens, underscores only)
- Make sure there's enough disk space

### Templates Not Found

- Verify the extension is properly installed
- Try running `bun run compile` in the extension directory
- Reinstall the extension if needed

### ROS 2 Issues

- Make sure ROS 2 is installed on your system
- Source your ROS 2 setup: `source /opt/ros/humble/setup.bash`
- Update paths in `launch/drone_sim.launch.py` to match your installation

## Example Workflow

Here's a complete workflow from project creation to first flight:

```bash
# 1. Create project (via UI)
# Click "🚀 New Project" → Enter "my-first-drone" → Select location

# 2. Open project in terminal
cd ~/projects/my-first-drone

# 3. Install dependencies
pip install -r requirements.txt

# 4. Review configuration
cat config/drone_config.yaml

# 5. Launch simulation (via TensorFleet panel)
# Click "Open Gazebo Workspace"

# 6. Start ROS 2 (via TensorFleet panel)
# Click "Launch ROS 2 & Stable Baselines Lab"

# 7. Run the controller
python src/main.py

# 8. Watch your drone fly! 🚁
```

## Contributing

Found a bug or have a suggestion for the project templates? Please open an issue or submit a pull request!

---

Happy drone development! 🚀✨
