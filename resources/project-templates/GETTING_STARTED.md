# Getting Started with Your TensorFleet Project

Welcome to your new drone development project! This guide will help you get up and running quickly.

## 📋 Prerequisites

Before you begin, make sure you have:

- [ ] ROS 2 installed (Humble or newer recommended)
- [ ] Python 3.8+ installed
- [ ] PX4 Autopilot (for simulation)
- [ ] Gazebo (for 3D simulation)
- [ ] TensorFleet VS Code extension installed

## 🚀 Quick Start (5 Minutes)

### Step 1: Install Python Dependencies

```bash
pip install -r requirements.txt
```

### Step 2: Configure Your Drone

Edit `config/drone_config.yaml` to set your preferences:

```yaml
drone:
  model: "iris" # Options: iris, typhoon_h480, plane

simulation:
  world: "empty" # Options: empty, warehouse, outdoor
```

### Step 3: Launch Simulation

**Option A - Using TensorFleet Panels (Recommended):**

1. Open VS Code
2. Click **TensorFleet** icon in sidebar
3. Click "Open Gazebo Workspace" to start simulation

**Option B - Command Line:**

```bash
ros2 launch launch/drone_sim.launch.py
```

### Step 4: Run Your First Flight

In a new terminal:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Run the example controller
python src/main.py
```

Watch your drone execute the waypoint mission! 🚁

## 📁 Project Structure

```
.
├── src/                    # Your Python code
│   └── main.py            # Main drone controller
├── config/                # Configuration files
│   └── drone_config.yaml  # Drone settings
├── missions/              # Mission plans
│   └── example_mission.plan
├── launch/                # ROS 2 launch files
│   └── drone_sim.launch.py
└── requirements.txt       # Python dependencies
```

## 🔧 Customization

### Modify Flight Behavior

Edit `src/main.py` to change waypoints:

```python
self.waypoints = [
    (0.0, 0.0, 5.0),   # Takeoff
    (10.0, 0.0, 5.0),  # Your waypoints here
    # ... add more
]
```

### Create New Missions

Add `.plan` files in `missions/` folder:

```json
{
  "fileType": "Plan",
  "version": 1,
  "mission": {
    "items": [...]
  }
}
```

Import these in QGroundControl!

### Configure Sensors

Edit `config/drone_config.yaml`:

```yaml
sensors:
  camera:
    enabled: true
    resolution: [640, 480]

  lidar:
    enabled: true
    range: 10.0
```

## 🤖 AI-Powered Development (Optional)

If you have TensorFleet MCP configured with Cursor or Claude:

**Just ask:**

- "Start a Gazebo simulation with iris"
- "Show me the current telemetry"
- "Run YOLOv8 on the camera feed"
- "Modify the waypoint mission"

The AI will interact with your workspace automatically!

## 📚 Learn More

### ROS 2 Topics

Your drone publishes to these topics:

- `/drone/setpoint_position` - Position commands
- `/drone/status` - Status messages

Subscribe to PX4 topics:

```bash
ros2 topic list
ros2 topic echo /fmu/out/vehicle_status
```

### Available Commands

In VS Code Command Palette (`Cmd+Shift+P`):

- `TensorFleet: Open Gazebo Workspace`
- `TensorFleet: Open QGroundControl Workspace`
- `TensorFleet: Open AI Ops Workspace`
- `TensorFleet: Open ROS 2 & Stable Baselines Lab`

### Useful Resources

- [PX4 User Guide](https://docs.px4.io/)
- [ROS 2 Documentation](https://docs.ros.org/)
- [Gazebo Tutorials](https://gazebosim.org/docs)
- [TensorFleet Extension Docs](../README.md)

## 🐛 Troubleshooting

### "ROS 2 not found"

```bash
# Source your ROS 2 installation
source /opt/ros/humble/setup.bash

# Add to ~/.bashrc for permanent:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### "Gazebo fails to start"

```bash
# Check if Gazebo is installed
which gazebo

# Install if needed (Ubuntu):
sudo apt install gazebo
```

### "Python dependencies error"

```bash
# Create a virtual environment
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### "PX4 not found"

Update the path in `launch/drone_sim.launch.py`:

```python
px4_sitl = ExecuteProcess(
    cmd=['make', 'px4_sitl', 'gazebo'],
    cwd='/path/to/your/PX4-Autopilot',  # Update this
    output='screen'
)
```

## 🎯 Next Steps

1. **Explore the example** - Understand how `main.py` works
2. **Modify waypoints** - Create your own flight path
3. **Add sensors** - Enable camera, lidar in config
4. **Implement vision** - Use AI models for object detection
5. **Create missions** - Design complex missions in QGroundControl

## 💡 Tips

✅ **Always test in simulation first** before real hardware

✅ **Use version control** - The `.gitignore` is pre-configured

✅ **Check logs** - Output goes to `logs/` directory

✅ **Leverage AI** - Use MCP integration for faster development

✅ **Join community** - Share your projects!

---

Happy flying! 🚁✨

Need help? Check the [main documentation](../README.md) or open an issue.
