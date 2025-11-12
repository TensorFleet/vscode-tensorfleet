# TensorFleet ROS2/PX4/Gazebo Setup Guide

This guide will help you connect the TensorFleet VS Code extension to real ROS2, PX4, and Gazebo environments.

## Overview

TensorFleet integrates with:
- **ROS2** - For real-time topic subscription and publishing
- **PX4** - Flight controller (via MAVROS or PX4-ROS2 bridge)
- **Gazebo** - For simulation and testing

The extension automatically falls back to simulation mode if ROS2 is not available.

---

## Prerequisites

### 1. System Requirements

- **OS**: Ubuntu 22.04 or later (recommended)
- **Node.js**: 18.x or later
- **VS Code**: 1.85.0 or later

### 2. ROS2 Installation

Install ROS2 Humble (or Iron/Jazzy):

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify installation:
```bash
ros2 --version
# Should output: ros2 cli version humble.X.X
```

### 3. PX4 Setup

#### Option A: PX4 with Gazebo (Simulation)

```bash
# Clone PX4 Autopilot
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Install dependencies
bash ./Tools/setup/ubuntu.sh

# Build for simulation
make px4_sitl gazebo-classic
```

#### Option B: PX4 with Real Hardware

For real hardware, install MAVROS:
```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

### 4. PX4-ROS2 Bridge

```bash
# Install px4_msgs and px4_ros_com
sudo apt install ros-humble-px4-msgs

# Clone px4_ros_com
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_ros_com.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 5. Gazebo

Gazebo Garden or Harmonic (for PX4 simulation):

```bash
# Install Gazebo Garden
sudo apt-get update
sudo apt-get install lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```

---

## TensorFleet Extension Setup

### 1. Install Node.js Dependencies

```bash
cd /home/shane/vscode-tensorfleet

# Install dependencies (including rclnodejs)
npm install

# Compile the extension
npm run compile
```

### 2. Install rclnodejs

**Important**: `rclnodejs` must be installed in an environment where ROS2 is sourced.

```bash
# Source ROS2 first!
source /opt/ros/humble/setup.bash

# Install rclnodejs
cd /home/shane/vscode-tensorfleet
npm install rclnodejs

# Build rclnodejs (this compiles native bindings)
cd node_modules/rclnodejs
npm run build
```

### 3. Verify Installation

```bash
# Check if rclnodejs can find ROS2
node -e "const rclnodejs = require('rclnodejs'); console.log('rclnodejs loaded:', !!rclnodejs);"
```

If you get an error, make sure:
1. ROS2 is sourced in your shell
2. `AMENT_PREFIX_PATH` environment variable is set
3. Run `npm run build` in `node_modules/rclnodejs`

---

## Launching TensorFleet with ROS2

### Method 1: Launch VS Code with ROS2 Environment

Create a launcher script `launch_tensorfleet.sh`:

```bash
#!/bin/bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Source your workspace if applicable
# source ~/ros2_ws/install/setup.bash

# Launch VS Code with ROS2 environment
code /home/shane/vscode-tensorfleet
```

Make it executable and use it:
```bash
chmod +x launch_tensorfleet.sh
./launch_tensorfleet.sh
```

### Method 2: Extension Settings

Set environment variables in VS Code settings (`.vscode/settings.json`):

```json
{
  "terminal.integrated.env.linux": {
    "ROS_DISTRO": "humble",
    "AMENT_PREFIX_PATH": "/opt/ros/humble",
    "ROS_VERSION": "2"
  }
}
```

---

## Running a Complete Stack

### 1. Start PX4 Simulation

Terminal 1 - Start PX4 with Gazebo:
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris
```

This starts:
- PX4 flight controller (SITL)
- Gazebo with Iris quadcopter model
- MAVLink on UDP port 14540

### 2. Start PX4-ROS2 Bridge

Terminal 2 - Start MicroXRCE-DDS agent:
```bash
cd ~/PX4-Autopilot
MicroXRCEAgent udp4 -p 8888
```

Or use ROS2 launch:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

### 3. Verify Topics

Terminal 3 - Check ROS2 topics:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

You should see topics like:
- `/fmu/in/vehicle_command`
- `/fmu/out/vehicle_status`
- `/fmu/out/sensor_combined`
- `/camera/image_raw` (if camera enabled)

### 4. Connect TensorFleet

1. Open VS Code with TensorFleet extension
2. Press `Ctrl+Shift+P` → "TensorFleet: Connect to ROS2"
3. Open Image Panel or Teleops Panel from sidebar
4. Click "Connect" in the panel

---

## Using TensorFleet Panels

### Image Panel

**Subscribe to Camera Topics:**
1. Open "Image Panel (Option 3)" from TensorFleet sidebar
2. Click "Connect" button
3. Select topic from dropdown (e.g., `/camera/image_raw`)
4. Images will stream in real-time

**Available Topics:**
- `/camera/image_raw` - Uncompressed images
- `/camera/compressed` - JPEG compressed images
- `/depth/image` - Depth camera data

### Teleops Panel

**Control Your Drone:**
1. Open "Teleops (Option 3)" from TensorFleet sidebar
2. Click "Connect" button
3. Configure speeds:
   - Linear Speed: forward/backward velocity (m/s)
   - Angular Speed: rotation velocity (rad/s)
   - Publish Rate: command frequency (Hz)
4. Use keyboard to control:
   - **W** or **↑** - Forward
   - **S** or **↓** - Backward
   - **A** or **←** - Turn left
   - **D** or **→** - Turn right
   - **STOP** button - Emergency stop

Commands are published to `/cmd_vel` topic.

---

## Commands Available

Access via `Ctrl+Shift+P`:

- **TensorFleet: Connect to ROS2** - Initialize ROS2 connection
- **TensorFleet: Disconnect from ROS2** - Close ROS2 connection
- **TensorFleet: Start PX4 Telemetry Monitor** - Stream PX4 telemetry to output panel
- **TensorFleet: Open Image Panel (Option 3 - React)** - Open image viewer
- **TensorFleet: Open Teleops Panel (Option 3 - React)** - Open teleops control

---

## Troubleshooting

### Error: "rclnodejs not installed"

**Solution:**
```bash
source /opt/ros/humble/setup.bash
cd /home/shane/vscode-tensorfleet
npm install rclnodejs
cd node_modules/rclnodejs
npm run build
```

### Error: "Failed to connect to ROS2"

**Check:**
1. ROS2 is sourced: `echo $AMENT_PREFIX_PATH`
2. ROS2 daemon is running: `ros2 daemon status`
3. VS Code was launched with ROS2 environment

**Fix:**
```bash
# Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start

# Relaunch VS Code with ROS2 environment
source /opt/ros/humble/setup.bash
code /home/shane/vscode-tensorfleet
```

### No Image Topics Available

**Check if camera is running:**
```bash
ros2 topic list | grep image
ros2 topic echo /camera/image_raw --no-arr
```

**Start a test camera:**
```bash
# Using usb_cam
sudo apt install ros-humble-usb-cam
ros2 run usb_cam usb_cam_node_exe
```

Or use Gazebo camera:
```bash
# In PX4-Autopilot
make px4_sitl gazebo-classic_iris_fpv_cam
```

### Twist Commands Not Working

**Check cmd_vel topic:**
```bash
ros2 topic list | grep cmd_vel
ros2 topic echo /cmd_vel
```

**Remap if needed:**
If your drone uses a different topic (like `/mavros/setpoint_velocity/cmd_vel_unstamped`), you can remap in the code or use ROS2 topic remapping:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/mavros/setpoint_velocity/cmd_vel
```

### PX4 Not Publishing Topics

**Check MicroXRCE-DDS agent:**
```bash
# Make sure agent is running
MicroXRCEAgent udp4 -p 8888 -v
```

**Check PX4 configuration:**
```bash
# In PX4 console (MAVLink shell)
uxrce_dds_client status
```

---

## Example: Complete Workflow

### Step 1: Start Simulation
```bash
# Terminal 1: PX4 + Gazebo
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris_fpv_cam

# Terminal 2: DDS Agent
MicroXRCEAgent udp4 -p 8888
```

### Step 2: Launch TensorFleet
```bash
# Terminal 3: VS Code with ROS2
source /opt/ros/humble/setup.bash
code /home/shane/vscode-tensorfleet
```

### Step 3: Connect Panels
1. Press F5 to run Extension Development Host
2. Open TensorFleet sidebar
3. Click "Image Panel (Option 3)" → Connect
4. Click "Teleops (Option 3)" → Connect
5. Subscribe to `/camera/image` in Image Panel
6. Use W/A/S/D to control drone in Teleops Panel

### Step 4: Monitor Telemetry
1. `Ctrl+Shift+P` → "TensorFleet: Start PX4 Telemetry Monitor"
2. View real-time telemetry in Output panel

---

## Advanced Configuration

### Custom ROS2 Topics

Edit `src/webviews/option3-panels/src/components/ImagePanel.tsx`:

```typescript
const [topics] = useState([
  '/camera/image_raw',
  '/camera/compressed',
  '/depth/image',
  '/your/custom/topic'  // Add your topic here
]);
```

### Custom Twist Topic

Edit `src/webviews/option3-panels/src/components/TeleopsPanel.tsx`:

```typescript
// Change from /cmd_vel to your topic
vscodeBridge.postMessage({
  command: 'publishTwist',
  topic: '/your/custom/cmd_vel',
  data: twist
});
```

### Connection Auto-Retry

The extension will automatically fall back to simulation if ROS2 is not available. You can manually retry connection:

1. `Ctrl+Shift+P` → "TensorFleet: Connect to ROS2"
2. Wait for initialization
3. Reopen panels

---

## Performance Tips

1. **Reduce Publish Rate**: Lower publish rate in Teleops (10Hz is usually sufficient)
2. **Use Compressed Images**: Subscribe to `/camera/compressed` instead of `/image_raw`
3. **Close Unused Panels**: Close panels when not in use to free resources
4. **Monitor CPU**: Use `htop` to monitor resource usage

---

## Additional Resources

- **ROS2 Documentation**: https://docs.ros.org/en/humble/
- **PX4 Documentation**: https://docs.px4.io/
- **Gazebo Documentation**: https://gazebosim.org/docs
- **rclnodejs GitHub**: https://github.com/RobotWebTools/rclnodejs
- **TensorFleet Extension**: Check `options.md` for implementation details

---

## Support

If you encounter issues:

1. Check this guide first
2. Review console logs: `Ctrl+Shift+I` in VS Code
3. Check Output panel: View → Output → "TensorFleet" or "PX4 Telemetry"
4. Verify ROS2 is working: `ros2 topic list`
5. Check extension logs: Developer → Show Logs

---

## What's Working

✅ ROS2 Connection  
✅ Image Topic Subscription (raw + compressed)  
✅ Twist Command Publishing  
✅ PX4 Telemetry via MAVROS  
✅ Fallback to Simulation  
✅ Topic Discovery  
✅ Connection Management  

## Coming Soon

🚧 Gazebo Integration (launch from extension)  
🚧 Mission Planning UI  
🚧 Multi-drone support  
🚧 Video recording  
🚧 Real-time plotting  

---

**Ready to fly!** 🚁

