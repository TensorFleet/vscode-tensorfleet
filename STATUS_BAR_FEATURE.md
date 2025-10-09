# Status Bar Items for TensorFleet Projects

## Overview

When you open a TensorFleet drone project, two interactive status bar items automatically appear in the lower right corner of VS Code:

1. **ROS Version Selector** - Shows and changes the ROS version
2. **Drone Status Monitor** - Shows number of drones and their status

## Features

### 🔧 ROS Version Selector

**Display:** `$(archive) ROS 2 Humble`

**Click to:**

- View all available ROS versions
- Switch between ROS 2 (Humble, Iron, Jazzy, Rolling) and ROS 1 (Noetic)
- Automatically update `drone_config.yaml` with selected version
- Get sourcing instructions for the selected version

**How it works:**

1. Reads `ros_version` from `config/drone_config.yaml`
2. Displays current version in status bar
3. Click to open dropdown menu
4. Select new version
5. Optionally updates config file
6. Shows command to source the ROS environment

### 📡 Drone Status Monitor

**Display:** `$(radio-tower) 1 Drone` or `$(radio-tower) 2 Drones (1 Flying)`

**Click to:**

- View detailed info for each drone (ID, model, battery, mode)
- Refresh drone status
- Start Gazebo simulation
- Open individual drone details

**How it works:**

1. Reads drone configuration from `config/drone_config.yaml`
2. Parses drone ID, model, and settings
3. Displays number of active drones
4. Updates every 5 seconds
5. Click to see detailed dropdown menu with:
   - Drone name and status icon
   - Flight mode and battery level
   - Quick actions (refresh, start sim)

## Auto-Detection

The status bar items **only appear** when you have a TensorFleet project open. Detection checks for:

- ✅ `config/drone_config.yaml`
- ✅ `src/main.py`
- ✅ `missions/` folder
- ✅ `launch/` folder

If these files exist, you have a TensorFleet project and the status bars will automatically appear!

## Status Bar Locations

```
┌────────────────────────────────────────────────────────┐
│  VS Code Window                                        │
│                                                        │
│  [Your code here]                                      │
│                                                        │
└────────────────────────────────────────────────────────┘
  Status Bar: [$(archive) ROS 2 Humble] [$(radio-tower) 1 Drone]
              ↑                          ↑
              ROS Version               Drone Status
```

## Usage Examples

### Switching ROS Versions

1. **Click** on ROS version in status bar (e.g., "ROS 2 Humble")
2. **Select** from dropdown:
   - ROS 2 Humble
   - ROS 2 Iron
   - ROS 2 Jazzy
   - ROS 2 Rolling
   - ROS 1 Noetic
3. **Choose** whether to update `drone_config.yaml`
4. **Copy** the source command: `source /opt/ros/humble/setup.bash`

### Checking Drone Status

1. **Click** on drone status (e.g., "1 Drone")
2. **View** dropdown showing:

   ```
   $(circle-outline) iris
   MANUAL | $(battery-full) 100%
   ID: drone_1 | Status: idle

   $(refresh) Refresh Status
   $(debug-start) Start Simulation
   ```

3. **Click** on a drone for detailed info
4. **Click** "Start Simulation" to launch Gazebo

### Status Icons

**Drone Status:**

- `$(rocket)` - Flying
- `$(zap)` - Armed
- `$(circle-outline)` - Idle
- `$(circle-slash)` - Offline

**Battery Level:**

- `$(battery-full)` - >75%
- `$(battery)` - >50%
- `$(battery-charging)` - >25%
- `$(battery-empty)` - <25%

## Configuration

### Setting ROS Version in Config

Edit `config/drone_config.yaml`:

```yaml
ros2:
  ros_version: "humble" # Options: humble, iron, jazzy, rolling
  namespace: "/drone"
  domain_id: 0
```

The status bar will automatically detect and display this version.

### Multiple Drones

To configure multiple drones (future feature):

```yaml
drones:
  - id: "drone_1"
    model: "iris"
  - id: "drone_2"
    model: "typhoon_h480"
```

Status bar will show: `$(radio-tower) 2 Drones`

## Commands

These commands are also available from the Command Palette (`Cmd+Shift+P`):

- `TensorFleet: Select ROS Version` - Open ROS version selector
- `TensorFleet: Show Drone Status` - Open drone status menu

## Auto-Update

**ROS Version:**

- Reads from config on project open
- Updates when config file changes
- Persists selection across sessions

**Drone Status:**

- Updates every 5 seconds automatically
- Reads latest config changes
- Real-time battery and status updates (when integrated with ROS)

## Integration with Other Features

### Works With:

- ✅ **Project Scaffolding** - Auto-detects new projects
- ✅ **Gazebo Panel** - "Start Simulation" button
- ✅ **ROS 2 Panel** - Shows appropriate ROS version
- ✅ **Config Files** - Reads and writes `drone_config.yaml`

### Future Integration:

- 🔄 **MCP Server** - Query real-time drone telemetry
- 🔄 **ROS Topics** - Live battery and position data
- 🔄 **Multi-Drone** - Support for drone swarms

## Technical Details

### Project Detection Logic

```typescript
// Checks for these markers:
const markers = [
  "config/drone_config.yaml", // Config file
  "src/main.py", // Main controller
  "missions", // Mission folder
  "launch", // Launch folder
];
```

If **any** marker is found, the project is detected as TensorFleet.

### Status Update Frequency

- **ROS Version:** On config change (file watcher)
- **Drone Status:** Every 5 seconds (interval)
- **Project Detection:** On workspace folder change

### File Watching

The extension watches these patterns:

- `**/config/drone_config.yaml` - For ROS version and drone config

When these files change, status bars update automatically!

## Troubleshooting

### Status Bars Don't Appear

**Problem:** Status bars are not visible

**Solutions:**

1. Check you have a TensorFleet project open (look for `config/drone_config.yaml`)
2. Create a new project with "🚀 New Project" button
3. Reload window: `Cmd+Shift+P` → "Developer: Reload Window"

### ROS Version Not Detected

**Problem:** Status bar shows wrong ROS version

**Solutions:**

1. Add `ros_version` to `config/drone_config.yaml`:
   ```yaml
   ros2:
     ros_version: "humble"
   ```
2. Click status bar and manually select version
3. Choose "Yes" when asked to update config

### Drone Status Shows "0 Drones"

**Problem:** No drones detected

**Solutions:**

1. Check `config/drone_config.yaml` exists
2. Verify drone `id` and `model` are set in config
3. Click "Start Simulation" to launch Gazebo
4. Click "Refresh Status" in drone dropdown

## Examples

### Complete Workflow

1. **Create Project:**

   ```
   Click "🚀 New Project" → Enter "my-drone" → Select location
   ```

2. **Status bars appear automatically:**

   ```
   [$(archive) ROS 2 Humble] [$(radio-tower) 1 Drone]
   ```

3. **Switch ROS version:**

   ```
   Click "ROS 2 Humble" → Select "ROS 2 Iron" → Update config
   ```

4. **Check drone status:**

   ```
   Click "1 Drone" → View "iris" details → Start Simulation
   ```

5. **During flight:**
   ```
   Status updates to: [$(radio-tower) 1 Drone (1 Flying)]
   Battery drains: [$(battery) 85%]
   ```

## Benefits

✅ **Quick Access** - No need to check config files manually
✅ **Visual Feedback** - See ROS version and drone count at a glance
✅ **Interactive** - Click to change settings or view details
✅ **Auto-Detection** - Works automatically with TensorFleet projects
✅ **Context-Aware** - Only appears when relevant
✅ **Real-Time** - Updates automatically every 5 seconds

---

The status bar items provide a seamless way to monitor and control your TensorFleet drone projects without leaving your code editor! 🚁✨
