# 🐛 Debugging Status Bar Items

## Quick Debug Steps

### 1. Open Developer Console

**View the logs to see what's happening:**

1. In VS Code Extension Development Host:

   - Press `Cmd+Shift+P` (Mac) or `Ctrl+Shift+P` (Windows)
   - Type: **"Developer: Toggle Developer Tools"**
   - Press Enter

2. Click the **Console** tab

3. Filter for TensorFleet logs by typing: `TensorFleet` in the filter box

### 2. What to Look For

You should see these log messages:

```
✅ GOOD - Extension activated:
[TensorFleet] Initializing status bar items...
[TensorFleet] ROS version status bar created
[TensorFleet] Drone status bar created
[TensorFleet] Updating status bars...

✅ GOOD - Project detected:
[TensorFleet] Checking for TensorFleet project markers...
[TensorFleet] Workspace folders: ["/path/to/your/project"]
[TensorFleet] ✓ Found marker: config/drone_config.yaml in /path/to/your/project
[TensorFleet] Project detected! Status bars should appear.
[TensorFleet] TensorFleet project detected, showing status bars
[TensorFleet] ROS version set to: ROS 2 Humble
[TensorFleet] Drone status set to: $(radio-tower) 1 Drone
[TensorFleet] ROS version status bar shown: $(archive) ROS 2 Humble
[TensorFleet] Drone status bar shown: $(radio-tower) 1 Drone

❌ BAD - No project detected:
[TensorFleet] Checking for TensorFleet project markers...
[TensorFleet] Workspace folders: ["/path/to/regular/project"]
[TensorFleet] ✗ Missing marker: config/drone_config.yaml in /path/to/regular/project
[TensorFleet] ✗ Missing marker: src/main.py in /path/to/regular/project
[TensorFleet] ✗ Missing marker: missions in /path/to/regular/project
[TensorFleet] No TensorFleet project detected. Status bars will be hidden.
[TensorFleet] Not a TensorFleet project, hiding status bars
```

## Common Issues & Fixes

### Issue 1: No Workspace Open

**Symptoms:**

```
[TensorFleet] No workspace folders open
```

**Fix:**

- Open a folder: `File` → `Open Folder`
- Make sure you have a TensorFleet project open

### Issue 2: Missing Project Markers

**Symptoms:**

```
[TensorFleet] ✗ Missing marker: config/drone_config.yaml
[TensorFleet] ✗ Missing marker: src/main.py
[TensorFleet] ✗ Missing marker: missions
```

**Fix:**
Create a new TensorFleet project:

1. Click **TensorFleet** icon in sidebar
2. Click **"🚀 New Project"**
3. Or manually create these files/folders:
   ```
   mkdir -p config src missions
   touch config/drone_config.yaml
   touch src/main.py
   ```

### Issue 3: Extension Not Activated

**Symptoms:**

- No `[TensorFleet]` logs at all in console

**Fix:**

1. Check extension is running:

   - Press `Cmd+Shift+P`
   - Type: "Extensions: Show Running Extensions"
   - Look for "TensorFleet Drone Suite"

2. If not running, reload:
   - Press `Cmd+Shift+P`
   - Type: "Developer: Reload Window"

### Issue 4: Status Bars Created But Not Visible

**Symptoms:**

```
[TensorFleet] ROS version status bar shown: $(archive) ROS 2 Humble
[TensorFleet] Drone status bar shown: $(radio-tower) 1 Drone
```

But you don't see them in the UI.

**Fix:**

1. **Check the status bar isn't hidden:**

   - View → Appearance → Show Status Bar

2. **Check for other extensions blocking:**

   - Disable other extensions temporarily
   - Reload window

3. **Try clicking on the status bar area:**

   - Sometimes they render but are hard to see
   - Look in the lower RIGHT corner

4. **Check VS Code theme:**
   - Some themes may hide status bar items
   - Try default theme temporarily

### Issue 5: Wrong Folder Open

**Symptoms:**

```
[TensorFleet] Workspace folders: ["/Users/you/wrong/path"]
```

**Fix:**

- Make sure you opened the TensorFleet project folder, not a parent/child folder
- The folder should contain `config/`, `src/`, and `missions/`

## Manual Test Checklist

### ✅ Step-by-Step Verification

1. **Create a test project:**

   ```bash
   mkdir ~/test-tensorfleet
   cd ~/test-tensorfleet
   mkdir -p config src missions
   touch src/main.py
   ```

2. **Create config file:**

   ```bash
   cat > config/drone_config.yaml << 'EOF'
   drone:
     model: "iris"
     id: "drone_1"

   ros2:
     ros_version: "humble"
     namespace: "/drone"
   EOF
   ```

3. **Open in VS Code Extension Development Host:**

   ```bash
   cd /Users/hyper/projects/drone/vscode-tensorfleet
   code --extensionDevelopmentPath="$(pwd)" ~/test-tensorfleet
   ```

4. **Open Developer Console:**

   - `Cmd+Shift+P` → "Developer: Toggle Developer Tools"

5. **Check logs:**

   - Should see "Project detected!" messages

6. **Look at status bar:**
   - Lower right corner
   - Should see ROS version and drone count

## Force Refresh

If status bars aren't updating:

1. **Reload window:**

   - `Cmd+Shift+P` → "Developer: Reload Window"

2. **Manually trigger update:**

   - Edit `config/drone_config.yaml`
   - Save the file
   - Status bars should update

3. **Use commands:**
   - `Cmd+Shift+P` → "TensorFleet: Select ROS Version"
   - `Cmd+Shift+P` → "TensorFleet: Show Drone Status"

## File Structure Check

Your TensorFleet project should look like this:

```
my-project/
├── config/
│   └── drone_config.yaml  ← REQUIRED
├── src/
│   └── main.py           ← REQUIRED
├── missions/             ← REQUIRED (folder)
│   └── example_mission.plan
└── launch/               ← Optional
    └── drone_sim.launch.py
```

**At minimum, you need:**

- `config/drone_config.yaml` (file)
- `src/main.py` (file)
- `missions/` (folder, can be empty)

## Verify Status Bar Items Exist

Run this in the Developer Console:

```javascript
// Check if status bar items were created
console.log("Status bars registered:", vscode.window.statusBarItems);
```

## Reset Everything

If all else fails:

1. **Close VS Code**

2. **Recompile extension:**

   ```bash
   cd /Users/hyper/projects/drone/vscode-tensorfleet
   rm -rf out/
   bun run compile
   ```

3. **Restart in clean state:**

   ```bash
   code --extensionDevelopmentPath="$(pwd)" --disable-extensions ~/test-tensorfleet
   ```

4. **Check console immediately:**
   - Look for initialization messages
   - Should see status bar creation

## Report What You See

When asking for help, provide:

1. **Console logs** (filter for `TensorFleet`)
2. **Folder structure:**
   ```bash
   ls -la config/ src/ missions/
   ```
3. **Config file contents:**
   ```bash
   cat config/drone_config.yaml
   ```
4. **VS Code version:**
   - Help → About

## Expected Behavior

**When working correctly:**

1. Extension activates
2. Checks workspace for markers
3. Finds at least one marker file
4. Shows status bars in lower right
5. Updates every 5 seconds
6. Responds to config file changes

**Timeline:**

- 0ms: Extension activates
- 10ms: Status bars created
- 50ms: Project detection runs
- 100ms: Status bars appear (if project detected)
- 5000ms: First auto-update
- Every 5s: Continues updating

## Still Not Working?

If after all these steps you still don't see the status bars:

1. **Share the console output** - Copy all `[TensorFleet]` logs
2. **Share folder structure** - Run `tree -L 2` or `ls -R`
3. **Screenshot** - Show the status bar area
4. **VS Code version** - Might be a compatibility issue

---

The debug logs should help identify exactly where the detection is failing! 🔍
