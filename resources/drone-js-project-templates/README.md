# TensorFleet Drone JS Project

JavaScript/Node.js template for drone control over rosbridge using `roslib`. Includes an OFFBOARD velocity guided mover driven by a mission plan.

## Requirements

**Recommended: [Bun](https://bun.sh)** (v1.0.0+)
```bash
# Install Bun (macOS, Linux, WSL)
curl -fsSL https://bun.sh/install | bash
```

**Alternative: Node.js** (v14.0.0+)
- This project uses modern JavaScript syntax (optional chaining `?.`)
- If you see `SyntaxError: Unexpected token '.'`, upgrade Node.js to v14+ or switch to Bun
- Run `npm run check` to verify your runtime compatibility


## Quick start
1) Install deps: `bun install` (or `npm install`)
2) **Check compatibility**: `bun run check` (verifies your runtime supports modern JavaScript)
3) Open `Simulation view` and `Map View`. 
4) Start your VM, then run:
   - `bun run restart` - Restart the simulation (resets drone state)
   - `bun drone:mover` - ARM → TAKEOFF → OFFBOARD waypoint mission → LAND


## Scripts
- **`check-runtime.js`**: Runtime compatibility checker. Verifies your JavaScript runtime (Bun or Node.js) supports modern syntax features like optional chaining. Run `bun run check` or `npm run check` if you encounter syntax errors. Provides specific troubleshooting steps if issues are detected.
- `src/restart_sim.js`: Restart the PX4 simulation via `/simulation_manager/start_simulation` service. Useful for resetting drone state between test runs.
- `src/drone_mover.js`: Complete autonomous mission: ARM → TAKEOFF → OFFBOARD → fly waypoints from `missions/example_mission.plan` → return home → LAND. Falls back to 8-point circle pattern if mission plan is missing. Uses env or `config/drone_config.yaml` (`offboard` section) for tuning.


## Tutorials

Learn drone control step-by-step with focused examples:

**Beginner** (getting started):
1. `bun run tutorial:01` - Connect to rosbridge and read drone state
2. `bun run tutorial:02` - Display all telemetry (position, GPS, battery)
3. `bun run tutorial:03` - Send ARM/DISARM command

**Intermediate** (basic flight):
4. `bun run tutorial:04` - Takeoff to altitude and land (complete flight cycle)
5. `bun run tutorial:05` - Enter OFFBOARD mode and hover

**Advanced** (autonomous navigation):
6. `bun run tutorial:06` - Move forward using velocity control
7. `bun run tutorial:07` - Navigate to waypoint with position feedback

Each tutorial is ~50-100 lines and demonstrates one concept clearly. See `src/tutorials/` for source code.

## Configuration
Edit `config/drone_config.yaml` or override via env vars:
- `TENSORFLEET_BASE_URL` + `TENSORFLEET_JWT` - primary TensorFleet connection (other URLs are derived automatically when blank)
- `ROSBRIDGE_URL` - rosbridge WebSocket URL
- `SETPOINT_FRAME_ID` - frame for setpoints (default `map`)
- `ALT_TARGET`, `EDGE_M`, `V_FAST`, `V_MIN`, `WAYPOINT_RADIUS`, `SLOW_RADIUS`, `SETPOINT_HZ`, `R2B_HOST`, `R2B_PORT` - OFFBOARD tuning for `drone_mover.js` (defaults favor a small world: ~1m alt, ~5m hop, gentle velocities)
- `MISSION_PLAN_PATH` - override the plan file used by `drone_mover.js` (defaults to `missions/example_mission.plan`)
The `offboard` section in the YAML mirrors the environment overrides for `drone_mover.js`.

## Layout
```
.
|-- src/
|   |-- tutorials/          # Step-by-step learning scripts
|   |-- lib/                # Shared utilities
|   |-- drone_mover.js      # Advanced mission example
|   `-- restart_sim.js      # Simulation restart utility
|-- config/                 # Network + flight config
|-- missions/               # Example mission plans
|-- launch/                 # Optional ROS 2 launch files
|-- package.json
`-- README.md
```

## Tips
- **New to drone control?** Start with the tutorials (`bun run tutorial:01` through `tutorial:07`)
- rosbridge runs in the VM; no local ROS 2 binaries needed
- Use `bun run restart` to reset simulation between test runs
- Verify connectivity with `src/drone_mover.js` for full autonomous mission

## Troubleshooting

### `SyntaxError: Unexpected token '.'`
This error means you're using an older Node.js version that doesn't support optional chaining (`?.`).

**Solutions:**
1. **Switch to Bun** (recommended):
   ```bash
   curl -fsSL https://bun.sh/install | bash
   bun install
   bun run restart
   ```

2. **Upgrade Node.js** to v14.0.0 or higher:
   ```bash
   # Check your version
   node --version
   
   # Upgrade via nvm (recommended)
   nvm install 18
   nvm use 18
   ```

3. **Check compatibility**:
   ```bash
   npm run check
   ```

### "I'm using Bun but still getting the error!"
If you run `bun run restart` but still see `SyntaxError: Unexpected token '.'`, **Node.js is actually running your script**, not Bun.

**How to verify:**
```bash
# This should show Bun version
bun --version

# Run the check script
npm run check
# or
bun run check
```

**Common causes:**
1. **Bun not in PATH**: Bun is installed but your shell can't find it
   ```bash
   # Add to ~/.bashrc or ~/.zshrc
   export PATH="$HOME/.bun/bin:$PATH"
   source ~/.bashrc  # or ~/.zshrc
   ```

2. **Using npm instead of bun**: Make sure you're using `bun` commands
   ```bash
   # ❌ Wrong (uses Node.js)
   npm run restart
   
   # ✅ Correct (uses Bun)
   bun run restart
   ```

3. **Shebang issues**: If running scripts directly (e.g., `./src/restart_sim.js`), ensure they have the correct shebang
   - Scripts should start with `#!/usr/bin/env -S bun run`
   - Make them executable: `chmod +x src/restart_sim.js`

**Quick fix:**
```bash
# Verify Bun is installed and working
which bun
bun --version

# If not found, reinstall Bun
curl -fsSL https://bun.sh/install | bash

# Restart your terminal, then try again
bun run restart
```

### Connection Issues
- Ensure your VM is running and connected
- Check that `TENSORFLEET_VM_MANAGER_URL`, `TENSORFLEET_NODE_ID`, and `TENSORFLEET_JWT` are set in `.env`
- Verify rosbridge is accessible (default port: 9091)
