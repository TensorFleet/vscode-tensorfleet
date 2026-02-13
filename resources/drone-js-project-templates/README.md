# TensorFleet Drone JS Project

JavaScript/Node.js template for drone control over rosbridge using `roslib`.

## Quick Start

1. **Start your VM**: Click the **TensorFleet** status bar at the bottom of VS Code and select **Start VM**.
2. **Install runtime and dependencies**:

   **Recommended: [Bun](https://bun.sh)** (v1.0.0+)
   ```bash
   # Install Bun (macOS, Linux, WSL)
   curl -fsSL https://bun.sh/install | bash
   
   # Install dependencies
   bun install
   ```

   **Alternative: Node.js** (v14.0.0+)
   - This project uses modern JavaScript syntax (optional chaining `?.`)
   - If you see `SyntaxError: Unexpected token '.'`, upgrade Node.js to v14+ or switch to Bun
   - Run `npm run check` to verify your runtime compatibility
   ```bash
   npm install
   ```
3. Open **Simulation View** from the sidebar.
4. Run any of the example scripts:
   - `bun tutorial:01` - Connect to rosbridge
   - `bun tutorial:02` - Monitor MAVROS telemetry
   - `bun tutorial:03` - Arm/disarm
   - `bun tutorial:04` - Takeoff and land
   - `bun tutorial:05` - Offboard hover
   - `bun tutorial:06` - Move forward
   - `bun tutorial:07` - Go to waypoint

## MAVLink Tunnel (Option A)

Use the VSCode extension commands for local USB serial to VM MAVLink tunneling. This bridge supports bidirectional binary MAVLink frames.

1. Run **TensorFleet: Drone Setup** in VSCode.
2. Pick one action:
   - **Set Drone Mode**
   - **Use REAL Mode**
   - **Use SITL Mode**
   - **Connect/Disconnect Real Telemetry** (changes based on tunnel state)
3. Configure tunnel once in project `.env`:
   - `TENSORFLEET_MAVLINK_SERIAL_PATH=/dev/ttyACM0`
   - `TENSORFLEET_MAVLINK_BAUD_RATE=115200`
4. After tunnel connection, run telemetry/tutorial scripts normally (for example `bun tutorial:02`).
   - `02_telemetry.js` now includes examples for monitoring **IMU data** (Linear Acceleration and Gyro) alongside standard MAVROS state.
5. When done, use **Connect/Disconnect Real Telemetry** again to release serial + websocket cleanly.

### Diagnostic Tools
- `scripts/mavlink_tunnel_probe.js` is available for diagnostics. 
- Use `--require-ws-rx` to verify that round-trip traffic is active (not just outbound).

Notes:
- No separate tunnel terminal is required for normal operation.
- The script/env based tunnel auto-connect flow (Option B) is not used.

## Scripts

### Simulation scripts
- `src/restart_sim.js` - Restarts the simulation
