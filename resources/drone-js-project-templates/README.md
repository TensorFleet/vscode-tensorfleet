# TensorFleet Robotic JS Project

JavaScript/Node.js template for robot control over rosbridge using `roslib`. Includes obstacle avoidance and vision examples.

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
3. Open **Simulation View** and **Image Panel** from the sidebar.
4. Run any of the example scripts:
   - `bun robot:mover` - Drive forward, backward, turn left/right sequence
   - `bun robot:obstacle` - LiDAR-based obstacle avoidance
   - `bun robot:vision` - YOLO object detection on camera feed
   - `bun robot:vision:colors` - Color-based detection (best for simulation)

## Scripts

### Simulation scripts
- `src/restart_sim.js` - Restarts the simulation

### Movement Scripts
- `src/drone_mover.js` - Makes the drone Take off, follow an "R" flight pattern and then land.

### Vision Scripts

**Setup:**
1. Open **Map Panel** from the sidebar
2. Open **Simulation Panel** to view the drone's movement in 3D view.