# TensorFleet Drone JS Project

JavaScript/Node.js template for drone control over rosbridge using `roslib`. Includes an OFFBOARD velocity guided mover driven by a mission plan.

## Quick start
1) Install deps: `bun install` (or `npm install`)
2) Point to rosbridge (default `ws://172.16.0.10:9091`): `export ROSBRIDGE_URL=ws://<vm-ip>:9091`
3) Start PX4 + MAVROS + rosbridge in your VM, then run:
   - `bun run restart` - Restart the simulation (resets drone state)
   - `bun src/drone_mover.js` - ARM → TAKEOFF → OFFBOARD waypoint mission → LAND
     - Reads waypoints from `missions/example_mission.plan` (5m square pattern)
     - Falls back to 8-point circle if mission plan is missing


## Scripts
- `src/restart_sim.js`: Restart the PX4 simulation via `/simulation_manager/start_simulation` service. Useful for resetting drone state between test runs.
- `src/drone_mover.js`: Complete autonomous mission: ARM → TAKEOFF → OFFBOARD → fly waypoints from `missions/example_mission.plan` → return home → LAND. Falls back to 8-point circle pattern if mission plan is missing. Uses env or `config/drone_config.yaml` (`offboard` section) for tuning.
- `src/vision_yolo.js`: Subscribes to `/drone_camera/image_raw`, runs YOLOv8 ONNX via `onnxruntime-node`, and republishes annotated images to `/drone_camera/image_annotated`. Uses env or `config/drone_config.yaml` (`vision` section) for model selection and topic names.

## Tutorials

Learn drone control step-by-step with focused examples:

**Beginner** (getting started):
1. `bun run tutorial:01` - Connect to rosbridge and read drone state
2. `bun run tutorial:02` - Display all telemetry (position, GPS, battery)
3. `bun run tutorial:03` - Send ARM command

**Intermediate** (basic flight):
4. `bun run tutorial:04` - Takeoff to altitude
5. `bun run tutorial:05` - Land the drone
6. `bun run tutorial:06` - Enter OFFBOARD mode and hover

**Advanced** (autonomous navigation):
7. `bun run tutorial:07` - Move forward using velocity control
8. `bun run tutorial:08` - Navigate to waypoint with position feedback

Each tutorial is ~50-100 lines and demonstrates one concept clearly. See `src/tutorials/` for source code.

## Configuration
Edit `config/drone_config.yaml` or override via env vars:
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
|   |-- vision_yolo.js      # Computer vision example
|   `-- restart_sim.js      # Simulation restart utility
|-- config/                 # Network + flight config
|-- missions/               # Example mission plans
|-- launch/                 # Optional ROS 2 launch files
|-- package.json
`-- README.md
```

## Tips
- **New to drone control?** Start with the tutorials (`bun run tutorial:01` through `tutorial:08`)
- rosbridge runs in the VM; no local ROS 2 binaries needed
- Use `bun run restart` to reset simulation between test runs
- Verify connectivity with `src/drone_mover.js` for full autonomous mission
