# TensorFleet Drone JS Project

JavaScript/Node.js template for drone control over rosbridge using `roslib`. Includes an OFFBOARD velocity guided mover driven by a mission plan.

## Quick start
1) Install deps: `bun install` (or `npm install`)
2) Point to rosbridge (default `ws://172.16.0.10:9091`): `export ROSBRIDGE_URL=ws://<vm-ip>:9091`
3) Start PX4 + MAVROS + rosbridge in your VM, then run:
   - `bun src/drone_mover.js` - AUTO.TAKEOFF -> OFFBOARD velocity legs from `missions/example_mission.plan` (or small hop fallback) -> return home -> AUTO.LAND

## Scripts
- `src/drone_mover.js`: Ports the Python guided mission. Arms, takes off to a low `ALT_TARGET`, enters OFFBOARD, flies waypoints from `missions/example_mission.plan` (lat/lon -> local ENU offsets from home) when present, then returns home and lands. Uses env or `config/drone_config.yaml` (`offboard` section) for tuning; falls back to a tiny hop/return if the plan is missing.

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
|-- src/                    # Drone control scripts
|-- config/                 # Network + flight config
|-- missions/               # Example QGC plans
|-- launch/                 # Optional ROS 2 launch files
|-- package.json
`-- README.md
```

## Tips
- rosbridge runs in the VM; no local ROS 2 binaries needed.
- Verify connectivity with `src/drone_mover.js` for OFFBOARD control.
