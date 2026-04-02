# TensorFleet Drone Tutorials

The runnable tutorials live in `src/tutorials/` and are intended to be run in
order after you have a working PX4 + rosbridge setup.

## Core Tutorials

1. `bun run tutorial:01` - connect to rosbridge
2. `bun run tutorial:02` - inspect telemetry topics
3. `bun run tutorial:03` - arm and disarm
4. `bun run tutorial:04` - take off and land
5. `bun run tutorial:05` - OFFBOARD hover
6. `bun run tutorial:06` - OFFBOARD velocity motion
7. `bun run tutorial:07` - OFFBOARD position waypoint navigation

## Vision Demos

These build on the same environment and assume the VM-side camera pipeline is
present:

- `bun run vision:yolo`
- `bun run vision:landing`

## Suggested Run Order

1. `bun run restart`
2. `bun run tutorial:01`
3. `bun run tutorial:04`
4. `bun run tutorial:05`
5. `bun run vision:yolo`
6. `bun run vision:landing`

## Requirements

- rosbridge available on the VM
- PX4 sim running and reset to a known state
- the default drone VM/world with `/drone_camera/image_raw` available
- the landing pad present in `e2e_test_world.sdf` for `vision:landing`
