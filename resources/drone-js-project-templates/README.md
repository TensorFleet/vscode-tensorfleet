# TensorFleet Drone JS Project

JavaScript/Bun template for PX4 drone control over rosbridge using `roslib` and
`tensorfleet-util`. The current vision demo focus is front-camera YOLO detection.

## Quick Start

1. Start your VM and make sure rosbridge is reachable.
2. Install dependencies:

```bash
bun install
```

3. Open the **Simulation View** and **Image Panel** in the TensorFleet sidebar.
4. Restart the sim before each run:

```bash
bun run restart
```

If you run scripts directly from a shell, make sure the workspace has current
TensorFleet connection metadata in `.tensorfleet` or set `ROSBRIDGE_URL`
explicitly. Otherwise the scripts fall back to `ws://127.0.0.1:9091`.

## VM Assumptions

Use the current default drone VM/world as-is. This first pass does not require
custom MMDS scene components.

The expected ROS image topics are:

- front camera: `/drone_camera/image_raw`
- front annotated output: `/drone_camera/image_annotated`

## Scripts

### Flight / tutorial scripts

- `bun run drone:mover` - draw an R-shaped autonomous mission
- `bun run tutorial:01` to `bun run tutorial:07` - incremental control examples
- `bun run restart` - restart the sim through `/simulation_manager/start_simulation`

### Vision demos

- `bun run vision:yolo` - run YOLO on `/drone_camera/image_raw` and publish annotated frames

## Config

The vision demo settings live in `config/drone_config.yaml` under `vision`:

- `vision.topics` - input and annotated image topics
- `vision.yolo` - model name/path and thresholds

Every key can also be overridden with environment variables such as:

- `IMAGE_TOPIC`
- `YOLO_MODEL_NAME`
- `ROSBRIDGE_URL`

## Notes

- The default YOLO target label remains `dining table`, but when using the
  existing default VM world you may want to override it based on whatever
  object class is actually visible from the drone's front camera.
- These scripts are intended to run with Bun, matching the rest of the template.