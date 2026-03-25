# TensorFleet Drone JS Project

JavaScript/Bun template for PX4 drone control over rosbridge using `roslib` and
`tensorfleet-util`. It currently includes front-camera YOLO detection plus a
colored-pad visual landing demo.

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

Use the current drone VM setup with the existing down camera and the colored
landing pad added to `~/firecracker-vm/assets/opt/gazebo/worlds/e2e_test_world.sdf`.

The expected ROS image topics are:

- front camera: `/drone_camera/image_raw`
- front annotated output: `/drone_camera/image_annotated`
- down camera: `/drone_camera/down/image_raw`
- down annotated output: `/drone_camera/down/image_annotated`

## Scripts

### Flight / tutorial scripts

- `bun run drone:mover` - draw an R-shaped autonomous mission
- `bun run tutorial:01` to `bun run tutorial:07` - incremental control examples
- `bun run restart` - restart the sim through `/simulation_manager/start_simulation`

### Vision demos

- `bun run vision:yolo` - run YOLO on `/drone_camera/image_raw` and publish annotated frames
- `bun run vision:landing` - take off, detect the colored landing pad in the downward camera, center over it, then hand off to `AUTO.LAND`

## Config

The vision demo settings live in `config/drone_config.yaml` under `vision`:

- `vision.topics` - input and annotated image topics
- `vision.yolo` - model name/path and thresholds
- `vision.landing_demo` - pad color, centering thresholds, and descent tuning

Every key can also be overridden with environment variables such as:

- `IMAGE_TOPIC`
- `LANDING_IMAGE_TOPIC`
- `YOLO_MODEL_NAME`
- `LANDING_PAD_COLOR`
- `ROSBRIDGE_URL`

## Notes

- The default YOLO target label remains `dining table`, but when using the
  existing default VM world you may want to override it based on whatever
  object class is actually visible from the drone's front camera.
- The landing demo assumes the VM world contains the cyan/magenta landing pad
  and that `/drone_camera/down/image_raw` is available.
- These scripts are intended to run with Bun, matching the rest of the template.