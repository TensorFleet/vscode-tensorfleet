# Vacuum Stack Plan

## Purpose

This document is the planning source of truth for the vacuum stack.

It covers both:

- the long-term product and architecture direction; and
- the current TurtleBot4-based simulation and VM integration reality.

## Goal

Build the software stack for a future robot vacuum before buying hardware.

The near-term goal is:

- a simulated robot appears in our system;
- it is controllable through ROS 2;
- it can navigate in simulation;
- the VS Code extension and related panels are useful operator surfaces;
- we avoid coupling product behavior to TurtleBot4-specific APIs.

The longer-term goal is:

- a vacuum-facing contract exists above the robot backend;
- docking, pause/resume, room cleaning, battery-aware execution, and mission
  state fit naturally into that contract;
- the same higher-level product logic can later move from TurtleBot4
  simulation to a real vacuum backend with minimal churn.

TurtleBot4 is not the product. It is the first development backend.

## Product Boundary

TurtleBot4 is the first development backend.

The product-facing boundary is a repo-owned vacuum contract.

Backend differences must surface through:

- capabilities;
- normalized state; and
- explicit unsupported operations.

No client above the adapter may depend on TurtleBot4-specific topics, helper
services, or node APIs. TurtleBot4-specific helpers such as
`TurtleBot4Navigator` are useful backend implementation details because they
add docking and undocking behavior on top of Nav2, but they must stay private
to the TurtleBot4 backend and must not define the public contract. See the
current TurtleBot4 docs for navigation and `TurtleBot4Navigator` behavior:
[Navigation](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html),
[TurtleBot 4 Navigator](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html).

## Planning Decisions

These are the current working decisions unless we find a concrete reason to
change them:

- ROS 2 is the foundation.
- TurtleBot4 is the first backend, not the public product contract.
- The public boundary should be a repo-owned vacuum-facing interface, not raw
  TurtleBot4 topics.
- The first real-hardware path should target Valetudo-compatible vacuums.
- The contract should reuse standard ROS 2 and Nav2 interfaces where they
  already fit.
- Simulation should be realistic enough to validate workflow, not just expose
  `cmd_vel`.
- Simulation should become the regression harness for later hardware work.
- The system should be local-first.

## Success Criteria

### First usable vertical slice

- one simulated robot appears in our system;
- the robot boots reliably in the VM;
- pose, map, and camera are visible;
- a ROS client can send a navigation goal and observe progress and result;
- the system uses standard ROS 2 / Nav2 interfaces where they fit;
- the path toward a later vacuum-specific abstraction stays clean.

### Full platform success

- the same client-facing vacuum contract works against TurtleBot4 simulation
  and a real vacuum backend;
- backend differences are expressed through capability flags and explicit
  unsupported operations, not product forks;
- docking, mission lifecycle, battery state, and room/zone workflows fit the
  same contract;
- simulation acts as the regression harness for real hardware work.

## Non-Goals For The First Milestone

- perfect simulation of brushes, dust bin, water tank, or consumables;
- support for every vacuum vendor;
- exact reproduction of a commercial vacuum UI;
- hardware procurement decisions;
- forcing OpenClaw to be the foundation before the robot stack is stable.

## Why TurtleBot4

TurtleBot4 is a good stand-in because it already gives us most of the surfaces
needed to validate the robotics side of the platform:

- differential-drive motion;
- lidar, IMU, odometry, and camera streams;
- hazard and contact-related signals;
- docking-related topics;
- strong ROS 2 compatibility;
- a simulation path we already control inside this repo and VM environment.

TurtleBot4 is still not a vacuum. It does not natively model:

- cleaning coverage;
- room and zone semantics;
- suction or water modes;
- consumables or maintenance state;
- vendor-specific charging, mapping, and docking behavior.

Those concepts should be introduced in repo-owned layers above the robot
backend, not leaked directly into UI or product logic.

## Product And Operator Positioning

The system should first work as a deterministic, local ROS 2 vacuum platform.

Current positioning:

- the VS Code extension should be treated as an important part of the platform,
  not just a debugging extra;
- the VS Code extension and related panels should be the main first-mile UI
  for bringup and validation;
- TurtleBot4 should be treated as the development harness;
- OpenClaw may become useful later, but should sit above a stable
  vacuum-facing contract rather than directly on TurtleBot4 internals.

Current extension truth for the active Layer 2 slice:

- `Vacuum Control` is the main operator surface now being hardened
- the panel is map-first and driven by the extracted shared Nav2 runtime seam
- the dedicated `MapCanvas` surface now renders live occupancy-grid data from
  `/map` and tolerates Foxglove typed-array payload shapes

Practical implication:

- first prove the stack through plain ROS clients and the current panels;
- only then add higher-level agent or workflow integration on top.

## Role Of The VS Code Extension

The existing VS Code extension is part of the platform strategy.

During the first milestones, it should be the main operator and developer
surface because it already gives us fast feedback on robot and simulation
state.

The extension should be the main place where we validate:

- whether the robot is present and healthy;
- whether lidar, camera, 3D, and simulation views are actually useful;
- whether navigation behavior is understandable enough to debug;
- whether real-time state is visible enough to support development;
- whether the vacuum-facing contract exposes the right product information.

The important existing surfaces are:

- lidar panels;
- camera panels;
- 3D panels;
- Gazebo web or simulation views;
- any robot/entity discovery and state surfaces already present in the
  extension.

Current truth:

- the existing TurtleBot4-facing panels are primarily debugging and validation
  surfaces;
- they are useful for proving runtime health, TF, Nav2 traffic, and motion;
- they do not yet provide the intended Layer 2 navigation operator workflow.

Near-term implication:

- the next extension-side Layer 2 slice should be a dedicated TurtleBot4
  navigation operator panel;
- that panel should be map-first, goal-oriented, and separate from the current
  debug surfaces.

As the stack matures, the extension should also be a natural place for a small
set of vacuum-specific controls and status surfaces, such as:

- start mission;
- pause, resume, and stop;
- return to dock;
- room or zone selection;
- mission state;
- battery and charging status;
- fault and recovery state.

## Architecture Direction

The intended stack is:

```text
Operator / developer surfaces
  -> ROS bridge / ROS-native integration
  -> vacuum_adapter contract
  -> backend implementation
     -> TurtleBot4 + Nav2 + simulation realism layer
     -> later: Valetudo-compatible real vacuum
```

The important boundary is `vacuum_adapter`.

That should become the stable surface that higher-level clients use.
TurtleBot4 and later vacuum-vendor specifics should remain implementation
details behind it.

## Capability Model

The vacuum contract must be capability-driven from the start.

Valetudo explicitly models robots as different subsets and supersets of
capabilities, and not every robot supports the same commands, status surfaces,
or map workflows. See
[Capabilities overview](https://valetudo.cloud/pages/usage/capabilities-overview.html)
and [MQTT](https://valetudo.cloud/pages/integrations/mqtt.html).

### Core capability tiers

**Tier 1: Required on every backend**

- robot identity
- availability / connectivity
- pose
- map access
- battery state
- mission status
- navigation-to-pose or equivalent move command

**Tier 2: Common vacuum controls**

- return to dock
- pause mission
- resume mission
- stop mission
- dock state
- charging state

**Tier 3: Advanced vacuum workflows**

- room / segment cleaning
- zone cleaning
- cleaning mode selection
- consumable state
- maintenance state

**Tier 4: Vendor-specific extensions**

- any backend-specific feature not portable enough for the shared contract

### Capability design rules

- every backend must advertise capability flags explicitly;
- clients must branch on capability flags, not backend names;
- capability presence must be validated against actual behavior;
- reconnect or backend reconfiguration may require capability refresh;
- vendor-specific features may exist, but they must not silently redefine the
  shared contract.

## Vacuum Contract

The public contract should describe a vacuum robot, not a TurtleBot4 and not a
specific vacuum vendor.

The contract must be:

- capability-based;
- stable across backends;
- built on standard ROS 2 / Nav2 interfaces where they already fit;
- extended only where vacuum-specific semantics are required.

### Core normalized surfaces

Every backend should provide the following normalized surfaces:

- robot identity
- connectivity / availability
- pose
- odometry
- map
- battery state
- mission state
- fault state
- capability flags

Where standard ROS 2 / Nav2 types already fit, they should be reused directly.
Examples include:

- `nav2_msgs/action/NavigateToPose`
- `sensor_msgs/msg/BatteryState`
- `nav_msgs/msg/OccupancyGrid`
- `nav_msgs/msg/Odometry`
- `geometry_msgs/msg/PoseStamped`
- `sensor_msgs/msg/Image` when a backend meaningfully exposes a camera feed

### Vacuum-specific normalized surfaces

Custom messages or services should exist only for concepts that are not
cleanly expressed through standard ROS 2 or Nav2 types. Likely examples
include:

- dock state
- mission lifecycle state
- pause / resume / stop mission controls
- room / segment cleaning requests
- zone cleaning requests
- backend capability advertisement

### Unsupported operations

If a backend does not support a capability:

- it must advertise that capability as unavailable;
- the unsupported command must fail explicitly and predictably;
- clients must not infer support from backend type;
- higher-level workflows must branch on capability flags, not backend name.
