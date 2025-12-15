# Tutorial


## Preparation

In these tutorials you can observe the state of the drone using our Simulation views or the map panel.
For basic functionality and monitoring the map panel is more useful.
We can also arm/disarm the drone and perform mission control operations using the map panel.

For consistent results you will need to restart the simulation.
For debugging your telemtry scripts you will need to use other ways to change the drone state to observe changes. You can do that with a basic takeoff/land in the map panel.


For the start login to your tensorfleet account, Start your VM and open the Map panel.
Then press "Restart simulation" from the right side to restart the simulation. Wait 10-15 seconds for it to finish. Do this every time for any script run (if it's not already included in the code)


## Connection

The connection tutorial demonstrates how to establish a connection to the drone and monitor its state. It showcases both direct ROS topic subscriptions and the use of a managed state model.

### Running the Tutorial

First we will go through the basics of connecting to the VM through javascript.
There are many ways to connect to a drone. The most basic ways are with a ROS Bridge or directly via ROS.

For the start we will use ROS Bridge for basic operations. The advantage of ROS Bridge is that it's suitable for websocket connectiosn which are easier to establish remotely.
We already have utilities that connect you to the drone's environment using an authenticated proxy and the "roslib" javascript library.

Here we have an example of how to connect to a drone and conduct basic telemetry of the states (armed, flight mode, etc.)

To run the 01_connect tutorial, use the following bun command:

```bash
bun run src/tutorials/01_connect.js
```

This will start the tutorial script, which connects to the drone via MAVROS and begins monitoring state changes.

### Expected Output Example.

When running the tutorial, you should see output similar to the following (this is the result of a successful connection and state monitoring and only prints again when changes occur):

```
[INFO] Listening to drone state...

[INFO] Showing both raw ROS subscription via wrapper and managed DroneStateModel

[DEBUG] DroneStateModel.connect() called
[DEBUG] Subscribing to topics: [
  {
    topic: "/mavros/global_position/raw/fix",
    type: "sensor_msgs/msg/NavSatFix",
  }, {
    topic: "/mavros/global_position/compass_hdg",
    type: "std_msgs/msg/Float64",
  }, {
    topic: "/mavros/state",
    type: "mavros_msgs/msg/State",
  }, {
    topic: "/mavros/extended_state",
    type: "mavros_msgs/msg/ExtendedState",
  }, {
    topic: "/mavros/battery",
    type: "sensor_msgs/msg/BatteryState",
  }, {
    topic: "/mavros/vfr_hud",
    type: "mavros_msgs/msg/VFR_HUD",
  }, {
    topic: "/mavros/local_position/pose",
    type: "geometry_msgs/msg/PoseStamped",
  }, {
    topic: "/mavros/local_position/velocity_local",
    type: "geometry_msgs/msg/TwistStamped",
  }, {
    topic: "/mavros/imu/data",
    type: "sensor_msgs/msg/Imu",
  }, {
    topic: "/mavros/altitude",
    type: "mavros_msgs/msg/Altitude",
  }, {
    topic: "/mavros/home_position/home",
    type: "mavros_msgs/msg/HomePosition",
  }
]
[DEBUG] DroneStateModel.connect() completed
Press Ctrl+C to exit

=== RAW ROS SUBSCRIPTION RECEIVED CHANGED DATA ===
Drone State:
  Connected: true
  Armed:     false
  Mode:      AUTO.LOITER
  Guided:    true

=== MANAGED DRONE STATE MODEL UPDATE ===
vehicle :
 {
  time_boot_ms: 1765832026405,
  connected: true,
  armed: false,
  guided: true,
  manual_input: false,
  mode: "AUTO.LOITER",
  system_status: 3,
}
status:
 {
  time_boot_ms: 1765832026439,
  connected: true,
  gcs_link: true,
  faults: [],
  armable: true,
  arm_reasons: [],
}
=== MANAGED DRONE STATE MODEL UPDATE ===
vehicle :
 {
  time_boot_ms: 1765832028302,
  connected: true,
  armed: false,
  guided: true,
  manual_input: false,
  mode: "AUTO.LOITER",
  system_status: 3,
}
status:
 {
  time_boot_ms: 1765832028358,
  connected: true,
  gcs_link: true,
  faults: [],
  armable: true,
  arm_reasons: [],
}
=== RAW ROS SUBSCRIPTION RECEIVED CHANGED DATA ===
Drone State:
  Connected: true
  Armed:     true
  Mode:      AUTO.TAKEOFF
  Guided:    true

=== MANAGED DRONE STATE MODEL UPDATE ===
vehicle :
 {
  time_boot_ms: 1765832051711,
  connected: true,
  armed: true,
  guided: true,
  manual_input: false,
  mode: "AUTO.TAKEOFF",
  system_status: 4,
}
status:
 {
  time_boot_ms: 1765832051789,
  connected: true,
  gcs_link: true,
  faults: [],
  armable: true,
  arm_reasons: [],
}
=== MANAGED DRONE STATE MODEL UPDATE ===
vehicle :
 {
  time_boot_ms: 1765832052734,
  connected: true,
  armed: true,
  guided: true,
  manual_input: false,
  mode: "AUTO.TAKEOFF",
  system_status: 4,
}
status:
 {
  time_boot_ms: 1765832052799,
  connected: true,
  gcs_link: true,
  faults: [],
  armable: false,
  arm_reasons: [ "not.on.ground" ],
}
=== MANAGED DRONE STATE MODEL UPDATE ===
vehicle :
 {
  time_boot_ms: 1765832056829,
  connected: true,
  armed: true,
  guided: true,
  manual_input: false,
  mode: "AUTO.TAKEOFF",
  system_status: 4,
}
status:
 {
  time_boot_ms: 1765832057647,
  connected: true,
  gcs_link: true,
  faults: [],
  armable: false,
  arm_reasons: [ "not.on.ground" ],
}
=== RAW ROS SUBSCRIPTION RECEIVED CHANGED DATA ===
Drone State:
  Connected: true
  Armed:     true
  Mode:      AUTO.LOITER
  Guided:    true

=== MANAGED DRONE STATE MODEL UPDATE ===
vehicle :
 {
  time_boot_ms: 1765832057848,
  connected: true,
  armed: true,
  guided: true,
  manual_input: false,
  mode: "AUTO.LOITER",
  system_status: 4,
}
status:
 {
  time_boot_ms: 1765832057849,
  connected: true,
  gcs_link: true,
  faults: [],
  armable: false,
  arm_reasons: [ "not.on.ground" ],
}
```

### How It Works

The tutorial demonstrates two approaches to accessing drone state:

1. **Direct MAVROS Usage**: You can subscribe directly to ROS topics using the MAVROS library. This gives you raw access to individual sensor and state messages, allowing for fine-grained control over what data you receive and how you process it.

2. **Managed State Utility**: The `DroneStateModel` provides an automatically managed state utility that handles subscriptions to multiple ROS topics internally. It aggregates and processes the data into a structured state object, making it easier to work with drone state without manually managing each subscription.

### Update Detection

Updates are detected through ROS topic subscriptions:

- For the raw approach, each topic subscription triggers a callback whenever a new message is published on that topic.
- For the managed approach, the `DroneStateModel` subscribes to all relevant topics and emits updates when the aggregated state changes, reducing the need for individual topic handlers.

This allows real-time monitoring of the drone's connection status, arming state, flight mode, and other critical parameters.
