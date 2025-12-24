# TensorFleet Util

## Overview

`tensorfleet-util` is a utility library designed for drone control and ROS (Robot Operating System) integration within the TensorFleet ecosystem. It provides essential abstractions and tools to simplify interaction with drones using MAVROS (MAVLink extendable communication library for ROS) and ROS Bridge.

## Purpose

This package serves as the core dependency for JavaScript-based drone applications in TensorFleet projects. It abstracts low-level ROS topic subscriptions and message handling, offering high-level APIs for:

- Establishing connections to drone environments
- Monitoring and managing drone state
- Performing mission control operations
- Integrating with ROS-based systems

## Key Components

### DroneStateModel

A managed state utility that aggregates data from multiple MAVROS ROS topics into a unified `DroneState` object. It handles subscriptions to topics such as:

- Global position (GPS fix)
- Compass heading
- Vehicle state (arming, mode, connection status)
- Extended state (landed/VTOL state)
- Battery telemetry
- VFR HUD data (airspeed, groundspeed, throttle)
- Local position and velocity
- IMU data
- Altitude information
- Home position

The model emits updates when state changes occur, providing real-time monitoring capabilities without manual topic management.

### DroneController

Provides mission control functionality for executing flight operations, waypoints, and automated tasks.

### ROS Bridge API

Defines the interface for ROS Bridge communication, including:

- Topic subscription and publishing
- Service calls
- Parameter management
- Topic discovery
- Connection handling

### ROS Types

TypeScript type definitions for various ROS message types used in MAVROS communications, ensuring type safety when working with ROS messages.

## Usage

The library is typically used as follows:

1. Establish a ROS Bridge connection using the provided APIs
2. Create a `DroneStateModel` instance to monitor drone state
3. Use the `DroneController` for mission execution
4. Subscribe to state updates for real-time monitoring

This abstraction layer makes it easier to build drone applications without deep knowledge of ROS internals, while still providing access to raw ROS functionality when needed.

## Integration

`tensorfleet-util` is included as a dependency in TensorFleet JavaScript projects and is built using TypeScript for type safety. It supports modern JavaScript environments and is compatible with bundlers like Bun.
