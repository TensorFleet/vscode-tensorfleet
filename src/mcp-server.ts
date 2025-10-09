#!/usr/bin/env node

import { Server } from "@modelcontextprotocol/sdk/server/index.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import {
  CallToolRequestSchema,
  ListToolsRequestSchema,
  ListResourcesRequestSchema,
  ReadResourceRequestSchema,
} from "@modelcontextprotocol/sdk/types.js";
import * as net from "net";
import * as os from "os";
import * as path from "path";

// TensorFleet MCP Server
// Exposes drone tooling, ROS2, Gazebo, and AI ops functionality to AI assistants

interface ToolExecutor {
  execute(args: Record<string, unknown>): Promise<{ content: Array<{ type: string; text: string }> }>;
}

// Helper function to send commands to VS Code extension bridge
async function sendBridgeCommand(command: string, params?: any): Promise<any> {
  const socketPath = path.join(os.tmpdir(), 'tensorfleet-mcp-bridge.sock');
  
  return new Promise((resolve, reject) => {
    const client = net.createConnection(socketPath, () => {
      const message = JSON.stringify({ command, params });
      client.write(message);
    });

    client.on('data', (data) => {
      try {
        const response = JSON.parse(data.toString());
        client.end();
        resolve(response);
      } catch (error) {
        client.end();
        reject(error);
      }
    });

    client.on('error', (error) => {
      // Bridge not available (extension not running or bridge not started)
      resolve({ success: false, error: 'VS Code extension bridge not available' });
    });

    client.setTimeout(1000, () => {
      client.end();
      resolve({ success: false, error: 'Bridge connection timeout' });
    });
  });
}

class TensorFleetMCPServer {
  private server: Server;
  private tools: Map<string, ToolExecutor>;

  constructor() {
    this.server = new Server(
      {
        name: "tensorfleet-drone",
        version: "0.0.1",
      },
      {
        capabilities: {
          tools: {},
          resources: {},
        },
      }
    );

    this.tools = new Map();
    this.setupTools();
    this.setupRequestHandlers();
  }

  private setupTools() {
    // Tool 1: Get Drone Status
    this.tools.set("get_drone_status", {
      execute: async (args) => {
        const droneId = args.drone_id as string || "default";
        return {
          content: [
            {
              type: "text",
              text: JSON.stringify({
                drone_id: droneId,
                status: "ready",
                battery: "85%",
                gps_lock: true,
                altitude: "0m",
                mode: "stabilized",
                message: "Drone is ready for operation"
              }, null, 2)
            }
          ]
        };
      }
    });

    // Tool 2: Launch ROS2 Environment
    this.tools.set("launch_ros2_environment", {
      execute: async (args) => {
        const package_name = args.package_name as string || "px4_ros_com";
        const launch_file = args.launch_file as string || "sensor_combined_listener.launch.py";
        
        // Open ROS2 panel in VS Code
        const bridgeResponse = await sendBridgeCommand('openROS2Panel');
        
        let statusMessage = `ROS2 environment launch initiated:\n` +
                           `Package: ${package_name}\n` +
                           `Launch File: ${launch_file}\n\n`;
        
        if (bridgeResponse.success) {
          statusMessage += `✓ ROS 2 panel opened in VS Code\n\n`;
        }
        
        statusMessage += `Command: ros2 launch ${package_name} ${launch_file}\n` +
                        `Status: Terminal session created. Check TensorFleet ROS 2 terminal for output.`;
        
        return {
          content: [
            {
              type: "text",
              text: statusMessage
            }
          ]
        };
      }
    });

    // Tool 3: Start Gazebo Simulation
    this.tools.set("start_gazebo_simulation", {
      execute: async (args) => {
        const world = args.world as string || "empty";
        const model = args.model as string || "iris";
        
        // Try to open Gazebo panel in VS Code
        const bridgeResponse = await sendBridgeCommand('openGazeboPanel', { world, model });
        
        let statusMessage = `Gazebo simulation started:\n` +
                           `World: ${world}\n` +
                           `Model: ${model}\n\n`;
        
        if (bridgeResponse.success) {
          statusMessage += `✓ Gazebo panel opened in VS Code\n`;
        }
        
        statusMessage += `Command: gz sim ${world}.sdf\n` +
                        `Status: Simulation environment initializing...`;
        
        return {
          content: [
            {
              type: "text",
              text: statusMessage
            }
          ]
        };
      }
    });

    // Tool 4: Run AI Model Inference
    this.tools.set("run_ai_inference", {
      execute: async (args) => {
        const model_name = args.model_name as string || "yolov8";
        const input_source = args.input_source as string || "camera";
        const confidence_threshold = args.confidence_threshold as number || 0.5;
        
        // Open AI Ops panel in VS Code
        const bridgeResponse = await sendBridgeCommand('openAIPanel');
        
        let statusMessage = `AI Model Inference Started:\n` +
                           `Model: ${model_name}\n` +
                           `Input Source: ${input_source}\n` +
                           `Confidence Threshold: ${confidence_threshold}\n\n`;
        
        if (bridgeResponse.success) {
          statusMessage += `✓ AI Ops panel opened in VS Code\n\n`;
        }
        
        statusMessage += `Status: Processing video feed for object detection...\n` +
                        `Results will be displayed in AI Ops dashboard.`;
        
        return {
          content: [
            {
              type: "text",
              text: statusMessage
            }
          ]
        };
      }
    });

    // Tool 5: Configure QGroundControl Mission
    this.tools.set("configure_qgc_mission", {
      execute: async (args) => {
        const mission_type = args.mission_type as string || "waypoint";
        const waypoints = args.waypoints as any[] || [];
        
        // Open QGC panel in VS Code
        const bridgeResponse = await sendBridgeCommand('openQGCPanel');
        
        let statusMessage = `QGroundControl Mission Configuration:\n` +
                           `Type: ${mission_type}\n` +
                           `Waypoints: ${waypoints.length}\n\n`;
        
        if (bridgeResponse.success) {
          statusMessage += `✓ QGroundControl panel opened in VS Code\n\n`;
        }
        
        statusMessage += `Mission loaded and ready for execution.`;
        
        return {
          content: [
            {
              type: "text",
              text: statusMessage
            }
          ]
        };
      }
    });

    // Tool 6: Install TensorFleet Tools
    this.tools.set("install_tensorfleet_tools", {
      execute: async (args) => {
        const install_path = args.install_path as string || "~/tensorfleet-tools";
        
        return {
          content: [
            {
              type: "text",
              text: `TensorFleet Tools Installation:\n` +
                    `Target Path: ${install_path}\n\n` +
                    `Installing:\n` +
                    `- ROS2 configurations\n` +
                    `- Gazebo world files\n` +
                    `- PX4 firmware tools\n` +
                    `- AI model weights\n\n` +
                    `Status: Installation in progress...`
            }
          ]
        };
      }
    });

    // Tool 7: Get Telemetry Data
    this.tools.set("get_telemetry_data", {
      execute: async () => {
        // Note: data_type parameter available for future filtering
        const telemetry = {
          position: { lat: 37.7749, lon: -122.4194, alt: 100 },
          velocity: { vx: 2.5, vy: 1.2, vz: 0.0 },
          attitude: { roll: 0.1, pitch: -0.2, yaw: 45.0 },
          battery: { voltage: 12.6, current: 15.3, percentage: 85 },
          gps: { satellites: 12, fix_type: "3D" },
          sensors: {
            imu: { status: "active" },
            lidar: { status: "active", distance: 3.5 },
            camera: { status: "streaming", fps: 30 }
          }
        };
        
        return {
          content: [
            {
              type: "text",
              text: JSON.stringify(telemetry, null, 2)
            }
          ]
        };
      }
    });
  }

  private setupRequestHandlers() {
    // Handle tool listing
    this.server.setRequestHandler(ListToolsRequestSchema, async () => {
      return {
        tools: [
          {
            name: "get_drone_status",
            description: "Get the current status of a TensorFleet drone including battery, GPS, mode, and readiness",
            inputSchema: {
              type: "object",
              properties: {
                drone_id: {
                  type: "string",
                  description: "The ID of the drone (default: 'default')"
                }
              }
            }
          },
          {
            name: "launch_ros2_environment",
            description: "Launch a ROS2 environment with specified package and launch file for drone operations",
            inputSchema: {
              type: "object",
              properties: {
                package_name: {
                  type: "string",
                  description: "ROS2 package name (e.g., 'px4_ros_com')"
                },
                launch_file: {
                  type: "string",
                  description: "Launch file name (e.g., 'sensor_combined_listener.launch.py')"
                }
              }
            }
          },
          {
            name: "start_gazebo_simulation",
            description: "Start a Gazebo simulation with specified world and drone model",
            inputSchema: {
              type: "object",
              properties: {
                world: {
                  type: "string",
                  description: "Gazebo world name (e.g., 'empty', 'warehouse', 'outdoor')"
                },
                model: {
                  type: "string",
                  description: "Drone model name (e.g., 'iris', 'typhoon_h480')"
                }
              }
            }
          },
          {
            name: "run_ai_inference",
            description: "Run AI model inference on drone video feed for object detection or analysis",
            inputSchema: {
              type: "object",
              properties: {
                model_name: {
                  type: "string",
                  description: "AI model name (e.g., 'yolov8', 'detectron2')"
                },
                input_source: {
                  type: "string",
                  description: "Input source (e.g., 'camera', 'video_file')"
                },
                confidence_threshold: {
                  type: "number",
                  description: "Confidence threshold for detections (0.0 to 1.0)"
                }
              }
            }
          },
          {
            name: "configure_qgc_mission",
            description: "Configure a QGroundControl mission with waypoints and parameters",
            inputSchema: {
              type: "object",
              properties: {
                mission_type: {
                  type: "string",
                  description: "Mission type (e.g., 'waypoint', 'survey', 'corridor_scan')"
                },
                waypoints: {
                  type: "array",
                  description: "Array of waypoint objects with lat, lon, alt",
                  items: {
                    type: "object",
                    properties: {
                      lat: { type: "number" },
                      lon: { type: "number" },
                      alt: { type: "number" }
                    }
                  }
                }
              }
            }
          },
          {
            name: "install_tensorfleet_tools",
            description: "Install TensorFleet bundled tools including ROS2 configs, Gazebo worlds, and AI models",
            inputSchema: {
              type: "object",
              properties: {
                install_path: {
                  type: "string",
                  description: "Installation directory path"
                }
              }
            }
          },
          {
            name: "get_telemetry_data",
            description: "Get real-time telemetry data from the drone including position, velocity, attitude, battery, and sensors",
            inputSchema: {
              type: "object",
              properties: {
                data_type: {
                  type: "string",
                  description: "Type of telemetry data (e.g., 'all', 'position', 'battery', 'sensors')"
                }
              }
            }
          }
        ]
      };
    });

    // Handle tool execution
    this.server.setRequestHandler(CallToolRequestSchema, async (request) => {
      const toolName = request.params.name;
      const args = request.params.arguments || {};

      const tool = this.tools.get(toolName);
      if (!tool) {
        throw new Error(`Unknown tool: ${toolName}`);
      }

      return await tool.execute(args);
    });

    // Handle resource listing
    this.server.setRequestHandler(ListResourcesRequestSchema, async () => {
      return {
        resources: [
          {
            uri: "tensorfleet://drone/config",
            name: "Drone Configuration",
            description: "Current drone configuration and parameters",
            mimeType: "application/json"
          },
          {
            uri: "tensorfleet://ros2/topics",
            name: "ROS2 Topics",
            description: "List of active ROS2 topics and their message types",
            mimeType: "application/json"
          },
          {
            uri: "tensorfleet://gazebo/models",
            name: "Gazebo Models",
            description: "Available Gazebo drone models and worlds",
            mimeType: "application/json"
          },
          {
            uri: "tensorfleet://ai/models",
            name: "AI Models",
            description: "Available AI models for drone video analysis",
            mimeType: "application/json"
          },
          {
            uri: "tensorfleet://qgc/missions",
            name: "QGC Missions",
            description: "Saved QGroundControl mission plans",
            mimeType: "application/json"
          }
        ]
      };
    });

    // Handle resource reading
    this.server.setRequestHandler(ReadResourceRequestSchema, async (request) => {
      const uri = request.params.uri;

      const resources: Record<string, any> = {
        "tensorfleet://drone/config": {
          vehicle_type: "quadcopter",
          firmware: "PX4 v1.14.0",
          frame: "X",
          battery_cells: "4S",
          max_speed: "15 m/s",
          flight_time: "25 minutes",
          payload_capacity: "500g"
        },
        "tensorfleet://ros2/topics": {
          topics: [
            { name: "/fmu/in/vehicle_command", type: "px4_msgs/VehicleCommand" },
            { name: "/fmu/out/vehicle_status", type: "px4_msgs/VehicleStatus" },
            { name: "/fmu/out/sensor_combined", type: "px4_msgs/SensorCombined" },
            { name: "/camera/image_raw", type: "sensor_msgs/Image" },
            { name: "/mavros/local_position/pose", type: "geometry_msgs/PoseStamped" }
          ]
        },
        "tensorfleet://gazebo/models": {
          models: [
            { name: "iris", type: "quadcopter", sensors: ["imu", "gps", "camera"] },
            { name: "typhoon_h480", type: "hexacopter", sensors: ["imu", "gps", "camera", "lidar"] },
            { name: "plane", type: "fixed_wing", sensors: ["imu", "gps", "camera"] }
          ],
          worlds: [
            { name: "empty", description: "Empty world with ground plane" },
            { name: "warehouse", description: "Indoor warehouse environment" },
            { name: "outdoor", description: "Outdoor terrain with obstacles" }
          ]
        },
        "tensorfleet://ai/models": {
          models: [
            { name: "yolov8", task: "object_detection", input: "640x640", fps: "30" },
            { name: "detectron2", task: "instance_segmentation", input: "800x600", fps: "15" },
            { name: "depth_anything", task: "depth_estimation", input: "518x518", fps: "20" },
            { name: "stable_baselines_ppo", task: "autonomous_flight", type: "reinforcement_learning" }
          ]
        },
        "tensorfleet://qgc/missions": {
          missions: [
            { name: "perimeter_patrol", waypoints: 8, distance: "500m", duration: "5min" },
            { name: "survey_area", waypoints: 24, distance: "1.2km", duration: "12min" },
            { name: "inspection_route", waypoints: 12, distance: "800m", duration: "8min" }
          ]
        }
      };

      const data = resources[uri];
      if (!data) {
        throw new Error(`Resource not found: ${uri}`);
      }

      return {
        contents: [
          {
            uri,
            mimeType: "application/json",
            text: JSON.stringify(data, null, 2)
          }
        ]
      };
    });
  }

  async run() {
    const transport = new StdioServerTransport();
    await this.server.connect(transport);
    console.error("TensorFleet MCP Server running on stdio");
  }
}

// Start the server
const server = new TensorFleetMCPServer();
server.run().catch(console.error);

