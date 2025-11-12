/**
 * ROS2 Bridge for TensorFleet VS Code Extension
 * 
 * Provides ROS2 connectivity for:
 * - Image topic subscription (sensor_msgs/Image, sensor_msgs/CompressedImage)
 * - Twist command publishing (geometry_msgs/Twist)
 * - PX4/MAVROS telemetry
 * - Connection management
 */

import * as vscode from 'vscode';
import { EventEmitter } from 'events';

// Type definitions for ROS2 messages
interface ROS2ImageMessage {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  height: number;
  width: number;
  encoding: string;
  is_bigendian: number;
  step: number;
  data: Buffer;
}

interface ROS2CompressedImageMessage {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  format: string;
  data: Buffer;
}

interface ROS2TwistMessage {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}

interface ROS2PoseMessage {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
}

export class ROS2Bridge extends EventEmitter {
  private rclnodejs: any = null;
  private node: any = null;
  private subscriptions: Map<string, any> = new Map();
  private publishers: Map<string, any> = new Map();
  private isInitialized: boolean = false;
  private isConnected: boolean = false;
  private initPromise: Promise<void> | null = null;

  constructor(_context: vscode.ExtensionContext) {
    super();
  }

  /**
   * Initialize ROS2 connection
   */
  async initialize(): Promise<void> {
    if (this.initPromise) {
      return this.initPromise;
    }

    this.initPromise = this._initializeInternal();
    return this.initPromise;
  }

  private async _initializeInternal(): Promise<void> {
    if (this.isInitialized) {
      return;
    }

    try {
      console.log('[ROS2Bridge] Initializing ROS2...');
      
      // Dynamically import rclnodejs
      try {
        this.rclnodejs = require('rclnodejs');
      } catch (error) {
        console.warn('[ROS2Bridge] rclnodejs not found. Install with: npm install rclnodejs');
        throw new Error('rclnodejs not installed. See ROS2 setup documentation.');
      }

      // Initialize ROS2 context
      await this.rclnodejs.init();
      
      // Create ROS2 node
      this.node = new this.rclnodejs.Node('tensorfleet_vscode_node');
      
      // Start spinning
      this.rclnodejs.spin(this.node);
      
      this.isInitialized = true;
      this.isConnected = true;
      
      console.log('[ROS2Bridge] ROS2 initialized successfully');
      this.emit('connected');
      
    } catch (error) {
      console.error('[ROS2Bridge] Failed to initialize ROS2:', error);
      this.isInitialized = false;
      this.isConnected = false;
      this.emit('error', error);
      throw error;
    }
  }

  /**
   * Check if ROS2 is connected
   */
  isROS2Connected(): boolean {
    return this.isConnected && this.isInitialized;
  }

  /**
   * Subscribe to image topic
   */
  async subscribeToImageTopic(
    topic: string,
    callback: (imageData: string, metadata: any) => void
  ): Promise<void> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    // Unsubscribe from existing subscription if any
    if (this.subscriptions.has(topic)) {
      this.unsubscribeFromTopic(topic);
    }

    try {
      const messageType = topic.includes('compressed') 
        ? 'sensor_msgs/msg/CompressedImage' 
        : 'sensor_msgs/msg/Image';

      console.log(`[ROS2Bridge] Subscribing to ${topic} (${messageType})`);

      const subscription = this.node.createSubscription(
        messageType,
        topic,
        (msg: ROS2ImageMessage | ROS2CompressedImageMessage) => {
          try {
            const imageData = this.convertROS2ImageToDataURI(msg, messageType);
            const metadata = {
              topic,
              timestamp: this.convertROS2Time(msg.header.stamp),
              encoding: 'encoding' in msg ? msg.encoding : 'format' in msg ? msg.format : 'unknown',
              width: 'width' in msg ? msg.width : 0,
              height: 'height' in msg ? msg.height : 0,
              frame_id: msg.header.frame_id
            };
            callback(imageData, metadata);
          } catch (error) {
            console.error(`[ROS2Bridge] Error processing image from ${topic}:`, error);
          }
        }
      );

      this.subscriptions.set(topic, subscription);
      console.log(`[ROS2Bridge] Successfully subscribed to ${topic}`);
      
    } catch (error) {
      console.error(`[ROS2Bridge] Failed to subscribe to ${topic}:`, error);
      throw error;
    }
  }

  /**
   * Publish twist message to cmd_vel topic
   */
  async publishTwist(topic: string, twist: ROS2TwistMessage): Promise<void> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    try {
      // Create publisher if it doesn't exist
      if (!this.publishers.has(topic)) {
        console.log(`[ROS2Bridge] Creating publisher for ${topic}`);
        const publisher = this.node.createPublisher(
          'geometry_msgs/msg/Twist',
          topic
        );
        this.publishers.set(topic, publisher);
      }

      const publisher = this.publishers.get(topic);
      publisher.publish(twist);
      
    } catch (error) {
      console.error(`[ROS2Bridge] Failed to publish to ${topic}:`, error);
      throw error;
    }
  }

  /**
   * Subscribe to PX4 telemetry via MAVROS
   */
  async subscribeToPX4Telemetry(
    callback: (telemetry: any) => void
  ): Promise<void> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    const topics = {
      pose: '/mavros/local_position/pose',
      velocity: '/mavros/local_position/velocity_body',
      battery: '/mavros/battery',
      state: '/mavros/state',
      gps: '/mavros/global_position/global'
    };

    const telemetryData: any = {
      pose: null,
      velocity: null,
      battery: null,
      state: null,
      gps: null
    };

    // Subscribe to pose
    try {
      const poseSub = this.node.createSubscription(
        'geometry_msgs/msg/PoseStamped',
        topics.pose,
        (msg: ROS2PoseMessage) => {
          telemetryData.pose = {
            position: msg.pose.position,
            orientation: msg.pose.orientation,
            timestamp: this.convertROS2Time(msg.header.stamp)
          };
          callback(telemetryData);
        }
      );
      this.subscriptions.set('px4_pose', poseSub);
    } catch (error) {
      console.warn('[ROS2Bridge] Could not subscribe to PX4 pose:', error);
    }

    // Subscribe to battery
    try {
      const batterySub = this.node.createSubscription(
        'sensor_msgs/msg/BatteryState',
        topics.battery,
        (msg: any) => {
          telemetryData.battery = {
            voltage: msg.voltage,
            current: msg.current,
            percentage: msg.percentage,
            remaining: msg.capacity
          };
          callback(telemetryData);
        }
      );
      this.subscriptions.set('px4_battery', batterySub);
    } catch (error) {
      console.warn('[ROS2Bridge] Could not subscribe to PX4 battery:', error);
    }

    // Subscribe to state
    try {
      const stateSub = this.node.createSubscription(
        'mavros_msgs/msg/State',
        topics.state,
        (msg: any) => {
          telemetryData.state = {
            connected: msg.connected,
            armed: msg.armed,
            mode: msg.mode,
            guided: msg.guided
          };
          callback(telemetryData);
        }
      );
      this.subscriptions.set('px4_state', stateSub);
    } catch (error) {
      console.warn('[ROS2Bridge] Could not subscribe to PX4 state:', error);
    }

    console.log('[ROS2Bridge] PX4 telemetry subscriptions created');
  }

  /**
   * Get list of active ROS2 topics
   */
  async getTopicList(): Promise<Array<{ name: string; type: string }>> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    try {
      const namesAndTypes = this.node.getTopicNamesAndTypes();
      return namesAndTypes.map((item: any) => ({
        name: item[0],
        type: item[1][0]
      }));
    } catch (error) {
      console.error('[ROS2Bridge] Failed to get topic list:', error);
      return [];
    }
  }

  /**
   * Unsubscribe from a topic
   */
  unsubscribeFromTopic(topic: string): void {
    const subscription = this.subscriptions.get(topic);
    if (subscription) {
      try {
        this.node.destroySubscription(subscription);
        this.subscriptions.delete(topic);
        console.log(`[ROS2Bridge] Unsubscribed from ${topic}`);
      } catch (error) {
        console.error(`[ROS2Bridge] Error unsubscribing from ${topic}:`, error);
      }
    }
  }

  /**
   * Shutdown ROS2 connection
   */
  async shutdown(): Promise<void> {
    console.log('[ROS2Bridge] Shutting down...');
    
    // Destroy all subscriptions
    for (const [topic, subscription] of this.subscriptions) {
      try {
        this.node.destroySubscription(subscription);
      } catch (error) {
        console.error(`[ROS2Bridge] Error destroying subscription ${topic}:`, error);
      }
    }
    this.subscriptions.clear();

    // Destroy all publishers
    for (const [topic, publisher] of this.publishers) {
      try {
        this.node.destroyPublisher(publisher);
      } catch (error) {
        console.error(`[ROS2Bridge] Error destroying publisher ${topic}:`, error);
      }
    }
    this.publishers.clear();

    // Destroy node
    if (this.node && this.rclnodejs) {
      try {
        this.node.destroy();
        await this.rclnodejs.shutdown();
      } catch (error) {
        console.error('[ROS2Bridge] Error during shutdown:', error);
      }
    }

    this.isInitialized = false;
    this.isConnected = false;
    this.emit('disconnected');
    
    console.log('[ROS2Bridge] Shutdown complete');
  }

  /**
   * Convert ROS2 image message to data URI for display
   */
  private convertROS2ImageToDataURI(
    msg: ROS2ImageMessage | ROS2CompressedImageMessage,
    messageType: string
  ): string {
    if (messageType.includes('CompressedImage')) {
      // Compressed image - convert buffer to base64
      const compressed = msg as ROS2CompressedImageMessage;
      const base64 = compressed.data.toString('base64');
      const mimeType = compressed.format.includes('jpeg') || compressed.format.includes('jpg') 
        ? 'image/jpeg' 
        : compressed.format.includes('png') 
          ? 'image/png' 
          : 'image/jpeg';
      return `data:${mimeType};base64,${base64}`;
    } else {
      // Raw image - need to encode
      const raw = msg as ROS2ImageMessage;
      
      // For simplicity, we'll convert raw images to canvas and then to data URI
      // This requires canvas support - in production you'd use a proper image encoder
      // For now, return a placeholder indicating raw image data
      const base64 = raw.data.toString('base64');
      return `data:application/octet-stream;base64,${base64}`;
    }
  }

  /**
   * Convert ROS2 timestamp to ISO string
   */
  private convertROS2Time(stamp: { sec: number; nanosec: number }): string {
    const milliseconds = stamp.sec * 1000 + stamp.nanosec / 1000000;
    return new Date(milliseconds).toISOString();
  }
}

