/**
 * ROS2 WebSocket Bridge for TensorFleet VS Code Extension
 * 
 * Provides ROS2 connectivity via WebSocket (rosbridge_suite) for:
 * - Image topic subscription (sensor_msgs/Image, sensor_msgs/CompressedImage)
 * - Twist command publishing (geometry_msgs/Twist)
 * - PX4/MAVROS telemetry
 * - Connection management
 * 
 * This bridge connects to remote ROS2 instances (like Firecracker VMs)
 * without requiring local ROS2 installation.
 */

import { EventEmitter } from 'events';

// Type definitions for ROSLIB
interface ROSLib {
  Ros: any;
  Topic: any;
  Message: any;
}

export class ROS2WebSocketBridge extends EventEmitter {
  private ROSLIB: ROSLib | null = null;
  private ros: any = null;
  private topics: Map<string, any> = new Map();
  private isConnected: boolean = false;
  private wsUrl: string;
  private reconnectTimer: NodeJS.Timeout | null = null;
  private autoReconnect: boolean = true;

  constructor(wsUrl: string = 'ws://172.16.0.2:9091') {
    super();
    this.wsUrl = wsUrl;
  }

  /**
   * Initialize WebSocket connection to ROS bridge
   */
  async connect(): Promise<void> {
    try {
      console.log(`[ROS2WebSocketBridge] Connecting to ${this.wsUrl}...`);
      
      // Dynamically import roslib
      try {
        this.ROSLIB = require('roslib');
      } catch (error) {
        console.error('[ROS2WebSocketBridge] roslib not found. Install with: npm install roslib');
        throw new Error('roslib not installed. Run: npm install roslib');
      }

      if (!this.ROSLIB) {
        throw new Error('Failed to load roslib');
      }

      // Create ROS connection
      this.ros = new this.ROSLIB.Ros({
        url: this.wsUrl
      });

      // Setup event handlers
      this.ros.on('connection', () => {
        console.log('[ROS2WebSocketBridge] Connected to ROS bridge');
        this.isConnected = true;
        this.emit('connected');
        
        // Update URL for reconnection
        this.wsUrl = this.ros?.url || this.wsUrl;
        
        // Clear reconnect timer if exists
        if (this.reconnectTimer) {
          clearTimeout(this.reconnectTimer);
          this.reconnectTimer = null;
        }
      });

      this.ros.on('error', (error: any) => {
        console.error('[ROS2WebSocketBridge] Connection error:', error);
        this.isConnected = false;
        this.emit('error', error);
      });

      this.ros.on('close', () => {
        console.log('[ROS2WebSocketBridge] Connection closed');
        this.isConnected = false;
        this.emit('disconnected');
        
        // Auto-reconnect if enabled
        if (this.autoReconnect && !this.reconnectTimer) {
          console.log('[ROS2WebSocketBridge] Scheduling reconnect in 5s...');
          this.reconnectTimer = setTimeout(() => {
            this.reconnectTimer = null;
            this.connect().catch(err => {
              console.error('[ROS2WebSocketBridge] Reconnect failed:', err);
            });
          }, 5000);
        }
      });

    } catch (error) {
      console.error('[ROS2WebSocketBridge] Failed to initialize:', error);
      this.emit('error', error);
      throw error;
    }
  }

  /**
   * Check if connected to ROS bridge
   */
  isROS2Connected(): boolean {
    return this.isConnected && this.ros !== null;
  }

  /**
   * Subscribe to image topic
   */
  async subscribeToImageTopic(
    topic: string,
    callback: (imageData: string, metadata: any) => void
  ): Promise<void> {
    if (!this.isConnected || !this.ROSLIB || !this.ros) {
      await this.connect();
    }

    if (!this.ROSLIB || !this.ros) {
      throw new Error('Failed to connect to ROS bridge');
    }

    // Unsubscribe from existing subscription if any
    if (this.topics.has(topic)) {
      this.unsubscribeFromTopic(topic);
    }

    try {
      const messageType = topic.includes('compressed') 
        ? 'sensor_msgs/CompressedImage' 
        : 'sensor_msgs/Image';

      console.log(`[ROS2WebSocketBridge] Subscribing to ${topic} (${messageType})`);

      const rosTopic = new this.ROSLIB.Topic({
        ros: this.ros,
        name: topic,
        messageType: messageType
      });

      rosTopic.subscribe((message: any) => {
        try {
          const imageData = this.convertROS2ImageToDataURI(message, messageType);
          const metadata = {
            topic,
            timestamp: message.header ? this.convertROS2Time(message.header.stamp) : new Date().toISOString(),
            encoding: message.encoding || message.format || 'unknown',
            width: message.width || 0,
            height: message.height || 0,
            frame_id: message.header?.frame_id || 'unknown'
          };
          callback(imageData, metadata);
        } catch (error) {
          console.error(`[ROS2WebSocketBridge] Error processing image from ${topic}:`, error);
        }
      });

      this.topics.set(topic, rosTopic);
      console.log(`[ROS2WebSocketBridge] Successfully subscribed to ${topic}`);
      
    } catch (error) {
      console.error(`[ROS2WebSocketBridge] Failed to subscribe to ${topic}:`, error);
      throw error;
    }
  }

  /**
   * Publish twist message to cmd_vel topic
   */
  async publishTwist(topic: string, twist: any): Promise<void> {
    if (!this.isConnected || !this.ROSLIB || !this.ros) {
      await this.connect();
    }

    if (!this.ROSLIB || !this.ros) {
      throw new Error('Failed to connect to ROS bridge');
    }

    try {
      // Create publisher if it doesn't exist
      if (!this.topics.has(topic)) {
        console.log(`[ROS2WebSocketBridge] Creating publisher for ${topic}`);
        const cmdVelTopic = new this.ROSLIB.Topic({
          ros: this.ros,
          name: topic,
          messageType: 'geometry_msgs/Twist'
        });
        this.topics.set(topic, cmdVelTopic);
      }

      const rosTopic = this.topics.get(topic);
      
      // Create twist message
      const twistMsg = new this.ROSLIB.Message(twist);
      
      // Publish
      rosTopic.publish(twistMsg);
      
    } catch (error) {
      console.error(`[ROS2WebSocketBridge] Failed to publish to ${topic}:`, error);
      throw error;
    }
  }

  /**
   * Subscribe to PX4 telemetry via MAVROS
   */
  async subscribeToPX4Telemetry(
    callback: (telemetry: any) => void
  ): Promise<void> {
    if (!this.isConnected || !this.ROSLIB || !this.ros) {
      await this.connect();
    }

    if (!this.ROSLIB || !this.ros) {
      throw new Error('Failed to connect to ROS bridge');
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
      const poseTopic = new this.ROSLIB.Topic({
        ros: this.ros,
        name: topics.pose,
        messageType: 'geometry_msgs/PoseStamped'
      });

      poseTopic.subscribe((message: any) => {
        telemetryData.pose = {
          position: message.pose.position,
          orientation: message.pose.orientation,
          timestamp: this.convertROS2Time(message.header.stamp)
        };
        callback(telemetryData);
      });

      this.topics.set('px4_pose', poseTopic);
    } catch (error) {
      console.warn('[ROS2WebSocketBridge] Could not subscribe to PX4 pose:', error);
    }

    // Subscribe to battery
    try {
      const batteryTopic = new this.ROSLIB.Topic({
        ros: this.ros,
        name: topics.battery,
        messageType: 'sensor_msgs/BatteryState'
      });

      batteryTopic.subscribe((message: any) => {
        telemetryData.battery = {
          voltage: message.voltage,
          current: message.current,
          percentage: message.percentage,
          remaining: message.capacity
        };
        callback(telemetryData);
      });

      this.topics.set('px4_battery', batteryTopic);
    } catch (error) {
      console.warn('[ROS2WebSocketBridge] Could not subscribe to PX4 battery:', error);
    }

    // Subscribe to state
    try {
      const stateTopic = new this.ROSLIB.Topic({
        ros: this.ros,
        name: topics.state,
        messageType: 'mavros_msgs/State'
      });

      stateTopic.subscribe((message: any) => {
        telemetryData.state = {
          connected: message.connected,
          armed: message.armed,
          mode: message.mode,
          guided: message.guided
        };
        callback(telemetryData);
      });

      this.topics.set('px4_state', stateTopic);
    } catch (error) {
      console.warn('[ROS2WebSocketBridge] Could not subscribe to PX4 state:', error);
    }

    console.log('[ROS2WebSocketBridge] PX4 telemetry subscriptions created');
  }

  /**
   * Get list of active ROS2 topics
   */
  async getTopicList(): Promise<Array<{ name: string; type: string }>> {
    if (!this.isConnected || !this.ROSLIB || !this.ros) {
      await this.connect();
    }

    if (!this.ros) {
      throw new Error('ROS connection not established');
    }

    return new Promise((resolve, reject) => {
      try {
        this.ros!.getTopics((topics: any) => {
          const topicList = topics.topics.map((name: string, index: number) => ({
            name: name,
            type: topics.types[index]
          }));
          resolve(topicList);
        }, (error: any) => {
          console.error('[ROS2WebSocketBridge] Failed to get topic list:', error);
          reject(error);
        });
      } catch (error) {
        console.error('[ROS2WebSocketBridge] Failed to get topic list:', error);
        resolve([]);
      }
    });
  }

  /**
   * Unsubscribe from a topic
   */
  unsubscribeFromTopic(topic: string): void {
    const rosTopic = this.topics.get(topic);
    if (rosTopic) {
      try {
        rosTopic.unsubscribe();
        this.topics.delete(topic);
        console.log(`[ROS2WebSocketBridge] Unsubscribed from ${topic}`);
      } catch (error) {
        console.error(`[ROS2WebSocketBridge] Error unsubscribing from ${topic}:`, error);
      }
    }
  }

  /**
   * Set WebSocket URL
   */
  setUrl(url: string): void {
    if (this.isConnected) {
      throw new Error('Cannot change URL while connected. Disconnect first.');
    }
    this.wsUrl = url;
  }

  /**
   * Get current WebSocket URL
   */
  getUrl(): string {
    return this.wsUrl;
  }

  /**
   * Enable/disable auto-reconnect
   */
  setAutoReconnect(enabled: boolean): void {
    this.autoReconnect = enabled;
  }

  /**
   * Shutdown WebSocket connection
   */
  async shutdown(): Promise<void> {
    console.log('[ROS2WebSocketBridge] Shutting down...');
    
    // Disable auto-reconnect
    this.autoReconnect = false;
    
    // Clear reconnect timer
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    
    // Unsubscribe from all topics
    for (const [topic, rosTopic] of this.topics) {
      try {
        rosTopic.unsubscribe();
      } catch (error) {
        console.error(`[ROS2WebSocketBridge] Error unsubscribing from ${topic}:`, error);
      }
    }
    this.topics.clear();

    // Close ROS connection
    if (this.ros) {
      try {
        this.ros.close();
      } catch (error) {
        console.error('[ROS2WebSocketBridge] Error closing connection:', error);
      }
    }

    this.isConnected = false;
    this.emit('disconnected');
    
    console.log('[ROS2WebSocketBridge] Shutdown complete');
  }

  /**
   * Convert ROS2 image message to data URI for display
   */
  private convertROS2ImageToDataURI(message: any, messageType: string): string {
    if (messageType.includes('CompressedImage')) {
      // Compressed image - data is base64 encoded string or uint8 array
      const imageData = message.data;
      
      // If it's already a base64 string
      if (typeof imageData === 'string') {
        const mimeType = message.format?.includes('jpeg') || message.format?.includes('jpg') 
          ? 'image/jpeg' 
          : message.format?.includes('png') 
            ? 'image/png' 
            : 'image/jpeg';
        return `data:${mimeType};base64,${imageData}`;
      }
      
      // If it's a uint8 array, convert to base64
      if (imageData instanceof Uint8Array || Array.isArray(imageData)) {
        const base64 = btoa(String.fromCharCode.apply(null, Array.from(imageData)));
        const mimeType = message.format?.includes('jpeg') || message.format?.includes('jpg') 
          ? 'image/jpeg' 
          : message.format?.includes('png') 
            ? 'image/png' 
            : 'image/jpeg';
        return `data:${mimeType};base64,${base64}`;
      }
    }
    
    // For raw images (sensor_msgs/Image), convert based on encoding
    return this.convertRawImageToDataURI(message);
  }

  /**
   * Convert raw ROS2 image to data URI
   * Supports: rgb8, rgba8, bgr8, bgra8, mono8, mono16
   */
  private convertRawImageToDataURI(message: any): string {
    const { width, height, encoding, data } = message;
    
    if (!width || !height || !data || !encoding) {
      console.error('[ROS2WebSocketBridge] Invalid image message:', { width, height, encoding });
      return this.getPlaceholderImage('Invalid Image Data');
    }

    try {
      // rosbridge sends data as base64 string, decode it first
      let imageData: number[];
      
      if (typeof data === 'string') {
        // Decode base64 string to byte array
        const binaryString = Buffer.from(data, 'base64');
        imageData = Array.from(binaryString);
      } else if (Array.isArray(data)) {
        imageData = data;
      } else if (data instanceof Uint8Array) {
        imageData = Array.from(data);
      } else {
        console.error('[ROS2WebSocketBridge] Unknown data type:', typeof data);
        return this.getPlaceholderImage('Unknown Data Type');
      }
      
      // Convert to RGBA format
      const rgba = this.convertToRGBA(imageData, encoding, width, height);
      
      // Return RGBA data as base64 (for direct ImageData usage)
      // This is more efficient than BMP encoding
      const base64 = Buffer.from(rgba).toString('base64');
      return `data:image/x-rgba;base64,${base64}`;
      
    } catch (error) {
      console.error('[ROS2WebSocketBridge] Failed to convert raw image:', error);
      return this.getPlaceholderImage('Decoding Error');
    }
  }

  /**
   * Convert various image encodings to RGBA
   */
  private convertToRGBA(data: number[], encoding: string, width: number, height: number): Uint8ClampedArray {
    const pixelCount = width * height;
    const rgba = new Uint8ClampedArray(pixelCount * 4);

    switch (encoding.toLowerCase()) {
      case 'rgb8':
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4] = data[i * 3];       // R
          rgba[i * 4 + 1] = data[i * 3 + 1]; // G
          rgba[i * 4 + 2] = data[i * 3 + 2]; // B
          rgba[i * 4 + 3] = 255;             // A
        }
        break;

      case 'rgba8':
        for (let i = 0; i < pixelCount * 4; i++) {
          rgba[i] = data[i];
        }
        break;

      case 'bgr8':
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4] = data[i * 3 + 2];     // R (from B)
          rgba[i * 4 + 1] = data[i * 3 + 1]; // G
          rgba[i * 4 + 2] = data[i * 3];     // B (from R)
          rgba[i * 4 + 3] = 255;             // A
        }
        break;

      case 'bgra8':
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4] = data[i * 4 + 2];     // R (from B)
          rgba[i * 4 + 1] = data[i * 4 + 1]; // G
          rgba[i * 4 + 2] = data[i * 4];     // B (from R)
          rgba[i * 4 + 3] = data[i * 4 + 3]; // A
        }
        break;

      case 'mono8':
        for (let i = 0; i < pixelCount; i++) {
          const gray = data[i];
          rgba[i * 4] = gray;     // R
          rgba[i * 4 + 1] = gray; // G
          rgba[i * 4 + 2] = gray; // B
          rgba[i * 4 + 3] = 255;  // A
        }
        break;

      case 'mono16':
        for (let i = 0; i < pixelCount; i++) {
          // Convert 16-bit to 8-bit by taking high byte
          const gray = data[i * 2 + 1];
          rgba[i * 4] = gray;     // R
          rgba[i * 4 + 1] = gray; // G
          rgba[i * 4 + 2] = gray; // B
          rgba[i * 4 + 3] = 255;  // A
        }
        break;

      default:
        console.warn(`[ROS2WebSocketBridge] Unsupported encoding: ${encoding}`);
        // Fill with gray
        for (let i = 0; i < pixelCount; i++) {
          rgba[i * 4] = 128;
          rgba[i * 4 + 1] = 128;
          rgba[i * 4 + 2] = 128;
          rgba[i * 4 + 3] = 255;
        }
    }

    return rgba;
  }


  /**
   * Get placeholder image SVG
   */
  private getPlaceholderImage(text: string): string {
    const svg = `<svg width="640" height="480" xmlns="http://www.w3.org/2000/svg">
      <rect width="100%" height="100%" fill="#333"/>
      <text x="50%" y="50%" font-family="Arial" font-size="24" fill="white" text-anchor="middle" dy=".3em">${text}</text>
    </svg>`;
    return `data:image/svg+xml;base64,${Buffer.from(svg).toString('base64')}`;
  }

  /**
   * Convert ROS2 timestamp to ISO string
   */
  private convertROS2Time(stamp: { sec: number; nsec?: number; nanosec?: number }): string {
    const nanosec = stamp.nsec || stamp.nanosec || 0;
    const milliseconds = stamp.sec * 1000 + nanosec / 1000000;
    return new Date(milliseconds).toISOString();
  }
}

