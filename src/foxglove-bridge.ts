/**
 * Foxglove Bridge for TensorFleet VS Code Extension
 * 
 * Provides ROS2 connectivity via Foxglove WebSocket protocol for:
 * - Image topic subscription (sensor_msgs/Image, sensor_msgs/CompressedImage)
 * - Twist command publishing (geometry_msgs/Twist)
 * - High-performance C++ bridge connection
 * 
 * Uses native Foxglove WebSocket protocol (more efficient than rosbridge)
 */

import { EventEmitter } from 'events';
import WebSocket from 'ws';
import { FoxgloveClient, SubscriptionId, Channel, MessageData } from '@foxglove/ws-protocol';

export class FoxgloveBridge extends EventEmitter {
  private client: FoxgloveClient | null = null;
  private subscriptions: Map<string, SubscriptionId> = new Map();
  private channels: Map<string, Channel> = new Map();
  private isConnected: boolean = false;
  private wsUrl: string;
  private ws: WebSocket | null = null;

  constructor(wsUrl: string = 'ws://172.16.0.2:8765') {
    super();
    this.wsUrl = wsUrl;
  }

  /**
   * Initialize WebSocket connection to Foxglove Bridge
   */
  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      try {
        console.log(`[FoxgloveBridge] Connecting to ${this.wsUrl}...`);

        // Create WebSocket connection with Foxglove subprotocol
        this.ws = new WebSocket(this.wsUrl, [FoxgloveClient.SUPPORTED_SUBPROTOCOL]);

        // Create Foxglove client
        this.client = new FoxgloveClient({
          ws: this.ws as any,
        });

        // Setup event handlers
        this.client.on('open', () => {
          console.log('[FoxgloveBridge] Connected to Foxglove Bridge');
          this.isConnected = true;
          this.emit('connected');
          resolve();
        });

        this.client.on('error', (error: Error) => {
          console.error('[FoxgloveBridge] Connection error:', error);
          this.isConnected = false;
          this.emit('error', error);
          reject(error);
        });

        this.client.on('close', () => {
          console.log('[FoxgloveBridge] Connection closed');
          this.isConnected = false;
          this.emit('disconnected');
        });

        this.client.on('advertise', (channels: Channel[]) => {
          console.log(`[FoxgloveBridge] Received ${channels.length} channel advertisements`);
          for (const channel of channels) {
            this.channels.set(channel.topic, channel);
          }
        });

        this.client.on('unadvertise', (channelIds: number[]) => {
          console.log(`[FoxgloveBridge] Channels unadvertised: ${channelIds.length}`);
          for (const [topic, channel] of this.channels.entries()) {
            if (channelIds.includes(channel.id)) {
              this.channels.delete(topic);
            }
          }
        });

      } catch (error) {
        console.error('[FoxgloveBridge] Failed to initialize:', error);
        this.emit('error', error);
        reject(error);
      }
    });
  }

  /**
   * Check if connected to Foxglove Bridge
   */
  isROS2Connected(): boolean {
    return this.isConnected && this.client !== null;
  }

  /**
   * Subscribe to image topic
   */
  async subscribeToImageTopic(
    topic: string,
    callback: (imageData: string, metadata: any) => void
  ): Promise<void> {
    if (!this.isConnected || !this.client) {
      await this.connect();
    }

    if (!this.client) {
      throw new Error('Failed to connect to Foxglove Bridge');
    }

    // Unsubscribe from existing subscription if any
    if (this.subscriptions.has(topic)) {
      this.unsubscribeFromTopic(topic);
    }

    try {
      // Find the channel for this topic
      const channel = this.channels.get(topic);
      if (!channel) {
        throw new Error(`Channel not found for topic: ${topic}. Available: ${Array.from(this.channels.keys()).join(', ')}`);
      }

      console.log(`[FoxgloveBridge] Subscribing to ${topic} (${channel.schemaName})`);

      // Subscribe to the channel
      const subscriptionId = this.client.subscribe(channel.id);
      this.subscriptions.set(topic, subscriptionId);

      // Handle incoming messages
      this.client.on('message', (event: MessageData) => {
        if (event.subscriptionId === subscriptionId) {
          try {
            // The data is ArrayBufferView, convert to Uint8Array
            const data = new Uint8Array(event.data.buffer, event.data.byteOffset, event.data.byteLength);
            const imageData = this.convertImageToDataURI(data, channel);
            const metadata = {
              topic,
              timestamp: event.timestamp ? new Date(Number(event.timestamp) / 1000000).toISOString() : new Date().toISOString(),
              encoding: channel.encoding,
              width: 0,
              height: 0,
              frame_id: 'unknown'
            };
            
            callback(imageData, metadata);
          } catch (error) {
            console.error(`[FoxgloveBridge] Error processing message from ${topic}:`, error);
          }
        }
      });

      console.log(`[FoxgloveBridge] Successfully subscribed to ${topic}`);
      
    } catch (error) {
      console.error(`[FoxgloveBridge] Failed to subscribe to ${topic}:`, error);
      throw error;
    }
  }

  /**
   * Publish twist message to cmd_vel topic
   */
  async publishTwist(topic: string, twist: any): Promise<void> {
    if (!this.isConnected || !this.client) {
      await this.connect();
    }

    if (!this.client) {
      throw new Error('Failed to connect to Foxglove Bridge');
    }

    try {
      console.log(`[FoxgloveBridge] Publishing to ${topic}:`, twist);
      // Note: Foxglove Bridge client publishing is not yet fully implemented
      // This requires advertising a client channel and sending message data
      console.warn('[FoxgloveBridge] Client publishing not yet implemented');
      
    } catch (error) {
      console.error(`[FoxgloveBridge] Failed to publish to ${topic}:`, error);
      throw error;
    }
  }

  /**
   * Get list of active channels/topics
   */
  async getTopicList(): Promise<Array<{ name: string; type: string }>> {
    if (!this.isConnected || !this.client) {
      await this.connect();
    }

    const topics: Array<{ name: string; type: string }> = [];
    for (const [name, channel] of this.channels.entries()) {
      topics.push({
        name,
        type: channel.schemaName
      });
    }
    
    return topics;
  }

  /**
   * Unsubscribe from a topic
   */
  unsubscribeFromTopic(topic: string): void {
    const subscriptionId = this.subscriptions.get(topic);
    if (subscriptionId !== undefined && this.client) {
      try {
        this.client.unsubscribe(subscriptionId);
        this.subscriptions.delete(topic);
        console.log(`[FoxgloveBridge] Unsubscribed from ${topic}`);
      } catch (error) {
        console.error(`[FoxgloveBridge] Error unsubscribing from ${topic}:`, error);
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
   * Shutdown WebSocket connection
   */
  async shutdown(): Promise<void> {
    console.log('[FoxgloveBridge] Shutting down...');
    
    // Unsubscribe from all topics
    for (const topic of this.subscriptions.keys()) {
      this.unsubscribeFromTopic(topic);
    }
    this.subscriptions.clear();

    // Close connection
    if (this.client) {
      try {
        this.client.close();
      } catch (error) {
        console.error('[FoxgloveBridge] Error closing client:', error);
      }
    }

    if (this.ws) {
      try {
        this.ws.close();
      } catch (error) {
        console.error('[FoxgloveBridge] Error closing WebSocket:', error);
      }
    }

    this.isConnected = false;
    this.channels.clear();
    this.emit('disconnected');
    
    console.log('[FoxgloveBridge] Shutdown complete');
  }

  /**
   * Convert image message to data URI
   * Note: Foxglove protocol sends binary data directly
   */
  private convertImageToDataURI(data: Uint8Array, channel: Channel): string {
    // For compressed images, the data should already be JPEG/PNG
    if (channel.schemaName.includes('CompressedImage')) {
      const base64 = Buffer.from(data).toString('base64');
      const mimeType = 'image/jpeg'; // Most common, could be png too
      return `data:${mimeType};base64,${base64}`;
    }
    
    // For raw images, we'd need to parse the ROS2 Image message structure
    // For now, return a placeholder
    return this.getPlaceholderImage('Raw Image Decoding Not Yet Implemented');
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
}
