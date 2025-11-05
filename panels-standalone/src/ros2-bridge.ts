/**
 * ROS2 Bridge for Standalone Mode
 * Connects directly to rosbridge or Foxglove Bridge WebSocket
 */

export type ConnectionMode = 'rosbridge' | 'foxglove';

export interface ImageMessage {
  topic: string;
  timestamp: string;
  encoding: string;
  width: number;
  height: number;
  data: string; // base64 or data URI
}

class ROS2Bridge {
  private ws: WebSocket | null = null;
  private messageHandlers: Set<(message: ImageMessage) => void> = new Set();
  private currentMode: ConnectionMode = 'rosbridge';
  private subscribedTopics: Set<string> = new Set();
  private reconnectTimeout: number | null = null;

  connect(mode: ConnectionMode = 'rosbridge') {
    this.currentMode = mode;
    const url = mode === 'rosbridge' 
      ? 'ws://172.16.0.2:9091'
      : 'ws://172.16.0.2:8765';

    console.log(`Connecting to ${mode} at ${url}...`);

    if (this.ws) {
      this.ws.close();
    }

    this.ws = new WebSocket(url);

    this.ws.onopen = () => {
      console.log(`Connected to ${mode}`);
      // Resubscribe to topics
      this.subscribedTopics.forEach(topic => this.subscribe(topic));
    };

    this.ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        
        if (mode === 'rosbridge') {
          this.handleRosbridgeMessage(data);
        } else {
          this.handleFoxgloveMessage(data);
        }
      } catch (error) {
        console.error('Failed to parse message:', error);
      }
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    this.ws.onclose = () => {
      console.log(`Disconnected from ${mode}`);
      // Auto-reconnect after 3 seconds
      this.reconnectTimeout = window.setTimeout(() => {
        console.log('Attempting to reconnect...');
        this.connect(mode);
      }, 3000);
    };
  }

  disconnect() {
    if (this.reconnectTimeout) {
      clearTimeout(this.reconnectTimeout);
    }
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  subscribe(topic: string) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      console.warn('Not connected, queueing subscription:', topic);
      this.subscribedTopics.add(topic);
      return;
    }

    this.subscribedTopics.add(topic);

    if (this.currentMode === 'rosbridge') {
      // ROS Bridge protocol
      this.ws.send(JSON.stringify({
        op: 'subscribe',
        topic: topic,
        type: 'sensor_msgs/CompressedImage' // or sensor_msgs/Image
      }));
    } else {
      // Foxglove Bridge protocol
      this.ws.send(JSON.stringify({
        op: 'subscribe',
        subscriptions: [{
          id: Date.now(),
          topic: topic
        }]
      }));
    }

    console.log(`Subscribed to ${topic}`);
  }

  unsubscribe(topic: string) {
    this.subscribedTopics.delete(topic);

    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return;
    }

    if (this.currentMode === 'rosbridge') {
      this.ws.send(JSON.stringify({
        op: 'unsubscribe',
        topic: topic
      }));
    }
  }

  onMessage(handler: (message: ImageMessage) => void) {
    this.messageHandlers.add(handler);
    return () => this.messageHandlers.delete(handler);
  }

  private handleRosbridgeMessage(data: any) {
    if (data.op === 'publish' && data.msg) {
      const msg = data.msg;
      
      // Handle CompressedImage
      if (msg.format || msg.data) {
        const imageMsg: ImageMessage = {
          topic: data.topic,
          timestamp: new Date().toISOString(),
          encoding: msg.format || 'jpeg',
          width: 640, // Default, actual size determined by browser
          height: 480,
          data: `data:image/${msg.format || 'jpeg'};base64,${msg.data}`
        };
        
        this.messageHandlers.forEach(handler => handler(imageMsg));
      }
    }
  }

  private handleFoxgloveMessage(data: any) {
    // Foxglove Bridge sends binary data in a different format
    // TODO: Implement Foxglove message parsing
    console.log('Foxglove message:', data);
  }
}

export const ros2Bridge = new ROS2Bridge();

// Auto-connect on load (using rosbridge by default)
ros2Bridge.connect('rosbridge');

