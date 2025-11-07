/**
 * ROS2 Bridge for Standalone Mode
 * Connects directly to rosbridge or Foxglove Bridge WebSocket
 */

export type ConnectionMode = 'rosbridge' | 'foxglove';

export interface ImageMessage {
  topic: string;
  timestamp: string; // ISO string
  timestampNanos?: number; // nanoseconds since epoch
  frameId: string;
  encoding: string;
  width: number;
  height: number;
  data: string; // base64 or data URI
  messageType: 'raw' | 'compressed';
}

export interface TwistMessage {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
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

    // Foxglove requires the subprotocol to be specified
    this.ws = mode === 'foxglove'
      ? new WebSocket(url, 'foxglove.websocket.v1')
      : new WebSocket(url);

    this.ws.onopen = () => {
      console.log(`Connected to ${mode}`);
      // Resubscribe to topics
      this.subscribedTopics.forEach(topic => this.subscribe(topic));
    };

    this.ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        
        if (this.currentMode === 'rosbridge') {
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
      // Auto-reconnect after 3 seconds (use currentMode in case mode changed)
      this.reconnectTimeout = window.setTimeout(() => {
        console.log('Attempting to reconnect...');
        this.connect(this.currentMode);
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

  subscribe(topic: string, messageType?: string) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      console.warn('Not connected, queueing subscription:', topic);
      this.subscribedTopics.add(topic);
      return;
    }

    this.subscribedTopics.add(topic);

    if (this.currentMode === 'rosbridge') {
      // ROS Bridge protocol
      // If no message type specified, let rosbridge auto-detect
      const subscribeMsg: any = {
        op: 'subscribe',
        topic: topic
      };
      
      // Optionally specify type (useful for disambiguation)
      if (messageType) {
        subscribeMsg.type = messageType;
      }
      
      this.ws.send(JSON.stringify(subscribeMsg));
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

    console.log(`Subscribed to ${topic}${messageType ? ` (${messageType})` : ''}`);
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

  publish(topic: string, messageType: string, message: any) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      console.warn('Cannot publish: not connected');
      return;
    }

    if (this.currentMode === 'rosbridge') {
      this.ws.send(JSON.stringify({
        op: 'publish',
        topic: topic,
        type: messageType,
        msg: message
      }));
    } else {
      // Foxglove Bridge publishing
      console.warn('Publishing not yet supported for Foxglove mode');
    }
  }

  isConnected(): boolean {
    return this.ws !== null && this.ws.readyState === WebSocket.OPEN;
  }

  onMessage(handler: (message: ImageMessage) => void) {
    this.messageHandlers.add(handler);
    return () => this.messageHandlers.delete(handler);
  }

  /**
   * Get available image topics
   * TODO: In the future, query rosbridge for actual available topics
   * For now, returns a configurable list
   */
  getAvailableImageTopics(): string[] {
    // Common ROS2 image topic patterns
    return [
      '/camera/image_raw',
      '/camera/image_compressed',
      '/camera/color/image_raw',
      '/camera/color/image_compressed',
      '/camera/depth/image_raw',
      '/camera/rgb/image_raw',
      '/camera/rgb/image_compressed',
      '/usb_cam/image_raw',
      '/usb_cam/image_compressed',
      '/image',
      '/image_raw',
      '/image_compressed'
    ];
  }

  private handleRosbridgeMessage(data: any) {
    if (data.op === 'publish' && data.msg) {
      const msg = data.msg;
      
      // Extract header information (common to both message types)
      const header = msg.header || {};
      const frameId = header.frame_id || '';
      let timestamp = new Date().toISOString();
      let timestampNanos: number | undefined;
      
      // Extract timestamp from header if available
      if (header.stamp) {
        const sec = header.stamp.sec || 0;
        const nanosec = header.stamp.nanosec || 0;
        timestampNanos = sec * 1_000_000_000 + nanosec;
        timestamp = new Date(sec * 1000 + nanosec / 1_000_000).toISOString();
      }
      
      // Handle raw Image messages (sensor_msgs/Image)
      if (msg.width && msg.height && msg.encoding && msg.data && !msg.format) {
        try {
          const dataURI = this.convertRawImageToDataURI(msg);
          const imageMsg: ImageMessage = {
            topic: data.topic,
            timestamp,
            timestampNanos,
            frameId,
            encoding: msg.encoding,
            width: msg.width,
            height: msg.height,
            data: dataURI,
            messageType: 'raw'
          };
          this.messageHandlers.forEach(handler => handler(imageMsg));
        } catch (error) {
          console.error('[ROS2Bridge] Failed to convert raw image:', error);
        }
      }
      
      // Handle compressed Image messages (sensor_msgs/CompressedImage)
      else if (msg.format && msg.data) {
        try {
          this.convertCompressedImageToDataURI(msg, (dataURI, width, height) => {
            const imageMsg: ImageMessage = {
              topic: data.topic,
              timestamp,
              timestampNanos,
              frameId,
              encoding: msg.format, // e.g., "jpeg", "png"
              width,
              height,
              data: dataURI,
              messageType: 'compressed'
            };
            this.messageHandlers.forEach(handler => handler(imageMsg));
          });
        } catch (error) {
          console.error('[ROS2Bridge] Failed to convert compressed image:', error);
        }
      }
    }
  }

  private convertRawImageToDataURI(msg: any): string {
    const { width, height, encoding, data } = msg;

    // Decode base64 data to byte array
    let imageData: Uint8Array;
    if (typeof data === 'string') {
      const binaryString = atob(data);
      imageData = new Uint8Array(binaryString.length);
      for (let i = 0; i < binaryString.length; i++) {
        imageData[i] = binaryString.charCodeAt(i);
      }
    } else if (Array.isArray(data)) {
      imageData = new Uint8Array(data);
    } else if (data instanceof Uint8Array) {
      imageData = data;
    } else {
      throw new Error('Unknown data type');
    }

    // Convert to RGBA
    const rgba = this.convertToRGBA(imageData, encoding, width, height);

    // Create canvas and draw RGBA data
    const canvas = document.createElement('canvas');
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext('2d');
    if (!ctx) {
      throw new Error('Failed to get canvas context');
    }

    const imageDataObj = ctx.createImageData(width, height);
    imageDataObj.data.set(rgba);
    ctx.putImageData(imageDataObj, 0, 0);

    // Return as data URI (JPEG for efficiency)
    return canvas.toDataURL('image/jpeg', 0.92);
  }

  private convertToRGBA(data: Uint8Array, encoding: string, width: number, height: number): Uint8ClampedArray {
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
        rgba.set(data.slice(0, pixelCount * 4));
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
        console.warn(`[ROS2Bridge] Unsupported encoding: ${encoding}`);
        // Fill with gray as fallback
        rgba.fill(128);
        for (let i = 3; i < rgba.length; i += 4) {
          rgba[i] = 255; // Alpha
        }
    }

    return rgba;
  }

  private convertCompressedImageToDataURI(
    msg: any,
    callback: (dataURI: string, width: number, height: number) => void
  ): void {
    const { format, data } = msg;
    
    // Determine MIME type from format
    let mimeType = 'image/jpeg'; // default
    const formatLower = format.toLowerCase();
    if (formatLower.includes('png')) {
      mimeType = 'image/png';
    } else if (formatLower.includes('webp')) {
      mimeType = 'image/webp';
    }
    
    // Create data URI from base64 data
    // rosbridge sends the data already base64 encoded
    const dataURI = `data:${mimeType};base64,${data}`;
    
    // Load image to get dimensions
    const img = new Image();
    img.onload = () => {
      callback(dataURI, img.width, img.height);
    };
    img.onerror = (error) => {
      console.error('[ROS2Bridge] Failed to load compressed image:', error);
    };
    img.src = dataURI;
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

