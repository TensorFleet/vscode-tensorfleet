import React, { useState, useEffect, useRef } from 'react';
import { vscodeBridge } from '../vscode-bridge';
import './ImagePanel.css';

interface ImageMessage {
  topic: string;
  timestamp: string;
  encoding: string;
  width: number;
  height: number;
  data: string; // base64 or data URI
}

export const ImagePanel: React.FC = () => {
  const [currentImage, setCurrentImage] = useState<ImageMessage | null>(null);
  const [isPaused, setIsPaused] = useState(false);
  const [topics] = useState(['/camera/image_raw', '/camera/compressed', '/depth/image']);
  const [selectedTopic, setSelectedTopic] = useState(topics[0]);
  const [brightness, setBrightness] = useState(100);
  const [contrast, setContrast] = useState(100);
  const [rotation, setRotation] = useState(0);
  const [connectionMode, setConnectionMode] = useState<'auto' | 'rosbridge' | 'foxglove' | 'native'>('rosbridge');
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animationFrameRef = useRef<number | null>(null);
  const pendingImageRef = useRef<ImageMessage | null>(null);

  useEffect(() => {
    // Listen for image data from extension
    const cleanup = vscodeBridge.onMessage((message) => {
      if (message.type === 'imageData' && !isPaused) {
        if (message.topic === selectedTopic || !message.topic) {
          // Store pending image for next animation frame
          pendingImageRef.current = message;
          
          // Request animation frame if not already scheduled
          if (animationFrameRef.current === null) {
            animationFrameRef.current = requestAnimationFrame(() => {
              if (pendingImageRef.current) {
                setCurrentImage(pendingImageRef.current);
                pendingImageRef.current = null;
              }
              animationFrameRef.current = null;
            });
          }
        }
      }
    });

    // Request initial data with connection mode
    vscodeBridge.postMessage({
      command: 'subscribeToTopic',
      topic: selectedTopic,
      connectionMode
    });

    return () => {
      cleanup();
      if (animationFrameRef.current !== null) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [selectedTopic, isPaused, connectionMode]);

  useEffect(() => {
    // Render image to canvas with transformations
    if (currentImage && canvasRef.current) {
      const canvas = canvasRef.current;
      const ctx = canvas.getContext('2d');
      if (!ctx) return;

      // Check if data is RGBA format (optimized path)
      if (currentImage.data.startsWith('data:image/x-rgba;base64,')) {
        try {
          // Decode base64 RGBA data directly
          const base64Data = currentImage.data.split(',')[1];
          const binaryString = atob(base64Data);
          const bytes = new Uint8ClampedArray(binaryString.length);
          for (let i = 0; i < binaryString.length; i++) {
            bytes[i] = binaryString.charCodeAt(i);
          }
          
          // Set canvas size
          canvas.width = currentImage.width;
          canvas.height = currentImage.height;
          
          // Create ImageData directly (much faster than Image element!)
          const imageData = new ImageData(bytes, currentImage.width, currentImage.height);
          
          // If no transformations, use fast path
          if (brightness === 100 && contrast === 100 && rotation === 0) {
            ctx.putImageData(imageData, 0, 0);
          } else {
            // For transformations, use temporary canvas
            const tempCanvas = document.createElement('canvas');
            tempCanvas.width = currentImage.width;
            tempCanvas.height = currentImage.height;
            const tempCtx = tempCanvas.getContext('2d');
            if (tempCtx) {
              tempCtx.putImageData(imageData, 0, 0);
              
              // Apply transformations when drawing to main canvas
              ctx.save();
              ctx.filter = `brightness(${brightness}%) contrast(${contrast}%)`;
              ctx.translate(canvas.width / 2, canvas.height / 2);
              ctx.rotate((rotation * Math.PI) / 180);
              ctx.translate(-canvas.width / 2, -canvas.height / 2);
              ctx.drawImage(tempCanvas, 0, 0);
              ctx.restore();
            }
          }
          
          return;
        } catch (error) {
          console.error('Failed to decode RGBA data:', error);
          // Fall through to Image loading
        }
      }
      
      // Fallback: Load as Image element (for BMP/JPEG/PNG)
      const img = new Image();
      img.onload = () => {
        canvas.width = currentImage.width;
        canvas.height = currentImage.height;

        ctx.save();
        ctx.filter = `brightness(${brightness}%) contrast(${contrast}%)`;
        ctx.translate(canvas.width / 2, canvas.height / 2);
        ctx.rotate((rotation * Math.PI) / 180);
        ctx.translate(-canvas.width / 2, -canvas.height / 2);
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
        ctx.restore();
      };
      img.src = currentImage.data;
    }
  }, [currentImage, brightness, contrast, rotation]);

  const handleTopicChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    setSelectedTopic(e.target.value);
    vscodeBridge.postMessage({
      command: 'subscribeToTopic',
      topic: e.target.value,
      connectionMode
    });
  };

  const handleConnectionModeChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const mode = e.target.value as 'auto' | 'rosbridge' | 'foxglove' | 'native';
    setConnectionMode(mode);
    // Reconnect with new mode
    vscodeBridge.postMessage({
      command: 'setConnectionMode',
      mode
    });
    vscodeBridge.postMessage({
      command: 'subscribeToTopic',
      topic: selectedTopic,
      connectionMode: mode
    });
  };

  const resetTransforms = () => {
    setBrightness(100);
    setContrast(100);
    setRotation(0);
  };

  return (
    <div className="image-panel">
      <div className="controls">
        <div className="control-group">
          <label>Connection:</label>
          <select value={connectionMode} onChange={handleConnectionModeChange}>
            <option value="rosbridge">ROS Bridge (9091)</option>
            <option value="foxglove">Foxglove (8765)</option>
            <option value="native">Native DDS</option>
          </select>
        </div>
        
        <div className="control-group">
          <label>Topic:</label>
          <select value={selectedTopic} onChange={handleTopicChange}>
            {topics.map(topic => (
              <option key={topic} value={topic}>{topic}</option>
            ))}
          </select>
        </div>

        <div className="control-group">
          <button onClick={() => setIsPaused(!isPaused)}>
            {isPaused ? '▶ Resume' : '⏸ Pause'}
          </button>
        </div>

        <div className="control-group">
          <label>Brightness:</label>
          <input
            type="range"
            min="0"
            max="200"
            value={brightness}
            onChange={(e) => setBrightness(Number(e.target.value))}
          />
          <span>{brightness}%</span>
        </div>

        <div className="control-group">
          <label>Contrast:</label>
          <input
            type="range"
            min="0"
            max="200"
            value={contrast}
            onChange={(e) => setContrast(Number(e.target.value))}
          />
          <span>{contrast}%</span>
        </div>

        <div className="control-group">
          <label>Rotation:</label>
          <input
            type="range"
            min="0"
            max="360"
            value={rotation}
            onChange={(e) => setRotation(Number(e.target.value))}
          />
          <span>{rotation}°</span>
        </div>

        <button onClick={resetTransforms}>Reset</button>
      </div>

      <div className="canvas-container">
        <canvas ref={canvasRef} />
        {currentImage && (
          <div className="image-info">
            <div>Topic: {currentImage.topic}</div>
            <div>Size: {currentImage.width}×{currentImage.height}</div>
            <div>Encoding: {currentImage.encoding}</div>
            <div>Timestamp: {new Date(currentImage.timestamp).toLocaleTimeString()}</div>
          </div>
        )}
        {!currentImage && (
          <div className="no-image">
            <p>Waiting for image data...</p>
            <p className="hint">Subscribe to topic: {selectedTopic}</p>
          </div>
        )}
      </div>
    </div>
  );
};

