import React, { useState, useEffect, useRef } from 'react';
import { ros2Bridge, type ImageMessage, type ConnectionMode } from '../ros2-bridge';
import './ImagePanel.css';

export const ImagePanel: React.FC = () => {
  const [currentImage, setCurrentImage] = useState<ImageMessage | null>(null);
  const [isPaused, setIsPaused] = useState(false);
  const [topics] = useState(['/camera/image_raw', '/camera/compressed', '/depth/image']);
  const [selectedTopic, setSelectedTopic] = useState(topics[1]); // Default to compressed
  const [brightness, setBrightness] = useState(100);
  const [contrast, setContrast] = useState(100);
  const [rotation, setRotation] = useState(0);
  const [flipHorizontal, setFlipHorizontal] = useState(false);
  const [flipVertical, setFlipVertical] = useState(false);
  const [connectionMode, setConnectionMode] = useState<ConnectionMode>('rosbridge');
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animationFrameRef = useRef<number | null>(null);
  const pendingImageRef = useRef<ImageMessage | null>(null);

  useEffect(() => {
    // Connect to ROS2
    ros2Bridge.connect(connectionMode);
    
    // Subscribe to selected topic
    ros2Bridge.subscribe(selectedTopic);

    // Listen for image messages
    const cleanup = ros2Bridge.onMessage((message) => {
      if (!isPaused && message.topic === selectedTopic) {
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

      const img = new Image();
      img.onload = () => {
        // Update canvas dimensions
        canvas.width = img.width;
        canvas.height = img.height;

        // Apply transformations
        ctx.save();
        
        // Move to center for transformations
        ctx.translate(canvas.width / 2, canvas.height / 2);
        
        // Apply flip
        const scaleX = flipHorizontal ? -1 : 1;
        const scaleY = flipVertical ? -1 : 1;
        ctx.scale(scaleX, scaleY);
        
        // Apply rotation
        ctx.rotate((rotation * Math.PI) / 180);
        
        // Apply brightness/contrast filter
        ctx.filter = `brightness(${brightness}%) contrast(${contrast}%)`;
        
        // Draw image centered
        ctx.drawImage(img, -canvas.width / 2, -canvas.height / 2, canvas.width, canvas.height);
        
        ctx.restore();
      };
      
      img.src = currentImage.data;
    }
  }, [currentImage, brightness, contrast, rotation, flipHorizontal, flipVertical]);

  const handleTopicChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const newTopic = e.target.value;
    ros2Bridge.unsubscribe(selectedTopic);
    ros2Bridge.subscribe(newTopic);
    setSelectedTopic(newTopic);
  };

  const handleConnectionModeChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const mode = e.target.value as ConnectionMode;
    setConnectionMode(mode);
    ros2Bridge.disconnect();
    ros2Bridge.connect(mode);
    ros2Bridge.subscribe(selectedTopic);
  };

  const resetTransforms = () => {
    setBrightness(100);
    setContrast(100);
    setRotation(0);
    setFlipHorizontal(false);
    setFlipVertical(false);
  };

  return (
    <div className="image-panel">
      <div className="controls">
        <div className="control-group">
          <label>Connection:</label>
          <select value={connectionMode} onChange={handleConnectionModeChange}>
            <option value="rosbridge">ROS Bridge (9091)</option>
            <option value="foxglove">Foxglove (8765)</option>
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

        <div className="control-group">
          <button onClick={() => setFlipHorizontal(!flipHorizontal)}>
            {flipHorizontal ? '↔️ H-Flip: ON' : '↔️ H-Flip: OFF'}
          </button>
        </div>

        <div className="control-group">
          <button onClick={() => setFlipVertical(!flipVertical)}>
            {flipVertical ? '↕️ V-Flip: ON' : '↕️ V-Flip: OFF'}
          </button>
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
            <p className="hint">Connecting to {connectionMode} - topic: {selectedTopic}</p>
          </div>
        )}
      </div>
    </div>
  );
};

