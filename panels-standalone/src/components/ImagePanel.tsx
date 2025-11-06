import React, { useState, useEffect, useRef } from 'react';
import { ros2Bridge, type ImageMessage } from '../ros2-bridge';
import './ImagePanel.css';

export const ImagePanel: React.FC = () => {
  const [currentImage, setCurrentImage] = useState<ImageMessage | null>(null);
  const [isPaused, setIsPaused] = useState(false);
  const [availableTopics, setAvailableTopics] = useState<string[]>([]);
  const [selectedTopic, setSelectedTopic] = useState<string>('');
  // Brightness/Contrast: 50 = neutral (maps to 0 brightness, 1.0 contrast)
  const [brightness, setBrightness] = useState(50);
  const [contrast, setContrast] = useState(50);
  const [rotation, setRotation] = useState(0);
  const [flipHorizontal, setFlipHorizontal] = useState(false);
  const [flipVertical, setFlipVertical] = useState(false);
  const [isLoadingImage, setIsLoadingImage] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState<'connected' | 'connecting' | 'disconnected'>('connecting');
  const [errorMessage, setErrorMessage] = useState<string | null>(null);
  const [lastMessageTime, setLastMessageTime] = useState<number>(Date.now());
  
  // Pan & Zoom state - ported from Lichtblick ImageMode
  const [panOffset, setPanOffset] = useState({ x: 0, y: 0 });
  const [zoomLevel, setZoomLevel] = useState(1);
  const dragStartPanOffset = useRef({ x: 0, y: 0 });
  const dragStartMouseCoords = useRef({ x: 0, y: 0 });
  const isDragging = useRef(false);
  
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animationFrameRef = useRef<number | null>(null);
  const pendingImageRef = useRef<ImageMessage | null>(null);

  // Initialize: Load available topics and auto-select first one
  useEffect(() => {
    const topics = ros2Bridge.getAvailableImageTopics();
    setAvailableTopics(topics);
    
    // Auto-select first topic if available
    if (topics.length > 0 && !selectedTopic) {
      setSelectedTopic(topics[0]);
    }
  }, []);

  useEffect(() => {
    // Ensure connection to rosbridge (single supported mode)
    ros2Bridge.connect('rosbridge');

    // Check connection status periodically
    const statusInterval = setInterval(() => {
      const isConnected = ros2Bridge.isConnected();
      setConnectionStatus(isConnected ? 'connected' : 'disconnected');
    }, 1000);

    // Only subscribe if we have a selected topic
    if (!selectedTopic) {
      return () => clearInterval(statusInterval);
    }

    // Subscribe to selected topic
    ros2Bridge.subscribe(selectedTopic);

    // Listen for image messages
    const cleanup = ros2Bridge.onMessage((message) => {
      if (!isPaused && message.topic === selectedTopic) {
        // Clear any error messages on successful receipt
        setErrorMessage(null);
        setLastMessageTime(Date.now());
        
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
      clearInterval(statusInterval);
      if (animationFrameRef.current !== null) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [selectedTopic, isPaused]);

  useEffect(() => {
    // Render image to canvas with transformations, pan/zoom, and aspect ratio preservation
    const renderImage = () => {
      if (!currentImage || !canvasRef.current) return;
      
      const canvas = canvasRef.current;
      const ctx = canvas.getContext('2d');
      if (!ctx) return;

      setIsLoadingImage(true);

      const img = new Image();
      img.onload = () => {
        setIsLoadingImage(false);
        // Get container dimensions
        const container = canvas.parentElement;
        if (!container) return;
        
        const containerWidth = container.clientWidth;
        const containerHeight = container.clientHeight;
        
        // Set canvas to match container
        canvas.width = containerWidth;
        canvas.height = containerHeight;
        
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Calculate aspect ratio preserved dimensions
        const imgAspect = img.width / img.height;
        const canvasAspect = canvas.width / canvas.height;
        
        let drawWidth, drawHeight;
        if (imgAspect > canvasAspect) {
          // Image is wider - fit to width
          drawWidth = canvas.width;
          drawHeight = canvas.width / imgAspect;
        } else {
          // Image is taller - fit to height
          drawHeight = canvas.height;
          drawWidth = canvas.height * imgAspect;
        }
        
        // Calculate position to center image
        const drawX = (canvas.width - drawWidth) / 2;
        const drawY = (canvas.height - drawHeight) / 2;

        // Apply transformations
        ctx.save();
        
        // Move to center of canvas for pan/zoom
        ctx.translate(canvas.width / 2, canvas.height / 2);
        
        // Apply user zoom (ported from Lichtblick)
        ctx.scale(zoomLevel, zoomLevel);
        
        // Apply pan offset (ported from Lichtblick)
        ctx.translate(panOffset.x / zoomLevel, panOffset.y / zoomLevel);
        
        // Move to center of where image will be drawn (adjusted for zoom/pan)
        ctx.translate((drawX - canvas.width / 2), (drawY - canvas.height / 2));
        ctx.translate(drawWidth / 2, drawHeight / 2);
        
        // Apply flip
        const scaleX = flipHorizontal ? -1 : 1;
        const scaleY = flipVertical ? -1 : 1;
        ctx.scale(scaleX, scaleY);
        
        // Apply rotation
        ctx.rotate((rotation * Math.PI) / 180);
        
        // Apply brightness/contrast filter
        // Map from 0-100 slider range to Lichtblick's internal ranges
        const brightnessValue = (brightness / 100) * 1.2 - 0.6; // Maps to -0.6 to 0.6
        const contrastValue = (contrast / 100) * 1.8 + 0.1; // Maps to 0.1 to 1.9
        ctx.filter = `brightness(${brightnessValue + 1}) contrast(${contrastValue})`;
        
        // Draw image centered at origin
        ctx.drawImage(img, -drawWidth / 2, -drawHeight / 2, drawWidth, drawHeight);
        
        ctx.restore();
      };
      
      img.onerror = (error) => {
        setIsLoadingImage(false);
        setErrorMessage('Failed to decode image data');
        console.error('[ImagePanel] Failed to load image:', error);
      };
      
      img.src = currentImage.data;
    };

    renderImage();
  }, [currentImage, brightness, contrast, rotation, flipHorizontal, flipVertical, panOffset, zoomLevel]);

  // Mouse event handlers for pan and zoom (ported from Lichtblick ImageMode)
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    // Mouse down - start drag
    const handleMouseDown = (e: MouseEvent) => {
      if (e.button !== 0) return; // Only left mouse button
      isDragging.current = true;
      dragStartPanOffset.current = { ...panOffset };
      dragStartMouseCoords.current = { x: e.clientX, y: e.clientY };
      canvas.style.cursor = 'grabbing';
    };

    // Mouse move - update pan during drag
    const handleMouseMove = (e: MouseEvent) => {
      if (!isDragging.current) return;
      
      const dx = e.clientX - dragStartMouseCoords.current.x;
      const dy = e.clientY - dragStartMouseCoords.current.y;
      
      setPanOffset({
        x: dragStartPanOffset.current.x + dx,
        y: dragStartPanOffset.current.y + dy,
      });
    };

    // Mouse up - end drag
    const handleMouseUp = () => {
      if (isDragging.current) {
        isDragging.current = false;
        canvas.style.cursor = 'grab';
      }
    };

    // Wheel - zoom centered on cursor (ported from Lichtblick)
    const handleWheel = (e: WheelEvent) => {
      e.preventDefault();
      
      // Get cursor position relative to canvas
      const rect = canvas.getBoundingClientRect();
      const cursorX = e.clientX - rect.left;
      const cursorY = e.clientY - rect.top;
      
      // Clamp wheel delta (from Lichtblick: -30 to 30)
      const clampedDelta = Math.max(-30, Math.min(30, e.deltaY));
      const zoomRatio = 1 - 0.01 * clampedDelta;
      
      // Clamp zoom level (from Lichtblick: 0.5 to 50)
      const newZoom = Math.max(0.5, Math.min(50, zoomLevel * zoomRatio));
      const finalRatio = newZoom / zoomLevel;
      
      // Adjust pan offset so zoom is centered around cursor
      const halfWidth = canvas.width / 2;
      const halfHeight = canvas.height / 2;
      
      setPanOffset({
        x: (halfWidth + panOffset.x - cursorX) * finalRatio - halfWidth + cursorX,
        y: (halfHeight + panOffset.y - cursorY) * finalRatio - halfHeight + cursorY,
      });
      
      setZoomLevel(newZoom);
    };

    canvas.style.cursor = 'grab';
    canvas.addEventListener('mousedown', handleMouseDown);
    canvas.addEventListener('mousemove', handleMouseMove);
    canvas.addEventListener('mouseup', handleMouseUp);
    canvas.addEventListener('mouseleave', handleMouseUp);
    canvas.addEventListener('wheel', handleWheel, { passive: false });

    return () => {
      canvas.removeEventListener('mousedown', handleMouseDown);
      canvas.removeEventListener('mousemove', handleMouseMove);
      canvas.removeEventListener('mouseup', handleMouseUp);
      canvas.removeEventListener('mouseleave', handleMouseUp);
      canvas.removeEventListener('wheel', handleWheel);
    };
  }, [panOffset, zoomLevel]);

  // Handle window resize
  useEffect(() => {
    const handleResize = () => {
      // Trigger re-render on resize by creating a synthetic re-render
      if (currentImage && canvasRef.current) {
        const canvas = canvasRef.current;
        const container = canvas.parentElement;
        if (container) {
          canvas.width = container.clientWidth;
          canvas.height = container.clientHeight;
          // The effect above will re-render when canvas dimensions change
        }
      }
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, [currentImage]);

  // Check for message timeout (no messages for 10 seconds)
  useEffect(() => {
    if (!selectedTopic || connectionStatus !== 'connected') {
      return;
    }

    const timeoutCheck = setInterval(() => {
      const timeSinceLastMessage = Date.now() - lastMessageTime;
      if (timeSinceLastMessage > 10000 && !currentImage) {
        // No messages for 10 seconds and no current image
        setErrorMessage(`No messages received on ${selectedTopic} for ${Math.floor(timeSinceLastMessage / 1000)}s`);
      }
    }, 2000);

    return () => clearInterval(timeoutCheck);
  }, [selectedTopic, connectionStatus, lastMessageTime, currentImage]);

  const handleTopicChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const newTopic = e.target.value;
    ros2Bridge.unsubscribe(selectedTopic);
    ros2Bridge.subscribe(newTopic);
    setSelectedTopic(newTopic);
  };

  // Connection mode is fixed to rosbridge; no handler needed

  const resetTransforms = () => {
    setBrightness(50); // Reset to middle value (maps to 0 brightness)
    setContrast(50); // Reset to middle value (maps to 1.0 contrast)
    setRotation(0);
    setFlipHorizontal(false);
    setFlipVertical(false);
  };

  // Reset view (pan & zoom) - ported from Lichtblick
  const resetView = () => {
    setPanOffset({ x: 0, y: 0 });
    setZoomLevel(1);
  };

  return (
    <div className="image-panel">
      <div className="controls">
        
        <div className="control-group">
          <label>Topic:</label>
          <select 
            value={selectedTopic} 
            onChange={handleTopicChange}
            disabled={availableTopics.length === 0}
          >
            {availableTopics.length === 0 && (
              <option value="">No image topics available</option>
            )}
            {availableTopics.map(topic => (
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

        <div className="control-group">
          <label>Zoom:</label>
          <span>{zoomLevel.toFixed(2)}x</span>
        </div>

        <div className="control-group">
          <button onClick={resetView}>Reset View</button>
        </div>

        <button onClick={resetTransforms}>Reset Transforms</button>
      </div>

      <div className="canvas-container">
        <canvas ref={canvasRef} />
        
        {/* HUD Overlays */}
        {connectionStatus === 'disconnected' && (
          <div className="hud-overlay error">
            <p>⚠️ Not connected to ROS2</p>
            <p className="hint">Check if rosbridge_server is running</p>
          </div>
        )}
        
        {errorMessage && (
          <div className="hud-overlay error">
            <p>❌ {errorMessage}</p>
          </div>
        )}
        
        {isLoadingImage && (
          <div className="loading-indicator">
            <p>Decoding image...</p>
          </div>
        )}
        
        {currentImage && (
          <div className="image-info">
            <div>Topic: {currentImage.topic}</div>
            <div>Frame: {currentImage.frameId || 'N/A'}</div>
            <div>Size: {currentImage.width}×{currentImage.height}</div>
            <div>Encoding: {currentImage.encoding}</div>
            <div>Type: {currentImage.messageType}</div>
            <div>Timestamp: {new Date(currentImage.timestamp).toLocaleTimeString()}</div>
          </div>
        )}
        {!currentImage && (
          <div className="no-image">
            {availableTopics.length === 0 ? (
              <>
                <p>No image topics available</p>
                <p className="hint">Please configure image topics in the bridge</p>
              </>
            ) : !selectedTopic ? (
              <>
                <p>No topic selected</p>
                <p className="hint">Please select an image topic from the dropdown</p>
              </>
            ) : (
              <>
                <p>Waiting for image data...</p>
                <p className="hint">Connecting to rosbridge - topic: {selectedTopic}</p>
              </>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

