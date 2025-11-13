import React, { useState, useEffect, useRef } from 'react';
import { ros2Bridge, Subscription, type ImageMessage } from '../ros2-bridge';
import { 
  type CameraInfo,
  type ICameraModel, 
  createFallbackCameraModel,
} from '../utils/CameraModel';
import './ImagePanel.css';

export const ImagePanel: React.FC = () => {
  const [currentImage, setCurrentImage] = useState<ImageMessage | null>(null);
  const [isPaused, setIsPaused] = useState(false);
  const [availableSubscriptions, setAvailableSubscriptions] = useState<Subscription[]>([]);
  const [selectedSubscription, setSelectedSubscription] = useState<Subscription|null>(null);
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
  
  // Context menu for download
  const [contextMenu, setContextMenu] = useState<{x: number; y: number} | null>(null);
  
  // Keep camera info type around for the info panel (always using fallback model for now)
  const [cameraInfo] = useState<CameraInfo | null>(null);
  const [cameraModel, setCameraModel] = useState<ICameraModel | null>(null);
  const [show3DAnnotations, setShow3DAnnotations] = useState<boolean>(false);
  
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animationFrameRef = useRef<number | null>(null);
  const pendingImageRef = useRef<ImageMessage | null>(null);

  // Initialize: Load available topics and refresh periodically, auto-select first one
  useEffect(() => {
    const updateImageTopics = () => {
      const imageSubs = ros2Bridge.getAvailableImageTopics();
      setAvailableSubscriptions(imageSubs);

      // Auto-select first topic if none selected yet
      if (!selectedSubscription && imageSubs.length > 0) {
        setSelectedSubscription(imageSubs[0]);
      }
    };

    updateImageTopics();
    const interval = setInterval(updateImageTopics, 1000);
    return () => clearInterval(interval);
  }, [selectedSubscription]);

  useEffect(() => {

    // Check connection status periodically
    const statusInterval = setInterval(() => {
      const isConnected = ros2Bridge.isConnected();
      setConnectionStatus(isConnected ? 'connected' : 'disconnected');
    }, 1000);

    // Only subscribe if we have a selected topic
    if (!selectedSubscription) {
      return () => clearInterval(statusInterval);
    }

    // Subscribe to selected topic
    // ros2Bridge.subscribe(selectedTopic);

    // Listen for image messages
    const cleanup = ros2Bridge.subscribe(
      selectedSubscription,
      (message) => {
      if (!isPaused) {
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
  }, [selectedSubscription, isPaused]);

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

        // Draw 3D annotations on top (if enabled and camera model available)
        if (show3DAnnotations && cameraModel) {
          draw3DAnnotations(ctx);
        }
      };
      
      img.onerror = (error) => {
        setIsLoadingImage(false);
        setErrorMessage('Failed to decode image data');
        console.error('[ImagePanel] Failed to load image:', error);
      };
      
      img.src = currentImage.data;
    };

    renderImage();
  }, [currentImage, brightness, contrast, rotation, flipHorizontal, flipVertical, panOffset, zoomLevel, show3DAnnotations, cameraModel]);

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
    if (!selectedSubscription || connectionStatus !== 'connected') {
      return;
    }

    const timeoutCheck = setInterval(() => {
      const timeSinceLastMessage = Date.now() - lastMessageTime;
      if (timeSinceLastMessage > 10000 && !currentImage) {
        // No messages for 10 seconds and no current image
        setErrorMessage(`No messages received on ${selectedSubscription} for ${Math.floor(timeSinceLastMessage / 1000)}s`);
      }
    }, 2000);

    return () => clearInterval(timeoutCheck);
  }, [selectedSubscription, connectionStatus, lastMessageTime, currentImage]);

  const handleTopicChange = (sub: Subscription | null) => {

    // ros2Bridge.unsubscribe(selectedSubscription);
    // ros2Bridge.subscribe(newTopic);
    setSelectedSubscription(sub);

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

  // Project 3D point to 2D pixel coordinates (Phase 4.3)
  const projectPoint = (point3D: { x: number; y: number; z: number }) => {
    if (!cameraModel) {
      console.warn('No camera model available for projection');
      return null;
    }
    return cameraModel.project(point3D);
  };

  // Example: Draw 3D annotations on canvas (Phase 4.3)
  // This demonstrates how to use the projection feature
  // In a real application, you'd receive 3D points from ROS messages
  const draw3DAnnotations = (ctx: CanvasRenderingContext2D) => {
    if (!cameraModel) return;

    // Example 3D points (in camera coordinate frame)
    // You would replace these with actual 3D data from your ROS topics
    const example3DPoints = [
      { x: 0.5, y: 0, z: 2, label: 'Point A' },
      { x: -0.5, y: 0, z: 2, label: 'Point B' },
      { x: 0, y: 0.5, z: 3, label: 'Point C' },
    ];

    example3DPoints.forEach(point => {
      const pixel = projectPoint(point);
      if (!pixel) return; // Behind camera or outside image

      // Draw circle at projected location
      ctx.save();
      ctx.fillStyle = 'red';
      ctx.strokeStyle = 'white';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(pixel.u, pixel.v, 5, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();

      // Draw label
      ctx.fillStyle = 'white';
      ctx.strokeStyle = 'black';
      ctx.lineWidth = 3;
      ctx.font = '12px Arial';
      ctx.strokeText(point.label, pixel.u + 8, pixel.v - 8);
      ctx.fillText(point.label, pixel.u + 8, pixel.v - 8);
      ctx.restore();
    });
  };

  // Download image with all transforms applied - ported from Lichtblick ImageMode
  const downloadImage = async () => {
    if (!currentImage) {
      console.warn('No image available to download');
      return;
    }

    try {
      // Calculate output dimensions (rotation by 90°/270° swaps width/height)
      const isRotated90or270 = rotation === 90 || rotation === 270;
      const outputWidth = isRotated90or270 ? currentImage.height : currentImage.width;
      const outputHeight = isRotated90or270 ? currentImage.width : currentImage.height;

      // Create offscreen canvas for export
      const canvas = document.createElement('canvas');
      canvas.width = outputWidth;
      canvas.height = outputHeight;
      const ctx = canvas.getContext('2d');
      if (!ctx) {
        throw new Error('Unable to create rendering context for image download');
      }

      // Load image as bitmap
      const img = new Image();
      img.src = currentImage.data;
      await new Promise<void>((resolve, reject) => {
        img.onload = () => resolve();
        img.onerror = reject;
      });

      // Create bitmap (needed for transform support)
      const bitmap = await createImageBitmap(img);

      // Apply transforms (order is critical!)
      ctx.save();
      
      // Translate to center (so rotation/flip happen around center)
      ctx.translate(outputWidth / 2, outputHeight / 2);
      
      // Apply flip
      ctx.scale(
        flipHorizontal ? -1 : 1,
        flipVertical ? -1 : 1
      );
      
      // Apply rotation
      ctx.rotate((rotation * Math.PI) / 180);
      
      // Apply brightness/contrast filter
      const brightnessValue = (brightness / 100) * 1.2 - 0.6; // Maps to -0.6 to 0.6
      const contrastValue = (contrast / 100) * 1.8 + 0.1; // Maps to 0.1 to 1.9
      ctx.filter = `brightness(${brightnessValue + 1}) contrast(${contrastValue})`;
      
      // Translate back and draw centered
      ctx.translate(-currentImage.width / 2, -currentImage.height / 2);
      ctx.drawImage(bitmap, 0, 0);
      
      ctx.restore();

      // Export as PNG blob
      const blob = await new Promise<Blob>((resolve, reject) => {
        canvas.toBlob((result) => {
          if (result) {
            resolve(result);
          } else {
            reject(new Error(`Failed to create image from ${outputWidth}x${outputHeight} canvas`));
          }
        }, 'image/png');
      });

      // Generate filename with topic and timestamp
      const topicName = selectedSubscription?.topic.replace(/^\/+/, '').replace(/\//g, '_');
      const timestamp = currentImage.timestamp || Date.now();
      const fileName = `${topicName}-${timestamp}.png`;

      // Trigger download
      const url = URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      link.download = fileName;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      URL.revokeObjectURL(url);

      console.log(`Downloaded image: ${fileName}`);
    } catch (error) {
      console.error('[ImagePanel] Failed to download image:', error);
      setErrorMessage(`Download failed: ${(error as Error).message}`);
    }
  };

  // Handle context menu (right-click)
  const handleContextMenu = (e: React.MouseEvent<HTMLCanvasElement>) => {
    e.preventDefault();
    setContextMenu({ x: e.clientX, y: e.clientY });
  };

  // Close context menu
  const closeContextMenu = () => {
    setContextMenu(null);
  };

  // Close context menu when clicking outside
  useEffect(() => {
    if (!contextMenu) return;

    const handleClick = () => closeContextMenu();
    document.addEventListener('click', handleClick);
    return () => document.removeEventListener('click', handleClick);
  }, [contextMenu]);

  // Create/update camera model (Phase 4.2)
  useEffect(() => {
    if (!currentImage) {
      setCameraModel(null);
      return;
    }

    // Create fallback camera model (Phase 4.2)
    const fallbackModel = createFallbackCameraModel(
      currentImage.width,
      currentImage.height,
      currentImage.frameId || 'camera',
      500 // Default focal length
    );
    setCameraModel(fallbackModel);
  }, [currentImage]);

  return (
    <div className="image-panel">
      {/* HEADER PANEL */}
      <div className="image-panel-header-panel">
        {/* Layer 1: Title + Status */}
        <div className="header-top">
          <div className="header-title-section">
            <h2 className="panel-title">Image Viewer</h2>
            <div className={`connection-indicator ${connectionStatus === 'connected' ? 'connected' : 'disconnected'}`}>
              <span className="status-dot"></span>
              <span className="status-text">
                {connectionStatus === 'connected' ? 'Connected' : connectionStatus === 'connecting' ? 'Connecting' : 'Disconnected'}
              </span>
            </div>
          </div>
        </div>
        
        {/* Layer 2: Primary Settings */}
          <div className="header-settings">
          <div className="settings-inline">
            <div className="setting-item">
              <label className="setting-label">
                <span className="label-text">Image Topic</span>
              </label>
              <select
                className="setting-input"
                value={selectedSubscription?.topic}
                onChange={(e) => {
                  const topic = e.target.value;
                  const sub = availableSubscriptions.find(s => s.topic === topic) ?? null;
                  handleTopicChange(sub);
                }}
                disabled={availableSubscriptions.length === 0}
              >
                {availableSubscriptions.length === 0 && (
                  <option value="">No image topics available</option>
                )}
                {availableSubscriptions.map(sub => (
                  <option key={sub.topic} value={sub.topic}>
                    {sub.topic}
                  </option>
                ))}
              </select>
            </div>
          </div>
        </div>

        {/* Layer 3: Advanced Settings */}
        <details className="advanced-settings">
          <summary>
            <span>Display & Transform Options</span>
          </summary>
          
          <div className="button-config">
            <div className="button-config-group">
              <h4>Image Adjustments</h4>
              <div className="setting-group">
                <label>Brightness: {brightness}%</label>
                <input
                  type="range"
                  min="0"
                  max="200"
                  value={brightness}
                  onChange={(e) => setBrightness(Number(e.target.value))}
                />
              </div>
              <div className="setting-group">
                <label>Contrast: {contrast}%</label>
                <input
                  type="range"
                  min="0"
                  max="200"
                  value={contrast}
                  onChange={(e) => setContrast(Number(e.target.value))}
                />
              </div>
            </div>

            <div className="button-config-group">
              <h4>Orientation</h4>
              <div className="setting-group">
                <label>Rotation: {rotation}°</label>
                <input
                  type="range"
                  min="0"
                  max="360"
                  value={rotation}
                  onChange={(e) => setRotation(Number(e.target.value))}
                />
              </div>
              <div className="button-config-row">
                <button 
                  className={`transform-button ${flipHorizontal ? 'active' : ''}`}
                  onClick={() => setFlipHorizontal(!flipHorizontal)}
                >
                  ↔️ Flip H
                </button>
                <button 
                  className={`transform-button ${flipVertical ? 'active' : ''}`}
                  onClick={() => setFlipVertical(!flipVertical)}
                >
                  ↕️ Flip V
                </button>
              </div>
            </div>

            <div className="button-config-group">
              <h4>View Controls</h4>
              <div className="setting-group">
                <label>Zoom: {zoomLevel.toFixed(2)}x</label>
                <div className="button-config-row">
                  <button className="view-button" onClick={resetView}>Reset View</button>
                  <button className="view-button" onClick={resetTransforms}>Reset All</button>
                </div>
              </div>
              <div className="setting-group">
                <label>3D Annotations</label>
                <button 
                  className={`transform-button ${show3DAnnotations ? 'active' : ''}`}
                  onClick={() => setShow3DAnnotations(!show3DAnnotations)} 
                  disabled={!cameraModel}
                >
                  {show3DAnnotations ? '🎯 Hide Points' : '🎯 Show Points'}
                </button>
              </div>
            </div>
          </div>
        </details>
      </div>

      <div className="canvas-container">
        <canvas ref={canvasRef} onContextMenu={handleContextMenu} />
        
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
            
            {cameraModel && (
              <>
                <div style={{ marginTop: '8px', borderTop: '1px solid rgba(255,255,255,0.3)', paddingTop: '8px' }}>
                  <strong>Camera Model:</strong>
                </div>
                <div>Focal Length: fx={cameraModel.fx.toFixed(1)}, fy={cameraModel.fy.toFixed(1)}</div>
                <div>Principal Point: ({cameraModel.cx.toFixed(1)}, {cameraModel.cy.toFixed(1)})</div>
                <div>Distortion: {cameraModel.distortionModel}{cameraInfo ? '' : ' (fallback)'}</div>
                {cameraModel.D.length > 0 && cameraModel.D.some(d => d !== 0) && (
                  <div>Coefficients: [{cameraModel.D.slice(0, 5).map(d => d.toFixed(4)).join(', ')}]</div>
                )}
              </>
            )}
          </div>
        )}
        {!currentImage && (
          <div className="no-image">
            {availableSubscriptions.length === 0 ? (
              <>
                <p>No image topics available</p>
                <p className="hint">Please configure image topics in the bridge</p>
              </>
            ) : !selectedSubscription ? (
              <>
                <p>No topic selected</p>
                <p className="hint">Please select an image topic from the dropdown</p>
              </>
            ) : (
              <>
                <p>Waiting for image data...</p>
                <p className="hint">Connecting to rosbridge - topic: {selectedSubscription.topic}</p>
              </>
            )}
          </div>
        )}

        {/* Context Menu for Download */}
        {contextMenu && (
          <div
            className="context-menu"
            style={{
              position: 'fixed',
              left: `${contextMenu.x}px`,
              top: `${contextMenu.y}px`,
              zIndex: 1000,
            }}
            onClick={(e) => e.stopPropagation()}
          >
            <button
              className="context-menu-item"
              onClick={() => {
                downloadImage();
                closeContextMenu();
              }}
              disabled={!currentImage}
            >
              📥 Download Image
            </button>
          </div>
        )}
      </div>
    </div>
  );
};
