import React, { useState, useEffect } from 'react';
import { ros2Bridge, type TwistMessage } from '../ros2-bridge';
import './TeleopsPanel.css';

interface ControlConfig {
  linearSpeed: number;
  angularSpeed: number;
  publishRate: number; // Hz
}

export const TeleopsPanel: React.FC = () => {
  const [config, setConfig] = useState<ControlConfig>({
    linearSpeed: 0.5,
    angularSpeed: 1.0,
    publishRate: 10
  });
  const [activeKeys, setActiveKeys] = useState<Set<string>>(new Set());
  const [isConnected, setIsConnected] = useState(false);
  const [lastPublished, setLastPublished] = useState<TwistMessage | null>(null);

  // Check connection status
  useEffect(() => {
    const checkConnection = () => {
      setIsConnected(ros2Bridge.isConnected());
    };
    
    checkConnection();
    const interval = setInterval(checkConnection, 1000);
    
    return () => clearInterval(interval);
  }, []);

  // Publish twist messages at configured rate
  useEffect(() => {
    if (!isConnected) return;

    const interval = setInterval(() => {
      const twist = computeTwist(activeKeys, config);
      if (twist) {
        ros2Bridge.publish('/cmd_vel', 'geometry_msgs/Twist', twist);
        setLastPublished(twist);
      }
    }, 1000 / config.publishRate);

    return () => clearInterval(interval);
  }, [activeKeys, config, isConnected]);

  // Keyboard event handlers
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      if (['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
        e.preventDefault();
        setActiveKeys(prev => new Set(prev).add(key));
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      setActiveKeys(prev => {
        const next = new Set(prev);
        next.delete(key);
        return next;
      });
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  const computeTwist = (keys: Set<string>, cfg: ControlConfig): TwistMessage | null => {
    const twist: TwistMessage = {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 }
    };

    let hasMovement = false;

    // Forward/Backward
    if (keys.has('w') || keys.has('arrowup')) {
      twist.linear.x = cfg.linearSpeed;
      hasMovement = true;
    } else if (keys.has('s') || keys.has('arrowdown')) {
      twist.linear.x = -cfg.linearSpeed;
      hasMovement = true;
    }

    // Left/Right turn
    if (keys.has('a') || keys.has('arrowleft')) {
      twist.angular.z = cfg.angularSpeed;
      hasMovement = true;
    } else if (keys.has('d') || keys.has('arrowright')) {
      twist.angular.z = -cfg.angularSpeed;
      hasMovement = true;
    }

    return hasMovement ? twist : null;
  };

  const handleEmergencyStop = () => {
    const stopTwist: TwistMessage = {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 }
    };
    ros2Bridge.publish('/cmd_vel', 'geometry_msgs/Twist', stopTwist);
    setActiveKeys(new Set());
  };

  const isKeyActive = (key: string) => activeKeys.has(key);

  return (
    <div className="teleops-panel">
      <div className="header">
        <h2>Teleoperations Control</h2>
        <div className="connection-status">
          <span className={`status-indicator ${isConnected ? 'connected' : 'disconnected'}`} />
          <span>{isConnected ? 'Connected to ROS2' : 'Disconnected'}</span>
        </div>
      </div>

      <div className="config-panel">
        <div className="config-group">
          <label>Linear Speed (m/s):</label>
          <input
            type="number"
            min="0"
            max="5"
            step="0.1"
            value={config.linearSpeed}
            onChange={(e) => setConfig({ ...config, linearSpeed: Number(e.target.value) })}
          />
        </div>
        <div className="config-group">
          <label>Angular Speed (rad/s):</label>
          <input
            type="number"
            min="0"
            max="5"
            step="0.1"
            value={config.angularSpeed}
            onChange={(e) => setConfig({ ...config, angularSpeed: Number(e.target.value) })}
          />
        </div>
        <div className="config-group">
          <label>Publish Rate (Hz):</label>
          <input
            type="number"
            min="1"
            max="100"
            step="1"
            value={config.publishRate}
            onChange={(e) => setConfig({ ...config, publishRate: Number(e.target.value) })}
          />
        </div>
      </div>

      <div className="control-area">
        <div className="controls-container">
          <div className="control-row">
            <div className="spacer" />
            <button
              className={`control-btn ${isKeyActive('w') || isKeyActive('arrowup') ? 'active' : ''}`}
              disabled={!isConnected}
            >
              ↑<br />Forward
            </button>
            <div className="spacer" />
          </div>
          <div className="control-row">
            <button
              className={`control-btn ${isKeyActive('a') || isKeyActive('arrowleft') ? 'active' : ''}`}
              disabled={!isConnected}
            >
              ←<br />Left
            </button>
            <button
              className="emergency-stop"
              onClick={handleEmergencyStop}
              disabled={!isConnected}
            >
              STOP
            </button>
            <button
              className={`control-btn ${isKeyActive('d') || isKeyActive('arrowright') ? 'active' : ''}`}
              disabled={!isConnected}
            >
              →<br />Right
            </button>
          </div>
          <div className="control-row">
            <div className="spacer" />
            <button
              className={`control-btn ${isKeyActive('s') || isKeyActive('arrowdown') ? 'active' : ''}`}
              disabled={!isConnected}
            >
              ↓<br />Backward
            </button>
            <div className="spacer" />
          </div>
        </div>

        <div className="keyboard-hint">
          <p>Use W/A/S/D or Arrow Keys to control</p>
          <p className="keys">
            <kbd>W</kbd> <kbd>A</kbd> <kbd>S</kbd> <kbd>D</kbd> or 
            <kbd>↑</kbd> <kbd>←</kbd> <kbd>↓</kbd> <kbd>→</kbd>
          </p>
        </div>
      </div>

      {lastPublished && isConnected && (
        <div className="status-panel">
          <h3>Current Command</h3>
          <div className="twist-display">
            <div>Linear: x={lastPublished.linear.x.toFixed(2)}, y={lastPublished.linear.y.toFixed(2)}, z={lastPublished.linear.z.toFixed(2)}</div>
            <div>Angular: x={lastPublished.angular.x.toFixed(2)}, y={lastPublished.angular.y.toFixed(2)}, z={lastPublished.angular.z.toFixed(2)}</div>
          </div>
        </div>
      )}
    </div>
  );
};

