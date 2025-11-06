// Ported from: lichtblick/packages/suite-base/src/panels/Teleop/TeleopPanel.tsx
// Date: 2025-11-06
// Modifications:
// - Removed PanelExtensionContext framework integration
// - Replaced context.publish() with direct ros2Bridge calls
// - Removed settings tree system, replaced with inline form controls
// - Removed lodash dependency (_.set replaced with spread operator)
// - Added localStorage for config persistence
// - Simplified lifecycle management (no render sync needed)
// - Removed framework components (Stack, EmptyState, ThemeProvider)

import React, { useCallback, useEffect, useLayoutEffect, useState } from 'react';
import { ros2Bridge } from '../../ros2-bridge';
import { DirectionalPad } from './DirectionalPad';
import { geometryMsgOptions } from './constants';
import { DirectionalPadAction, TeleopConfig } from './types';
import './TeleopPanel.css';

const DEFAULT_CONFIG: TeleopConfig = {
  topic: '/cmd_vel',
  publishRate: 10,
  upButton: { field: 'linear-x', value: 1 },
  downButton: { field: 'linear-x', value: -1 },
  leftButton: { field: 'angular-z', value: 1 },
  rightButton: { field: 'angular-z', value: -1 },
};

export function TeleopPanel(): React.JSX.Element {
  // Load config from localStorage or use defaults
  const [config, setConfig] = useState<TeleopConfig>(() => {
    const saved = localStorage.getItem('teleopConfig');
    if (saved) {
      try {
        const parsed = JSON.parse(saved);
        // Merge with defaults to handle missing fields
        return { ...DEFAULT_CONFIG, ...parsed };
      } catch (e) {
        console.error('Failed to parse saved teleop config:', e);
      }
    }
    return DEFAULT_CONFIG;
  });

  const [currentAction, setCurrentAction] = useState<DirectionalPadAction | undefined>();
  const [isConnected, setIsConnected] = useState(false);
  const [lastMessage, setLastMessage] = useState<Record<string, unknown> | null>(null);

  // Save config to localStorage when it changes
  useEffect(() => {
    localStorage.setItem('teleopConfig', JSON.stringify(config));
  }, [config]);

  // Check ROS connection status
  useEffect(() => {
    const checkConnection = () => {
      setIsConnected(ros2Bridge.isConnected());
    };

    checkConnection();
    const interval = setInterval(checkConnection, 1000);

    return () => {
      clearInterval(interval);
    };
  }, []);

  // Build twist message from current action and config
  const buildTwistMessage = useCallback(
    (action: DirectionalPadAction) => {
      const message = {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      };

      function setFieldValue(field: string, value: number) {
        switch (field) {
          case 'linear-x':
            message.linear.x = value;
            break;
          case 'linear-y':
            message.linear.y = value;
            break;
          case 'linear-z':
            message.linear.z = value;
            break;
          case 'angular-x':
            message.angular.x = value;
            break;
          case 'angular-y':
            message.angular.y = value;
            break;
          case 'angular-z':
            message.angular.z = value;
            break;
        }
      }

      switch (action) {
        case DirectionalPadAction.UP:
          setFieldValue(config.upButton.field, config.upButton.value);
          break;
        case DirectionalPadAction.DOWN:
          setFieldValue(config.downButton.field, config.downButton.value);
          break;
        case DirectionalPadAction.LEFT:
          setFieldValue(config.leftButton.field, config.leftButton.value);
          break;
        case DirectionalPadAction.RIGHT:
          setFieldValue(config.rightButton.field, config.rightButton.value);
          break;
      }

      return message;
    },
    [config],
  );

  // Publish messages when action is active
  useLayoutEffect(() => {
    if (currentAction === undefined || !config.topic) {
      return;
    }

    if (!isConnected) {
      return;
    }

    // Don't publish if rate is 0 or negative - this is a config error
    if (config.publishRate <= 0) {
      return;
    }

    const message = buildTwistMessage(currentAction);
    const intervalMs = 1000 / config.publishRate;

    // Publish immediately
    ros2Bridge.publish(config.topic, 'geometry_msgs/Twist', message);
    setLastMessage(message);

    // Then publish at configured rate
    const intervalHandle = setInterval(() => {
      ros2Bridge.publish(config.topic!, 'geometry_msgs/Twist', message);
      setLastMessage(message);
    }, intervalMs);

    return () => {
      clearInterval(intervalHandle);
    };
  }, [currentAction, config.topic, config.publishRate, isConnected, buildTwistMessage]);

  const canPublish = isConnected && config.publishRate > 0;
  const hasTopic = Boolean(config.topic);
  const enabled = canPublish && hasTopic;

  // Update button configuration
  const updateButton = (
    button: 'upButton' | 'downButton' | 'leftButton' | 'rightButton',
    field: 'field' | 'value',
    value: string | number,
  ) => {
    setConfig((prev) => ({
      ...prev,
      [button]: {
        ...prev[button],
        [field]: value,
      },
    }));
  };

  return (
    <div className="teleop-panel">
      <div className="teleop-header-panel">
        <div className="header-top">
          <div className="header-title-section">
            <h2 className="panel-title">Teleop Control</h2>
            <div className={`connection-indicator ${isConnected ? 'connected' : 'disconnected'}`}>
              <span className="status-dot"></span>
              <span className="status-text">{isConnected ? 'Connected' : 'Disconnected'}</span>
            </div>
          </div>
        </div>
        
        <div className="header-settings">
          <div className="settings-inline">
            <div className="setting-item">
              <label className="setting-label">
                <span className="label-text">Topic</span>
              </label>
              <input
                className="setting-input"
                type="text"
                value={config.topic ?? ''}
                onChange={(e) => setConfig({ ...config, topic: e.target.value })}
                placeholder="/cmd_vel"
              />
            </div>
            <div className="setting-item setting-item-narrow">
              <label className="setting-label">
                <span className="label-text">Rate</span>
                <span className="label-unit">Hz</span>
              </label>
              <input
                className="setting-input"
                type="number"
                min="1"
                max="100"
                value={config.publishRate}
                onChange={(e) => setConfig({ ...config, publishRate: Number(e.target.value) })}
              />
            </div>
          </div>
        </div>

        <details className="advanced-settings">
          <summary>
            <span>Button Mapping Configuration</span>
          </summary>
          <div className="button-config">
            {/* Up Button */}
            <div className="button-config-group">
              <h4>Up Button</h4>
              <div className="button-config-row">
                <div className="setting-group">
                  <label>Field</label>
                  <select
                    value={config.upButton.field}
                    onChange={(e) => updateButton('upButton', 'field', e.target.value)}
                  >
                    {geometryMsgOptions.map((opt) => (
                      <option key={opt.value} value={opt.value}>
                        {opt.label}
                      </option>
                    ))}
                  </select>
                </div>
                <div className="setting-group">
                  <label>Value</label>
                  <input
                    type="number"
                    step="0.1"
                    value={config.upButton.value}
                    onChange={(e) => updateButton('upButton', 'value', Number(e.target.value))}
                  />
                </div>
              </div>
            </div>

            {/* Down Button */}
            <div className="button-config-group">
              <h4>Down Button</h4>
              <div className="button-config-row">
                <div className="setting-group">
                  <label>Field</label>
                  <select
                    value={config.downButton.field}
                    onChange={(e) => updateButton('downButton', 'field', e.target.value)}
                  >
                    {geometryMsgOptions.map((opt) => (
                      <option key={opt.value} value={opt.value}>
                        {opt.label}
                      </option>
                    ))}
                  </select>
                </div>
                <div className="setting-group">
                  <label>Value</label>
                  <input
                    type="number"
                    step="0.1"
                    value={config.downButton.value}
                    onChange={(e) => updateButton('downButton', 'value', Number(e.target.value))}
                  />
                </div>
              </div>
            </div>

            {/* Left Button */}
            <div className="button-config-group">
              <h4>Left Button</h4>
              <div className="button-config-row">
                <div className="setting-group">
                  <label>Field</label>
                  <select
                    value={config.leftButton.field}
                    onChange={(e) => updateButton('leftButton', 'field', e.target.value)}
                  >
                    {geometryMsgOptions.map((opt) => (
                      <option key={opt.value} value={opt.value}>
                        {opt.label}
                      </option>
                    ))}
                  </select>
                </div>
                <div className="setting-group">
                  <label>Value</label>
                  <input
                    type="number"
                    step="0.1"
                    value={config.leftButton.value}
                    onChange={(e) => updateButton('leftButton', 'value', Number(e.target.value))}
                  />
                </div>
              </div>
            </div>

            {/* Right Button */}
            <div className="button-config-group">
              <h4>Right Button</h4>
              <div className="button-config-row">
                <div className="setting-group">
                  <label>Field</label>
                  <select
                    value={config.rightButton.field}
                    onChange={(e) => updateButton('rightButton', 'field', e.target.value)}
                  >
                    {geometryMsgOptions.map((opt) => (
                      <option key={opt.value} value={opt.value}>
                        {opt.label}
                      </option>
                    ))}
                  </select>
                </div>
                <div className="setting-group">
                  <label>Value</label>
                  <input
                    type="number"
                    step="0.1"
                    value={config.rightButton.value}
                    onChange={(e) => updateButton('rightButton', 'value', Number(e.target.value))}
                  />
                </div>
              </div>
            </div>
          </div>
        </details>
      </div>

      <div className="control-section">
        {!canPublish && (
          <div className="empty-state">
            {!isConnected && <p>Connect to a ROS data source to enable control</p>}
            {isConnected && config.publishRate <= 0 && <p>Invalid publish rate configuration</p>}
          </div>
        )}
        {canPublish && !hasTopic && (
          <div className="empty-state">
            <p>Select a publish topic in the settings above</p>
          </div>
        )}
        {enabled && (
          <div className="directional-pad-wrapper">
            <DirectionalPad onAction={setCurrentAction} disabled={!enabled} />
          </div>
        )}
      </div>

      {lastMessage && enabled && (
        <div className="status-panel">
          <h3>Last Published Message</h3>
          <div className="twist-display">
            <div>
              <span className="field-name">linear:</span> x=
              {(lastMessage.linear as { x: number }).x.toFixed(2)}, y=
              {(lastMessage.linear as { y: number }).y.toFixed(2)}, z=
              {(lastMessage.linear as { z: number }).z.toFixed(2)}
            </div>
            <div>
              <span className="field-name">angular:</span> x=
              {(lastMessage.angular as { x: number }).x.toFixed(2)}, y=
              {(lastMessage.angular as { y: number }).y.toFixed(2)}, z=
              {(lastMessage.angular as { z: number }).z.toFixed(2)}
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

