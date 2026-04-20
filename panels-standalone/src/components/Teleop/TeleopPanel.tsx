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

import React, { useCallback, useEffect, useLayoutEffect, useMemo, useState } from 'react';
import { ros2Bridge } from '../../ros2-bridge';
import { getTopicSuggestions } from '../../utils/discoveredTopics';
import { DirectionalPad } from './DirectionalPad';
import { geometryMsgOptions } from './constants';
import { DirectionalPadAction, TeleopConfig } from './types';
import { getTeleopProfile, getTeleopStorageKey, TeleopTopicConfig } from './profiles';
import './TeleopPanel.css';
import { ConnectionSettingsProvider, ConnectionSettingsTrigger } from '../ConnectionSettingsProvider';

function getActiveVmConfigId(): string {
  return (typeof window !== 'undefined' ? (window as any).TENSORFLEET_VM_CONFIG_ID : '') ?? '';
}

function dedupeTopics(topics: TeleopTopicConfig[]): TeleopTopicConfig[] {
  const seen = new Set<string>();
  return topics.filter((topic) => {
    if (seen.has(topic.topic)) {
      return false;
    }
    seen.add(topic.topic);
    return true;
  });
}

function getPublishMessage(topicType: string | undefined, message: {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}) {
  if (topicType?.includes('TwistStamped')) {
    return {
      messageType: topicType,
      message: {
        header: {
          stamp: { sec: 0, nanosec: 0 },
          frame_id: '',
        },
        twist: message,
      },
    };
  }

  return {
    messageType: topicType ?? 'geometry_msgs/msg/Twist',
    message,
  };
}

function getDisplayedTwist(message: Record<string, unknown> | null) {
  if (!message) {
    return undefined;
  }

  if ('twist' in message) {
    return message.twist as {
      linear: { x: number; y: number; z: number };
      angular: { x: number; y: number; z: number };
    };
  }

  return message as {
    linear: { x: number; y: number; z: number };
    angular: { x: number; y: number; z: number };
  };
}

const KEYBOARD_ACTIONS: Record<string, DirectionalPadAction> = {
  arrowup: DirectionalPadAction.UP,
  w: DirectionalPadAction.UP,
  arrowdown: DirectionalPadAction.DOWN,
  s: DirectionalPadAction.DOWN,
  arrowleft: DirectionalPadAction.LEFT,
  a: DirectionalPadAction.LEFT,
  arrowright: DirectionalPadAction.RIGHT,
  d: DirectionalPadAction.RIGHT,
};

const EDITABLE_TAGS = new Set(['INPUT', 'TEXTAREA', 'SELECT']);

const normalizeKey = (key: string) => key.toLowerCase();

function shouldIgnoreKeyboardEvent(target: EventTarget | null): boolean {
  if (!target || !(target instanceof HTMLElement)) {
    return false;
  }

  if (target.isContentEditable) {
    return true;
  }

  return EDITABLE_TAGS.has(target.tagName);
}

export function TeleopPanel(): React.JSX.Element {
  const activeVmConfigId = getActiveVmConfigId();
  const teleopProfile = useMemo(() => getTeleopProfile(activeVmConfigId), [activeVmConfigId]);
  const storageKey = useMemo(() => getTeleopStorageKey(activeVmConfigId), [activeVmConfigId]);

  // Load config from localStorage or use defaults
  const [config, setConfig] = useState<TeleopConfig>(() => {
    const saved = localStorage.getItem(storageKey) ?? localStorage.getItem('teleopConfig');
    if (saved) {
      try {
        const parsed = JSON.parse(saved);
        return {
          ...teleopProfile.defaultConfig,
          ...parsed,
        };
      } catch (e) {
        console.error('Failed to parse saved teleop config:', e);
      }
    }
    return teleopProfile.defaultConfig;
  });

  const [padAction, setPadAction] = useState<DirectionalPadAction | undefined>();
  const [keyboardAction, setKeyboardAction] = useState<DirectionalPadAction | undefined>();
  const [isConnected, setIsConnected] = useState(false);
  const [lastMessage, setLastMessage] = useState<Record<string, unknown> | null>(null);
  const [discoveredTopics, setDiscoveredTopics] = useState<TeleopTopicConfig[]>([]);
  const activeAction = keyboardAction ?? padAction;

  const availableTopics = useMemo(() => {
    const preferredTopicNames = new Set(teleopProfile.preferredTopics.map((topic) => topic.topic));
    const discoveredCompatibleTopics = discoveredTopics.filter((topic) =>
      teleopProfile.compatibleMessageTypes.includes(topic.type),
    );
    const otherCompatibleTopics = discoveredCompatibleTopics.filter(
      (topic) => !preferredTopicNames.has(topic.topic),
    );

    if (teleopProfile.topicSelectionMode === 'strict') {
      return dedupeTopics(teleopProfile.preferredTopics);
    }

    return dedupeTopics([...teleopProfile.preferredTopics, ...otherCompatibleTopics]);
  }, [discoveredTopics, teleopProfile]);

  const selectedTopicType = useMemo(() => {
    return availableTopics.find((topic) => topic.topic === config.topic)?.type;
  }, [availableTopics, config.topic]);

  // Save config to localStorage when it changes
  useEffect(() => {
    localStorage.setItem(storageKey, JSON.stringify(config));
  }, [config, storageKey]);

  useEffect(() => {
    setConfig((prev) => {
      if (teleopProfile.topicSelectionMode !== 'strict') {
        return prev;
      }

      const allowedTopics = new Set(teleopProfile.preferredTopics.map((topic) => topic.topic));
      if (prev.topic && allowedTopics.has(prev.topic)) {
        return prev;
      }

      return { ...prev, topic: teleopProfile.defaultConfig.topic };
    });
  }, [teleopProfile]);

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

  // Discover available topics for dropdown
  useEffect(() => {
    const updateTopics = () => {
      const topics = getTopicSuggestions().map((t) => ({ topic: t.topic, type: t.type }));
      setDiscoveredTopics(topics);
    };

    updateTopics();

    const interval = setInterval(updateTopics, 1000);
    return () => clearInterval(interval);
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
    if (activeAction === undefined || !config.topic) {
      return;
    }

    if (!isConnected) {
      return;
    }

    // Don't publish if rate is 0 or negative - this is a config error
    if (config.publishRate <= 0) {
      return;
    }

    const message = buildTwistMessage(activeAction);
    const publishPayload = getPublishMessage(selectedTopicType, message);
    const intervalMs = 1000 / config.publishRate;

    // Publish immediately
    ros2Bridge.publish(config.topic, publishPayload.messageType, publishPayload.message);
    setLastMessage(publishPayload.message);

    // Then publish at configured rate
    const intervalHandle = setInterval(() => {
      ros2Bridge.publish(config.topic!, publishPayload.messageType, publishPayload.message);
      setLastMessage(publishPayload.message);
    }, intervalMs);

    return () => {
      clearInterval(intervalHandle);
    };
  }, [activeAction, config.topic, config.publishRate, isConnected, buildTwistMessage, selectedTopicType]);

  const canPublish = isConnected && config.publishRate > 0;
  const hasTopic = Boolean(config.topic);
  const enabled = teleopProfile.layout === 'twist-pad' && canPublish && hasTopic;
  const displayedTwist = getDisplayedTwist(lastMessage);

  useEffect(() => {
    if (!enabled) {
      return;
    }

    const activeKeys: string[] = [];

    const handleKeyDown = (event: KeyboardEvent) => {
      if (!enabled || shouldIgnoreKeyboardEvent(event.target)) {
        return;
      }

      const key = normalizeKey(event.key);
      const action = KEYBOARD_ACTIONS[key];
      if (action === undefined) {
        return;
      }

      if (event.repeat && activeKeys.includes(key)) {
        event.preventDefault();
        return;
      }

      event.preventDefault();

      if (!activeKeys.includes(key)) {
        activeKeys.push(key);
      }

      setKeyboardAction(action);
    };

    const handleKeyUp = (event: KeyboardEvent) => {
      const key = normalizeKey(event.key);
      if (!(key in KEYBOARD_ACTIONS)) {
        return;
      }

      const index = activeKeys.indexOf(key);
      if (index !== -1) {
        activeKeys.splice(index, 1);
      }

      if (activeKeys.length === 0) {
        setKeyboardAction(undefined);
        return;
      }

      const nextActionKey = activeKeys[activeKeys.length - 1];
      const nextAction = KEYBOARD_ACTIONS[nextActionKey];
      setKeyboardAction(nextAction ?? undefined);
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      activeKeys.length = 0;
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      setKeyboardAction(undefined);
    };
  }, [enabled, setKeyboardAction]);

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
    <ConnectionSettingsProvider onSettingsChange={(settings) => {
      // Handle connection settings changes - could trigger reconnection
      console.log('Connection settings changed:', settings);
      // TODO: Implement reconnection logic if needed
    }}>
      <div className="teleop-panel">
        <div className="teleop-header-panel">
          <div className="header-top">
            <div className="header-title-section">
              <h2 className="panel-title">{teleopProfile.title}</h2>
              <div className="header-actions">
                <ConnectionSettingsTrigger />
              </div>
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
                {availableTopics.length > 0 ? (
                  <select
                    className="setting-input setting-select"
                    value={config.topic ?? ''}
                    onChange={(e) => setConfig({ ...config, topic: e.target.value })}
                    disabled={teleopProfile.topicSelectionMode === 'strict' && availableTopics.length <= 1}
                  >
                    {availableTopics.map((topic) => (
                      <option key={topic.topic} value={topic.topic}>
                        {topic.label ? `${topic.label} • ${topic.topic}` : topic.topic}
                      </option>
                    ))}
                  </select>
                ) : (
                  <input
                    className="setting-input"
                    type="text"
                    value={config.topic ?? ''}
                    onChange={(e) => setConfig({ ...config, topic: e.target.value })}
                    placeholder={teleopProfile.defaultConfig.topic ?? '/cmd_vel'}
                    disabled={teleopProfile.layout !== 'twist-pad'}
                  />
                )}
              </div>
              <div className="setting-item setting-item-narrow">
                <label className="setting-label">
                  <span className="label-text">Rate</span>
                  <span className="label-unit">Hz</span>
                </label>
                <input
                  className="setting-input numeric-input"
                  type="number"
                  min="1"
                  max="100"
                  inputMode="numeric"
                  value={config.publishRate}
                  onChange={(e) => setConfig({ ...config, publishRate: Number(e.target.value) })}
                />
              </div>
            </div>
          </div>

          {teleopProfile.showAdvancedMapping && (
            <details className="advanced-settings">
              <summary>
                <span>Button Mapping Configuration</span>
              </summary>
              <div className="button-config">
              {/* Up Button */}
              <div className="button-config-group">
                <h4>Up Button</h4>
                <div className="button-config-row">
                  <div className="setting-group field-group">
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
                  <div className="setting-group value-group">
                    <label>Value</label>
                    <input
                      className="numeric-input"
                      type="number"
                      step="0.1"
                      inputMode="decimal"
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
                  <div className="setting-group field-group">
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
                  <div className="setting-group value-group">
                    <label>Value</label>
                    <input
                      className="numeric-input"
                      type="number"
                      step="0.1"
                      inputMode="decimal"
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
                  <div className="setting-group field-group">
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
                  <div className="setting-group value-group">
                    <label>Value</label>
                    <input
                      className="numeric-input"
                      type="number"
                      step="0.1"
                      inputMode="decimal"
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
                  <div className="setting-group field-group">
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
                  <div className="setting-group value-group">
                    <label>Value</label>
                    <input
                      className="numeric-input"
                      type="number"
                      step="0.1"
                      inputMode="decimal"
                      value={config.rightButton.value}
                      onChange={(e) => updateButton('rightButton', 'value', Number(e.target.value))}
                    />
                  </div>
                </div>
              </div>
              </div>
            </details>
          )}
        </div>

        <div className="control-section">
          {teleopProfile.layout !== 'twist-pad' && (
            <div className="empty-state">
              <p>{teleopProfile.description}</p>
              <p>Use the config-specific teleop workflow for this VM rather than a generic velocity pad.</p>
            </div>
          )}
          {teleopProfile.layout === 'twist-pad' && !canPublish && (
            <div className="empty-state">
              {!isConnected && <p>Connect to a ROS data source to enable control</p>}
              {isConnected && config.publishRate <= 0 && <p>Invalid publish rate configuration</p>}
            </div>
          )}
          {teleopProfile.layout === 'twist-pad' && canPublish && !hasTopic && (
            <div className="empty-state">
              <p>Select a publish topic in the settings above</p>
            </div>
          )}
          {enabled && (
            <div className="directional-pad-wrapper">
              <DirectionalPad
                onAction={setPadAction}
                disabled={!enabled}
                activeAction={activeAction}
              />
            </div>
          )}
        </div>

        {displayedTwist && enabled && (
          <div className="status-panel">
            <h3>Last Published Message</h3>
            <div className="twist-display">
              <div>
                <span className="field-name">linear:</span> x=
                {displayedTwist.linear.x.toFixed(2)}, y=
                {displayedTwist.linear.y.toFixed(2)}, z=
                {displayedTwist.linear.z.toFixed(2)}
              </div>
              <div>
                <span className="field-name">angular:</span> x=
                {displayedTwist.angular.x.toFixed(2)}, y=
                {displayedTwist.angular.y.toFixed(2)}, z=
                {displayedTwist.angular.z.toFixed(2)}
              </div>
            </div>
          </div>
        )}
      </div>
    </ConnectionSettingsProvider>
  );
}
