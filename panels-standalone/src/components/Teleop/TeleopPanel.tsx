// Ported from: lichtblick/packages/suite-base/src/panels/Teleop/TeleopPanel.tsx
// Date: 2025-11-06
// Modifications:
// - Removed PanelExtensionContext framework integration
// - Replaced context.publish() with direct ros2Bridge calls
// - Removed settings tree system, replaced with inline form controls
// - Added localStorage for config persistence
// - Added VM-config teleop profiles for ground robots, TurtleBot4, drones, and custom teleops

import React, { useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState } from 'react';
import { ros2Bridge } from 'tensorfleet-ros';
import { getTopicSuggestions } from '../../utils/discoveredTopics';
import { DirectionalPad } from './DirectionalPad';
import { geometryMsgOptions } from './constants';
import { DirectionalPadAction, TeleopButtonKey, TeleopConfig } from './types';
import {
  getTeleopProfile,
  getTeleopStorageKey,
  KeyboardHint,
  mergeTeleopConfig,
  TeleopPadDefinition,
  TeleopPadId,
  TeleopProfile,
  TeleopServiceAction,
  TeleopTopicConfig,
  TensorFleetVmConfig,
} from './profiles';
import './TeleopPanel.css';
import { ConnectionSettingsProvider, ConnectionSettingsTrigger } from '../ConnectionSettingsProvider';

type TwistVector = {
  x: number;
  y: number;
  z: number;
};

type TwistMessage = {
  linear: TwistVector;
  angular: TwistVector;
};

type TriggerResponse = {
  success?: boolean;
  message?: string;
};

const EDITABLE_TAGS = new Set(['INPUT', 'TEXTAREA', 'SELECT']);

function getActiveVmConfigId(): string {
  return (typeof window !== 'undefined' ? (window as { TENSORFLEET_VM_CONFIG_ID?: string }).TENSORFLEET_VM_CONFIG_ID : '') ?? '';
}

function readWindowVmConfig(): TensorFleetVmConfig | null {
  if (typeof window === 'undefined') {
    return null;
  }

  const config = (window as Window & { TENSORFLEET_VM_CONFIG?: TensorFleetVmConfig }).TENSORFLEET_VM_CONFIG;
  return config && typeof config === 'object' ? config : null;
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

function normalizeKey(key: string): string {
  return key.toLowerCase();
}

function shouldIgnoreKeyboardEvent(target: EventTarget | null): boolean {
  if (!target || !(target instanceof HTMLElement)) {
    return false;
  }

  if (target.isContentEditable) {
    return true;
  }

  return EDITABLE_TAGS.has(target.tagName);
}

function createZeroTwistMessage(): TwistMessage {
  return {
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 },
  };
}

function getPublishMessage(topicType: string | undefined, message: TwistMessage) {
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

function getDisplayedTwist(message: Record<string, unknown> | null): TwistMessage | undefined {
  if (!message) {
    return undefined;
  }

  if ('twist' in message) {
    return message.twist as TwistMessage;
  }

  return message as TwistMessage;
}

function getPadActiveAction(
  pad: TeleopPadDefinition,
  activeBindings: readonly TeleopButtonKey[],
): DirectionalPadAction | undefined {
  if (activeBindings.includes(pad.actions[DirectionalPadAction.UP])) {
    return DirectionalPadAction.UP;
  }
  if (activeBindings.includes(pad.actions[DirectionalPadAction.DOWN])) {
    return DirectionalPadAction.DOWN;
  }
  if (activeBindings.includes(pad.actions[DirectionalPadAction.LEFT])) {
    return DirectionalPadAction.LEFT;
  }
  if (activeBindings.includes(pad.actions[DirectionalPadAction.RIGHT])) {
    return DirectionalPadAction.RIGHT;
  }
  return undefined;
}

function getKeyboardActionMap(profile: TeleopProfile): Record<string, TeleopButtonKey> {
  if (profile.layout === 'drone') {
    return {
      w: 'upButton',
      s: 'downButton',
      a: 'leftButton',
      d: 'rightButton',
      arrowup: 'upButton',
      arrowdown: 'downButton',
      q: 'secondaryLeftButton',
      e: 'secondaryRightButton',
      arrowleft: 'secondaryLeftButton',
      arrowright: 'secondaryRightButton',
      r: 'secondaryUpButton',
      f: 'secondaryDownButton',
      pageup: 'secondaryUpButton',
      pagedown: 'secondaryDownButton',
    };
  }

  if (profile.layout === 'custom') {
    return {};
  }

  return {
    w: 'upButton',
    s: 'downButton',
    arrowup: 'upButton',
    arrowdown: 'downButton',
    a: 'leftButton',
    d: 'rightButton',
    arrowleft: 'leftButton',
    arrowright: 'rightButton',
  };
}

export function TeleopPanel(): React.JSX.Element {
  const vmConfig = useMemo(() => readWindowVmConfig(), []);
  const activeVmConfigId = getActiveVmConfigId() || vmConfig?.id || '';
  const teleopProfile = useMemo(() => getTeleopProfile(activeVmConfigId, vmConfig), [activeVmConfigId, vmConfig]);
  const storageKeyId = activeVmConfigId || teleopProfile.id;
  const storageKey = useMemo(() => getTeleopStorageKey(storageKeyId), [storageKeyId]);

  const [config, setConfig] = useState<TeleopConfig>(() => {
    const saved = localStorage.getItem(storageKey) ?? localStorage.getItem('teleopConfig');
    if (saved) {
      try {
        return mergeTeleopConfig(teleopProfile.defaultConfig, JSON.parse(saved));
      } catch (error) {
        console.error('Failed to parse saved teleop config:', error);
      }
    }

    return teleopProfile.defaultConfig;
  });

  const [padActions, setPadActions] = useState<Partial<Record<TeleopPadId, TeleopButtonKey>>>({});
  const [keyboardActions, setKeyboardActions] = useState<TeleopButtonKey[]>([]);
  const [isConnected, setIsConnected] = useState(false);
  const [lastMessage, setLastMessage] = useState<Record<string, unknown> | null>(null);
  const [discoveredTopics, setDiscoveredTopics] = useState<TeleopTopicConfig[]>([]);
  const [busyServiceId, setBusyServiceId] = useState<string | null>(null);
  const [serviceFeedback, setServiceFeedback] = useState<{
    tone: 'neutral' | 'success' | 'error';
    message: string;
  } | null>(null);
  const [manualControlEnabled, setManualControlEnabled] = useState(false);

  const wasPublishingRef = useRef(false);
  const keyboardActionMap = useMemo(() => getKeyboardActionMap(teleopProfile), [teleopProfile]);

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

  const topicOptions = useMemo(() => {
    if (!config.topic || teleopProfile.topicSelectionMode === 'strict') {
      return availableTopics;
    }

    const currentTopic =
      availableTopics.find((topic) => topic.topic === config.topic) ??
      discoveredTopics.find((topic) => topic.topic === config.topic) ??
      teleopProfile.preferredTopics.find((topic) => topic.topic === config.topic) ??
      { topic: config.topic, type: '', label: 'Current topic' };

    return dedupeTopics([currentTopic, ...availableTopics]);
  }, [availableTopics, config.topic, discoveredTopics, teleopProfile]);

  const selectedTopicType = useMemo(() => {
    return (
      topicOptions.find((topic) => topic.topic === config.topic)?.type ||
      discoveredTopics.find((topic) => topic.topic === config.topic)?.type ||
      teleopProfile.preferredTopics.find((topic) => topic.topic === config.topic)?.type
    );
  }, [config.topic, discoveredTopics, teleopProfile, topicOptions]);

  const orderedActiveBindings = useMemo(() => {
    const activeBindings = new Set<TeleopButtonKey>([
      ...keyboardActions,
      ...Object.values(padActions).filter((value): value is TeleopButtonKey => value !== undefined),
    ]);

    return teleopProfile.buttonDefinitions
      .map((definition) => definition.key)
      .filter((key) => activeBindings.has(key));
  }, [keyboardActions, padActions, teleopProfile.buttonDefinitions]);

  const buildTwistMessage = useCallback(
    (activeBindings: readonly TeleopButtonKey[]) => {
      const message = createZeroTwistMessage();

      const addFieldValue = (field: string, value: number) => {
        switch (field) {
          case 'linear-x':
            message.linear.x += value;
            break;
          case 'linear-y':
            message.linear.y += value;
            break;
          case 'linear-z':
            message.linear.z += value;
            break;
          case 'angular-x':
            message.angular.x += value;
            break;
          case 'angular-y':
            message.angular.y += value;
            break;
          case 'angular-z':
            message.angular.z += value;
            break;
        }
      };

      for (const bindingKey of activeBindings) {
        const binding = config[bindingKey];
        addFieldValue(binding.field, binding.value);
      }

      return message;
    },
    [config],
  );

  const publishZeroTwist = useCallback(() => {
    if (!config.topic || !isConnected) {
      return;
    }

    const zeroMessage = createZeroTwistMessage();
    const publishPayload = getPublishMessage(selectedTopicType, zeroMessage);
    ros2Bridge.publish(config.topic, publishPayload.messageType, publishPayload.message);
    setLastMessage(publishPayload.message);
  }, [config.topic, isConnected, selectedTopicType]);

  useEffect(() => {
    const saved = localStorage.getItem(storageKey) ?? localStorage.getItem('teleopConfig');
    if (saved) {
      try {
        setConfig(mergeTeleopConfig(teleopProfile.defaultConfig, JSON.parse(saved)));
        return;
      } catch (error) {
        console.error('Failed to parse saved teleop config:', error);
      }
    }

    setConfig(teleopProfile.defaultConfig);
  }, [storageKey, teleopProfile]);

  useEffect(() => {
    const serializedConfig = JSON.stringify(config);
    if (serializedConfig) {
      localStorage.setItem(storageKey, serializedConfig);
    }
  }, [config, storageKey]);

  useEffect(() => {
    if (teleopProfile.layout !== 'drone') {
      setManualControlEnabled(false);
      setBusyServiceId(null);
      setServiceFeedback(null);
      setPadActions({});
      setKeyboardActions([]);
    }
  }, [teleopProfile.layout]);

  useEffect(() => {
    setConfig((previous) => {
      if (teleopProfile.topicSelectionMode !== 'strict') {
        return previous;
      }

      const allowedTopics = new Set(teleopProfile.preferredTopics.map((topic) => topic.topic));
      if (previous.topic && allowedTopics.has(previous.topic)) {
        return previous;
      }

      return { ...previous, topic: teleopProfile.defaultConfig.topic };
    });
  }, [teleopProfile]);

  useEffect(() => {
    const checkConnection = () => {
      setIsConnected(ros2Bridge.isConnected());
    };

    checkConnection();
    const interval = setInterval(checkConnection, 1000);
    return () => clearInterval(interval);
  }, []);

  useEffect(() => {
    const updateTopics = () => {
      const topics = getTopicSuggestions().map((topic) => ({ topic: topic.topic, type: topic.type }));
      setDiscoveredTopics(topics);
    };

    updateTopics();
    const interval = setInterval(updateTopics, 1000);
    return () => clearInterval(interval);
  }, []);

  useLayoutEffect(() => {
    if (teleopProfile.layout === 'custom') {
      return;
    }

    if (!config.topic || !isConnected || config.publishRate <= 0 || orderedActiveBindings.length === 0) {
      return;
    }

    const message = buildTwistMessage(orderedActiveBindings);
    const publishPayload = getPublishMessage(selectedTopicType, message);
    const intervalMs = 1000 / config.publishRate;

    ros2Bridge.publish(config.topic, publishPayload.messageType, publishPayload.message);
    setLastMessage(publishPayload.message);

    const intervalHandle = setInterval(() => {
      ros2Bridge.publish(config.topic!, publishPayload.messageType, publishPayload.message);
      setLastMessage(publishPayload.message);
    }, intervalMs);

    return () => {
      clearInterval(intervalHandle);
    };
  }, [buildTwistMessage, config.publishRate, config.topic, isConnected, orderedActiveBindings, selectedTopicType, teleopProfile.layout]);

  const canPublish = isConnected && config.publishRate > 0;
  const hasTopic = Boolean(config.topic);
  const enabled = teleopProfile.layout !== 'custom' && canPublish && hasTopic;
  const teleopInputEnabled = teleopProfile.layout === 'drone' ? enabled && manualControlEnabled : enabled;
  const displayedTwist = getDisplayedTwist(lastMessage);

  useEffect(() => {
    const isPublishing = teleopInputEnabled && orderedActiveBindings.length > 0;

    if (!teleopInputEnabled) {
      if (wasPublishingRef.current) {
        publishZeroTwist();
      }
      wasPublishingRef.current = false;
      return;
    }

    if (!isPublishing && wasPublishingRef.current) {
      publishZeroTwist();
    }

    wasPublishingRef.current = isPublishing;
  }, [orderedActiveBindings, publishZeroTwist, teleopInputEnabled]);

  useEffect(() => {
    if (!teleopInputEnabled) {
      setKeyboardActions([]);
      return;
    }

    const pressedKeys = new Map<string, TeleopButtonKey>();

    const syncPressedBindings = () => {
      const active = new Set<TeleopButtonKey>(pressedKeys.values());
      const ordered = teleopProfile.buttonDefinitions
        .map((definition) => definition.key)
        .filter((key) => active.has(key));

      setKeyboardActions(ordered);
    };

    const handleKeyDown = (event: KeyboardEvent) => {
      if (shouldIgnoreKeyboardEvent(event.target)) {
        return;
      }

      const key = normalizeKey(event.key);
      const action = keyboardActionMap[key];
      if (!action) {
        return;
      }

      event.preventDefault();

      if (pressedKeys.get(key) === action) {
        return;
      }

      pressedKeys.set(key, action);
      syncPressedBindings();
    };

    const handleKeyUp = (event: KeyboardEvent) => {
      const key = normalizeKey(event.key);
      if (!keyboardActionMap[key]) {
        return;
      }

      pressedKeys.delete(key);
      syncPressedBindings();
    };

    const handleWindowBlur = () => {
      pressedKeys.clear();
      syncPressedBindings();
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    window.addEventListener('blur', handleWindowBlur);

    return () => {
      pressedKeys.clear();
      setKeyboardActions([]);
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      window.removeEventListener('blur', handleWindowBlur);
    };
  }, [keyboardActionMap, teleopInputEnabled, teleopProfile.buttonDefinitions]);

  const handlePadAction = useCallback((pad: TeleopPadDefinition, action?: DirectionalPadAction) => {
    setPadActions((previous) => {
      const next = { ...previous };
      if (action === undefined) {
        delete next[pad.id];
      } else {
        next[pad.id] = pad.actions[action];
      }
      return next;
    });
  }, []);

  const invokeService = useCallback(
    async (action: TeleopServiceAction) => {
      if (!isConnected) {
        setServiceFeedback({ tone: 'error', message: 'Connect to a ROS data source to call drone services.' });
        return;
      }

      setBusyServiceId(action.id);
      setServiceFeedback({ tone: 'neutral', message: `Calling ${action.service}...` });

      try {
        const response = await ros2Bridge.callService<TriggerResponse>(action.service, {});
        const succeeded = response?.success === true;
        const message =
          response?.message?.trim() ||
          (succeeded ? `${action.label} succeeded.` : `${action.label} failed.`);

        if (succeeded && action.id === teleopProfile.manualControlActions?.enable.id) {
          setManualControlEnabled(true);
        }

        if (
          succeeded &&
          (action.id === teleopProfile.manualControlActions?.disable.id ||
            action.id === 'land' ||
            action.id === 'disarm')
        ) {
          setManualControlEnabled(false);
        }

        if (action.id === 'stop' || action.id === teleopProfile.manualControlActions?.disable.id) {
          setPadActions({});
          setKeyboardActions([]);
          publishZeroTwist();
        }

        setServiceFeedback({ tone: succeeded ? 'success' : 'error', message });
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        setServiceFeedback({ tone: 'error', message });
      } finally {
        setBusyServiceId(null);
      }
    },
    [isConnected, publishZeroTwist, teleopProfile.manualControlActions],
  );

  const updateButton = useCallback(
    (button: TeleopButtonKey, field: 'field' | 'value', value: string | number) => {
      setConfig((previous) => ({
        ...previous,
        [button]: {
          ...previous[button],
          [field]: value,
        },
      }));
    },
    [],
  );

  const currentManualControlAction = manualControlEnabled
    ? teleopProfile.manualControlActions?.disable
    : teleopProfile.manualControlActions?.enable;
  const flightActions =
    teleopProfile.layout === 'drone'
      ? teleopProfile.serviceActions.filter((action) => action.id !== 'stop' && action.id !== 'disarm')
      : [];
  const safetyActions =
    teleopProfile.layout === 'drone'
      ? teleopProfile.serviceActions.filter((action) => action.id === 'stop' || action.id === 'disarm')
      : [];
  const inputStatusLabel =
    teleopProfile.layout !== 'drone'
      ? enabled
        ? 'Live'
        : 'Unavailable'
      : !enabled
        ? 'Unavailable'
        : manualControlEnabled
          ? 'Live'
          : 'Locked';
  const profileIconClass = teleopProfile.layout === 'drone' ? 'profile-drone' : 'profile-ground';

  return (
    <ConnectionSettingsProvider
      onSettingsChange={(settings) => {
        console.log('Connection settings changed:', settings);
      }}
    >
      <div className="teleop-panel">
        <div className="teleop-toolbar">
          <div className="toolbar-identity" title={teleopProfile.description}>
            <span className={`profile-icon ${profileIconClass}`} />
            <h2 className="toolbar-title">{teleopProfile.title}</h2>
          </div>

          {teleopProfile.layout !== 'custom' && (
            <div className="toolbar-controls">
              <div className="toolbar-field">
                <label className="toolbar-label">Topic</label>
                {topicOptions.length > 0 ? (
                  <select
                    className="toolbar-select"
                    value={config.topic ?? ''}
                    onChange={(event) => setConfig({ ...config, topic: event.target.value })}
                    disabled={teleopProfile.topicSelectionMode === 'strict' && topicOptions.length <= 1}
                  >
                    {topicOptions.map((topic) => (
                      <option key={topic.topic} value={topic.topic}>
                        {topic.label ? `${topic.label} • ${topic.topic}` : topic.topic}
                      </option>
                    ))}
                  </select>
                ) : (
                  <input
                    className="toolbar-input"
                    type="text"
                    value={config.topic ?? ''}
                    onChange={(event) => setConfig({ ...config, topic: event.target.value })}
                    placeholder={teleopProfile.defaultConfig.topic ?? '/cmd_vel'}
                  />
                )}
              </div>

              <div className="toolbar-field toolbar-field-narrow">
                <label className="toolbar-label">Rate</label>
                <div className="rate-wrapper">
                  <input
                    className="toolbar-input rate-input"
                    type="number"
                    min="1"
                    max="100"
                    inputMode="numeric"
                    value={config.publishRate}
                    onChange={(event) => setConfig({ ...config, publishRate: Number(event.target.value) })}
                  />
                  <span className="rate-unit">Hz</span>
                </div>
              </div>
            </div>
          )}

          <div className="toolbar-status">
            <div className={`conn-badge ${isConnected ? 'conn-on' : 'conn-off'}`}>
              <span className="conn-dot" />
              {isConnected ? 'Connected' : 'Disconnected'}
            </div>
            <ConnectionSettingsTrigger />
          </div>
        </div>

        <div className={`teleop-body ${teleopProfile.layout === 'drone' ? 'teleop-body-drone' : ''}`}>
          {teleopProfile.layout === 'custom' && (
            <div className="empty-state">
              <p>{teleopProfile.description}</p>
              <p>Use the config-specific teleop workflow for this VM rather than the generic velocity pad.</p>
            </div>
          )}

          {teleopProfile.layout !== 'custom' && !canPublish && (
            <div className="empty-state">
              <div className="empty-icon">
                <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
                  <circle cx="12" cy="12" r="10" />
                  <line x1="4.93" y1="4.93" x2="19.07" y2="19.07" />
                </svg>
              </div>
              {!isConnected && <p>Connect to a ROS bridge to enable teleop control.</p>}
              {isConnected && config.publishRate <= 0 && <p>Set a valid publish rate to continue.</p>}
            </div>
          )}

          {teleopProfile.layout !== 'custom' && canPublish && !hasTopic && (
            <div className="empty-state">
              <div className="empty-icon">
                <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
                  <path d="M12 2L2 7l10 5 10-5-10-5z" />
                  <path d="M2 17l10 5 10-5" />
                  <path d="M2 12l10 5 10-5" />
                </svg>
              </div>
              <p>Select a publish topic to start.</p>
            </div>
          )}

          {enabled && teleopProfile.layout === 'ground' && teleopProfile.pads[0] && (
            <div className="ground-layout">
              <div className="ground-hud">
                <svg className="hud-rings" viewBox="0 0 300 300" aria-hidden="true">
                  <circle cx="150" cy="150" r="148" />
                  <circle cx="150" cy="150" r="125" />
                  <circle cx="150" cy="150" r="102" />
                  <line x1="150" y1="2" x2="150" y2="298" />
                  <line x1="2" y1="150" x2="298" y2="150" />
                </svg>
                <div className="ground-pad">
                  <DirectionalPad
                    onAction={(action) => handlePadAction(teleopProfile.pads[0]!, action)}
                    disabled={!enabled}
                    activeAction={getPadActiveAction(teleopProfile.pads[0]!, orderedActiveBindings)}
                  />
                </div>
              </div>
            </div>
          )}

          {canPublish && teleopProfile.layout === 'drone' && (
            <div className="drone-layout">
              <div className="drone-pads-row">
                <div className="teleop-ops-card">
                  <div className="ops-header">
                    <h3>Operator Flow</h3>
                    <p>{teleopProfile.description}</p>
                  </div>

                  <div className="ops-status-grid">
                    <div className="ops-status-tile">
                      <span className="ops-status-label">Connection</span>
                      <span className={`ops-status-value ${isConnected ? 'is-on' : 'is-off'}`}>
                        {isConnected ? 'Connected' : 'Disconnected'}
                      </span>
                    </div>
                    <div className="ops-status-tile">
                      <span className="ops-status-label">Manual Control</span>
                      <span className={`ops-status-value ${manualControlEnabled ? 'is-on' : 'is-off'}`}>
                        {manualControlEnabled ? 'On' : 'Off'}
                      </span>
                    </div>
                    <div className="ops-status-tile">
                      <span className="ops-status-label">Pad Input</span>
                      <span className={`ops-status-value ${teleopInputEnabled ? 'is-on' : 'is-idle'}`}>
                        {inputStatusLabel}
                      </span>
                    </div>
                  </div>

                  {currentManualControlAction && (
                    <div className="ops-section">
                      <div className="ops-section-header">
                        <h4>Manual Control</h4>
                        <p>{currentManualControlAction.description}</p>
                      </div>
                      <button
                        type="button"
                        className={`action-btn tone-${currentManualControlAction.tone ?? 'primary'} manual-toggle-btn`}
                        disabled={!isConnected || busyServiceId !== null}
                        onClick={() => void invokeService(currentManualControlAction)}
                      >
                        {busyServiceId === currentManualControlAction.id ? 'Working…' : currentManualControlAction.label}
                      </button>
                    </div>
                  )}

                  <div className="ops-section">
                    <div className="ops-section-header">
                      <h4>Flight</h4>
                      <p>Use these for setup and recovery.</p>
                    </div>
                    <div className="actions-list compact">
                      {flightActions.map((action) => (
                        <button
                          type="button"
                          key={action.id}
                          className={`action-btn tone-${action.tone ?? 'primary'}`}
                          disabled={!isConnected || busyServiceId !== null}
                          onClick={() => void invokeService(action)}
                          title={action.description}
                        >
                          {busyServiceId === action.id ? 'Working…' : action.label}
                        </button>
                      ))}
                    </div>
                  </div>

                  <div className="ops-section danger-zone">
                    <div className="ops-section-header">
                      <h4>Safety</h4>
                      <p>Use only when you need to halt or shut down.</p>
                    </div>
                    <div className="actions-list compact">
                      {safetyActions.map((action) => (
                        <button
                          type="button"
                          key={action.id}
                          className={`action-btn tone-${action.tone ?? 'primary'}`}
                          disabled={!isConnected || busyServiceId !== null}
                          onClick={() => void invokeService(action)}
                          title={action.description}
                        >
                          {busyServiceId === action.id ? 'Working…' : action.label}
                        </button>
                      ))}
                    </div>
                  </div>

                  {serviceFeedback && (
                    <div className={`svc-feedback tone-${serviceFeedback.tone}`}>{serviceFeedback.message}</div>
                  )}
                </div>

                {teleopProfile.pads.map((pad) => (
                  <div className={`drone-pad-card pad-${pad.id}`} key={pad.id}>
                    <div className="pad-card-header">
                      <h3>{pad.title}</h3>
                      <p>{manualControlEnabled ? pad.description : 'Enable manual control to use this pad.'}</p>
                    </div>
                    <div className="pad-card-body">
                      <DirectionalPad
                        onAction={(action) => handlePadAction(pad, action)}
                        disabled={!teleopInputEnabled}
                        activeAction={getPadActiveAction(pad, orderedActiveBindings)}
                      />
                    </div>
                  </div>
                ))}
              </div>
            </div>
          )}
        </div>

        {enabled && (
          <div className="teleop-footer">
            {teleopProfile.keyboardHints.length > 0 && (
              <div className="kbd-strip">
                {teleopProfile.keyboardHints.map((hint: KeyboardHint) => (
                  <span className="kbd-group" key={hint.label}>
                    <span className="kbd-label">{hint.label}</span>
                    {hint.keys.map((key) => (
                      <kbd key={key}>{key}</kbd>
                    ))}
                  </span>
                ))}
              </div>
            )}

            {displayedTwist && (
              <div className="telemetry-strip">
                <span className="telem-vec">
                  <span className="telem-tag">lin</span>
                  <span className="telem-val">{displayedTwist.linear.x.toFixed(1)}</span>
                  <span className="telem-val">{displayedTwist.linear.y.toFixed(1)}</span>
                  <span className="telem-val">{displayedTwist.linear.z.toFixed(1)}</span>
                </span>
                <span className="telem-divider" />
                <span className="telem-vec">
                  <span className="telem-tag">ang</span>
                  <span className="telem-val">{displayedTwist.angular.x.toFixed(1)}</span>
                  <span className="telem-val">{displayedTwist.angular.y.toFixed(1)}</span>
                  <span className="telem-val">{displayedTwist.angular.z.toFixed(1)}</span>
                </span>
              </div>
            )}
          </div>
        )}

        {teleopProfile.showAdvancedMapping && teleopProfile.buttonDefinitions.length > 0 && (
          <details className="advanced-config">
            <summary>
              <span>Command Mapping</span>
            </summary>
            <div className="mapping-grid">
              {teleopProfile.buttonDefinitions.map((definition) => (
                <div className="mapping-item" key={definition.key}>
                  <div className="mapping-label">
                    <span className="mapping-name">{definition.label}</span>
                    <span className="mapping-desc">{definition.description}</span>
                  </div>
                  <div className="mapping-fields">
                    <select
                      className="mapping-select"
                      value={config[definition.key].field}
                      onChange={(event) => updateButton(definition.key, 'field', event.target.value)}
                    >
                      {geometryMsgOptions.map((option) => (
                        <option key={option.value} value={option.value}>
                          {option.label}
                        </option>
                      ))}
                    </select>
                    <input
                      className="mapping-value"
                      type="number"
                      step="0.1"
                      inputMode="decimal"
                      value={config[definition.key].value}
                      onChange={(event) => updateButton(definition.key, 'value', Number(event.target.value))}
                    />
                  </div>
                </div>
              ))}
            </div>
          </details>
        )}
      </div>
    </ConnectionSettingsProvider>
  );
}
