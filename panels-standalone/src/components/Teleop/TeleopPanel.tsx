// Ported from: lichtblick/packages/suite-base/src/panels/Teleop/TeleopPanel.tsx
// Date: 2025-11-06
// Modifications:
// - Removed PanelExtensionContext framework integration
// - Replaced context.publish() with direct ros2Bridge calls
// - Removed settings tree system, replaced with inline form controls
// - Added localStorage for config persistence
// - Added VM-config-driven drone teleops layout and command actions

import React, { useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState } from 'react';
import { ros2Bridge } from '../../ros2-bridge';
import { getTopicSuggestions } from '../../utils/discoveredTopics';
import { DirectionalPad } from './DirectionalPad';
import { geometryMsgOptions } from './constants';
import { DirectionalPadAction, TeleopButtonKey, TeleopConfig } from './types';
import './TeleopPanel.css';
import { ConnectionSettingsProvider, ConnectionSettingsTrigger } from '../ConnectionSettingsProvider';

type TwistMessage = {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
};

type TensorFleetVmConfig = {
  id?: string;
  name?: string;
  description?: string;
  sim_config?: Record<string, unknown>;
};

type TeleopProfileKind = 'ground' | 'drone';
type TeleopPadId = 'primary' | 'secondary';
type ServiceFeedbackTone = 'neutral' | 'success' | 'error';

type TeleopButtonDefinition = {
  key: TeleopButtonKey;
  label: string;
  description: string;
};

type TeleopPadDefinition = {
  id: TeleopPadId;
  title: string;
  description: string;
  actions: Record<DirectionalPadAction, TeleopButtonKey>;
};

type KeyboardHint = {
  label: string;
  keys: string[];
};

type TeleopServiceAction = {
  id: string;
  label: string;
  service: string;
  tone?: 'primary' | 'secondary' | 'danger';
  description?: string;
};

type TeleopProfile = {
  kind: TeleopProfileKind;
  title: string;
  subtitle: string;
  storageKey: string;
  defaultConfig: TeleopConfig;
  buttonDefinitions: TeleopButtonDefinition[];
  pads: TeleopPadDefinition[];
  keyboardHints: KeyboardHint[];
  serviceActions: TeleopServiceAction[];
};

type TriggerResponse = {
  success?: boolean;
  message?: string;
};

const TWIST_TYPE_PATTERNS = ['geometry_msgs/Twist', 'geometry_msgs/msg/Twist'];

function isTwistTopic(type: string): boolean {
  return TWIST_TYPE_PATTERNS.some((pattern) => type === pattern || type.endsWith('/' + pattern));
}

const DEFAULT_GROUND_CONFIG: TeleopConfig = {
  topic: '/cmd_vel_raw',
  publishRate: 10,
  upButton: { field: 'linear-x', value: 1 },
  downButton: { field: 'linear-x', value: -1 },
  leftButton: { field: 'angular-z', value: 1 },
  rightButton: { field: 'angular-z', value: -1 },
  secondaryUpButton: { field: 'linear-z', value: 1 },
  secondaryDownButton: { field: 'linear-z', value: -1 },
  secondaryLeftButton: { field: 'angular-z', value: 1 },
  secondaryRightButton: { field: 'angular-z', value: -1 },
};

const DEFAULT_DRONE_CONFIG: TeleopConfig = {
  topic: '/drone/cmd_vel',
  publishRate: 10,
  upButton: { field: 'linear-x', value: 1 },
  downButton: { field: 'linear-x', value: -1 },
  leftButton: { field: 'linear-y', value: 1 },
  rightButton: { field: 'linear-y', value: -1 },
  secondaryUpButton: { field: 'linear-z', value: 1 },
  secondaryDownButton: { field: 'linear-z', value: -1 },
  secondaryLeftButton: { field: 'angular-z', value: 1 },
  secondaryRightButton: { field: 'angular-z', value: -1 },
};

const GROUND_BUTTONS: TeleopButtonDefinition[] = [
  { key: 'upButton', label: 'Forward', description: 'Linear X+' },
  { key: 'downButton', label: 'Reverse', description: 'Linear X-' },
  { key: 'leftButton', label: 'Turn Left', description: 'Yaw +' },
  { key: 'rightButton', label: 'Turn Right', description: 'Yaw -' },
];

const DRONE_BUTTONS: TeleopButtonDefinition[] = [
  { key: 'upButton', label: 'Forward', description: 'Linear X+' },
  { key: 'downButton', label: 'Backward', description: 'Linear X-' },
  { key: 'leftButton', label: 'Strafe Left', description: 'Linear Y+' },
  { key: 'rightButton', label: 'Strafe Right', description: 'Linear Y-' },
  { key: 'secondaryUpButton', label: 'Climb', description: 'Linear Z+' },
  { key: 'secondaryDownButton', label: 'Descend', description: 'Linear Z-' },
  { key: 'secondaryLeftButton', label: 'Yaw Left', description: 'Angular Z+' },
  { key: 'secondaryRightButton', label: 'Yaw Right', description: 'Angular Z-' },
];

const GROUND_PROFILE: TeleopProfile = {
  kind: 'ground',
  title: 'Teleop Control',
  subtitle: 'Ground robot velocity control using `/cmd_vel_raw`.',
  storageKey: 'teleopConfig:ground',
  defaultConfig: DEFAULT_GROUND_CONFIG,
  buttonDefinitions: GROUND_BUTTONS,
  pads: [
    {
      id: 'primary',
      title: 'Drive',
      description: 'Forward, reverse, and turning.',
      actions: {
        [DirectionalPadAction.UP]: 'upButton',
        [DirectionalPadAction.DOWN]: 'downButton',
        [DirectionalPadAction.LEFT]: 'leftButton',
        [DirectionalPadAction.RIGHT]: 'rightButton',
      },
    },
  ],
  keyboardHints: [
    { label: 'Move', keys: ['W', 'S', '↑', '↓'] },
    { label: 'Turn', keys: ['A', 'D', '←', '→'] },
  ],
  serviceActions: [],
};

const DRONE_SERVICE_ACTIONS: TeleopServiceAction[] = [
  { id: 'arm', label: 'Arm', service: '/drone/arm', description: 'Prepare motors for flight.' },
  { id: 'takeoff', label: 'Take Off', service: '/drone/takeoff', description: 'Lift into controlled flight.' },
  { id: 'land', label: 'Land', service: '/drone/land', tone: 'secondary', description: 'Exit flight and descend.' },
  { id: 'stop', label: 'Stop Motion', service: '/drone/stop', tone: 'secondary', description: 'Send zero motion and clear inputs.' },
  { id: 'disarm', label: 'Disarm', service: '/drone/disarm', tone: 'danger', description: 'Cut motors when safe.' },
];

const ENABLE_MANUAL_CONTROL_ACTION: TeleopServiceAction = {
  id: 'enable_external',
  label: 'Enable Manual Control',
  service: '/drone/enable_external_control',
  tone: 'primary',
  description: 'Switch to OFFBOARD or GUIDED for pad input.',
};

const DISABLE_MANUAL_CONTROL_ACTION: TeleopServiceAction = {
  id: 'disable_external',
  label: 'Exit Manual Control',
  service: '/drone/disable_external_control',
  tone: 'secondary',
  description: 'Hand control back to hold mode.',
};

function createDroneProfile(vmConfig: TensorFleetVmConfig | null): TeleopProfile {
  const vehicleLabel = vmConfig?.name?.trim() || 'Drone';

  return {
    kind: 'drone',
    title: 'Drone Teleops',
    subtitle: `${vehicleLabel} control via \`/drone/cmd_vel\` and VM-side trigger services.`,
    storageKey: `teleopConfig:drone:${vmConfig?.id ?? 'default'}`,
    defaultConfig: DEFAULT_DRONE_CONFIG,
    buttonDefinitions: DRONE_BUTTONS,
    pads: [
      {
        id: 'primary',
        title: 'Planar Velocity',
        description: 'Forward/back and left/right strafe velocity.',
        actions: {
          [DirectionalPadAction.UP]: 'upButton',
          [DirectionalPadAction.DOWN]: 'downButton',
          [DirectionalPadAction.LEFT]: 'leftButton',
          [DirectionalPadAction.RIGHT]: 'rightButton',
        },
      },
      {
        id: 'secondary',
        title: 'Altitude + Yaw',
        description: 'Climb/descend and yaw rate control.',
        actions: {
          [DirectionalPadAction.UP]: 'secondaryUpButton',
          [DirectionalPadAction.DOWN]: 'secondaryDownButton',
          [DirectionalPadAction.LEFT]: 'secondaryLeftButton',
          [DirectionalPadAction.RIGHT]: 'secondaryRightButton',
        },
      },
    ],
    keyboardHints: [
      { label: 'Planar', keys: ['W', 'A', 'S', 'D'] },
      { label: 'Yaw', keys: ['Q', 'E', '←', '→'] },
      { label: 'Altitude', keys: ['R', 'F', 'PgUp', 'PgDn'] },
    ],
    serviceActions: DRONE_SERVICE_ACTIONS,
  };
}

const EDITABLE_TAGS = new Set(['INPUT', 'TEXTAREA', 'SELECT']);

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

function readWindowVmConfig(): TensorFleetVmConfig | null {
  const tensorfleetWindow = window as Window & {
    TENSORFLEET_VM_CONFIG?: TensorFleetVmConfig;
  };

  const config = tensorfleetWindow.TENSORFLEET_VM_CONFIG;
  return config && typeof config === 'object' ? config : null;
}

function isDroneVmConfig(vmConfig: TensorFleetVmConfig | null): boolean {
  if (!vmConfig) {
    return false;
  }

  if (vmConfig.id === 'px4' || vmConfig.id === 'ardupilot') {
    return true;
  }

  const simConfig = vmConfig.sim_config ?? {};
  return simConfig.gazebo_px4_enabled === 'true' || simConfig.gazebo_ardupilot_enabled === 'true';
}

function resolveTeleopProfile(vmConfig: TensorFleetVmConfig | null): TeleopProfile {
  return isDroneVmConfig(vmConfig) ? createDroneProfile(vmConfig) : GROUND_PROFILE;
}

function mergeTeleopConfig(defaultConfig: TeleopConfig, savedConfig: unknown): TeleopConfig {
  if (!savedConfig || typeof savedConfig !== 'object') {
    return defaultConfig;
  }

  const parsed = savedConfig as Partial<TeleopConfig>;

  return {
    ...defaultConfig,
    ...parsed,
    upButton: { ...defaultConfig.upButton, ...parsed.upButton },
    downButton: { ...defaultConfig.downButton, ...parsed.downButton },
    leftButton: { ...defaultConfig.leftButton, ...parsed.leftButton },
    rightButton: { ...defaultConfig.rightButton, ...parsed.rightButton },
    secondaryUpButton: { ...defaultConfig.secondaryUpButton, ...parsed.secondaryUpButton },
    secondaryDownButton: { ...defaultConfig.secondaryDownButton, ...parsed.secondaryDownButton },
    secondaryLeftButton: { ...defaultConfig.secondaryLeftButton, ...parsed.secondaryLeftButton },
    secondaryRightButton: { ...defaultConfig.secondaryRightButton, ...parsed.secondaryRightButton },
  };
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
  if (profile.kind === 'drone') {
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
  const profile = useMemo(() => resolveTeleopProfile(vmConfig), [vmConfig]);

  const [config, setConfig] = useState<TeleopConfig>(() => {
    const saved = localStorage.getItem(profile.storageKey);
    if (saved) {
      try {
        return mergeTeleopConfig(profile.defaultConfig, JSON.parse(saved));
      } catch (error) {
        console.error('Failed to parse saved teleop config:', error);
      }
    }

    return profile.defaultConfig;
  });

  const [padActions, setPadActions] = useState<Partial<Record<TeleopPadId, TeleopButtonKey>>>({});
  const [keyboardActions, setKeyboardActions] = useState<TeleopButtonKey[]>([]);
  const [isConnected, setIsConnected] = useState(false);
  const [lastMessage, setLastMessage] = useState<TwistMessage | null>(null);
  const [availableTopics, setAvailableTopics] = useState<string[]>([]);
  const [busyServiceId, setBusyServiceId] = useState<string | null>(null);
  const [serviceFeedback, setServiceFeedback] = useState<{
    tone: ServiceFeedbackTone;
    message: string;
  } | null>(null);
  const [manualControlEnabled, setManualControlEnabled] = useState(false);

  const wasPublishingRef = useRef(false);
  const keyboardActionMap = useMemo(() => getKeyboardActionMap(profile), [profile]);

  useEffect(() => {
    localStorage.setItem(profile.storageKey, JSON.stringify(config));
  }, [config, profile.storageKey]);

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
      const topics = getTopicSuggestions()
        .filter((t) => isTwistTopic(t.type))
        .map((t) => t.topic);
      setAvailableTopics(topics);
    };

    updateTopics();
    const interval = setInterval(updateTopics, 1000);
    return () => clearInterval(interval);
  }, []);

  const orderedActiveBindings = useMemo(() => {
    const activeBindings = new Set<TeleopButtonKey>([
      ...keyboardActions,
      ...Object.values(padActions).filter((value): value is TeleopButtonKey => value !== undefined),
    ]);

    return profile.buttonDefinitions
      .map((definition) => definition.key)
      .filter((key) => activeBindings.has(key));
  }, [keyboardActions, padActions, profile.buttonDefinitions]);

  const topicOptions = useMemo(() => {
    const topics = new Set(availableTopics);
    if (config.topic) {
      topics.add(config.topic);
    }
    return Array.from(topics);
  }, [availableTopics, config.topic]);

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

  useLayoutEffect(() => {
    if (!config.topic || !isConnected || config.publishRate <= 0 || orderedActiveBindings.length === 0) {
      return;
    }

    const message = buildTwistMessage(orderedActiveBindings);
    const intervalMs = 1000 / config.publishRate;

    ros2Bridge.publish(config.topic, 'geometry_msgs/Twist', message);
    setLastMessage(message);

    const intervalHandle = setInterval(() => {
      ros2Bridge.publish(config.topic!, 'geometry_msgs/Twist', message);
      setLastMessage(message);
    }, intervalMs);

    return () => {
      clearInterval(intervalHandle);
    };
  }, [buildTwistMessage, config.publishRate, config.topic, isConnected, orderedActiveBindings]);

  const canPublish = isConnected && config.publishRate > 0;
  const hasTopic = Boolean(config.topic);
  const enabled = canPublish && hasTopic;
  const teleopInputEnabled = profile.kind === 'drone' ? enabled && manualControlEnabled : enabled;

  useEffect(() => {
    const isPublishing = teleopInputEnabled && orderedActiveBindings.length > 0;

    if (!teleopInputEnabled) {
      wasPublishingRef.current = false;
      return;
    }

    if (!isPublishing && wasPublishingRef.current && config.topic) {
      const zeroMessage = createZeroTwistMessage();
      ros2Bridge.publish(config.topic, 'geometry_msgs/Twist', zeroMessage);
      setLastMessage(zeroMessage);
    }

    wasPublishingRef.current = isPublishing;
  }, [config.topic, orderedActiveBindings, teleopInputEnabled]);

  useEffect(() => {
    if (!teleopInputEnabled) {
      setKeyboardActions([]);
      return;
    }

    const pressedKeys = new Map<string, TeleopButtonKey>();

    const syncPressedBindings = () => {
      const active = new Set<TeleopButtonKey>(pressedKeys.values());
      const ordered = profile.buttonDefinitions
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
  }, [keyboardActionMap, profile.buttonDefinitions, teleopInputEnabled]);

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

  const publishZeroTwist = useCallback(() => {
    if (!config.topic || !isConnected) {
      return;
    }

    const zeroMessage = createZeroTwistMessage();
    ros2Bridge.publish(config.topic, 'geometry_msgs/Twist', zeroMessage);
    setLastMessage(zeroMessage);
  }, [config.topic, isConnected]);

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

        if (succeeded && action.id === 'enable_external') {
          setManualControlEnabled(true);
        }

        if (succeeded && (action.id === 'disable_external' || action.id === 'land' || action.id === 'disarm')) {
          setManualControlEnabled(false);
        }

        if (action.id === 'stop' || action.id === 'disable_external') {
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
    [isConnected, publishZeroTwist],
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
    ? DISABLE_MANUAL_CONTROL_ACTION
    : ENABLE_MANUAL_CONTROL_ACTION;
  const flightActions = profile.kind === 'drone' ? profile.serviceActions.filter((action) => action.id !== 'stop' && action.id !== 'disarm') : [];
  const safetyActions = profile.kind === 'drone' ? profile.serviceActions.filter((action) => action.id === 'stop' || action.id === 'disarm') : [];
  const inputStatusLabel = !enabled
    ? 'Unavailable'
    : manualControlEnabled
      ? 'Live'
      : 'Locked';

  return (
    <ConnectionSettingsProvider
      onSettingsChange={(settings) => {
        console.log('Connection settings changed:', settings);
      }}
    >
      <div className="teleop-panel">
        <div className="teleop-toolbar">
          <div className="toolbar-identity">
            <span className={`profile-icon profile-${profile.kind}`} />
            <h2 className="toolbar-title">{profile.title}</h2>
          </div>
          <div className="toolbar-controls">
            <div className="toolbar-field">
              <label className="toolbar-label">Topic</label>
              {topicOptions.length > 0 ? (
                <select
                  className="toolbar-select"
                  value={config.topic ?? ''}
                  onChange={(event) => setConfig({ ...config, topic: event.target.value })}
                >
                  {topicOptions.map((topic) => (
                    <option key={topic} value={topic}>{topic}</option>
                  ))}
                </select>
              ) : (
                <input
                  className="toolbar-input"
                  type="text"
                  value={config.topic ?? ''}
                  onChange={(event) => setConfig({ ...config, topic: event.target.value })}
                  placeholder={profile.defaultConfig.topic}
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
          <div className="toolbar-status">
            <div className={`conn-badge ${isConnected ? 'conn-on' : 'conn-off'}`}>
              <span className="conn-dot" />
              {isConnected ? 'Connected' : 'Disconnected'}
            </div>
            <ConnectionSettingsTrigger />
          </div>
        </div>

        <div className={`teleop-body ${profile.kind === 'drone' ? 'teleop-body-drone' : ''}`}>
          {!canPublish && (
            <div className="empty-state">
              <div className="empty-icon">
                <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
                  <circle cx="12" cy="12" r="10" /><line x1="4.93" y1="4.93" x2="19.07" y2="19.07" />
                </svg>
              </div>
              {!isConnected && <p>Connect to a ROS bridge to enable teleop control.</p>}
              {isConnected && config.publishRate <= 0 && <p>Set a valid publish rate to continue.</p>}
            </div>
          )}

          {canPublish && !hasTopic && (
            <div className="empty-state">
              <div className="empty-icon">
                <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
                  <path d="M12 2L2 7l10 5 10-5-10-5z" /><path d="M2 17l10 5 10-5" /><path d="M2 12l10 5 10-5" />
                </svg>
              </div>
              <p>Select a publish topic to start.</p>
            </div>
          )}

          {enabled && profile.kind === 'ground' && (
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
                    onAction={(action) => handlePadAction(profile.pads[0]!, action)}
                    disabled={!enabled}
                    activeAction={getPadActiveAction(profile.pads[0]!, orderedActiveBindings)}
                  />
                </div>
              </div>
            </div>
          )}

          {canPublish && profile.kind === 'drone' && (
            <div className="drone-layout">
              <div className="drone-pads-row">
                <div className="teleop-ops-card">
                  <div className="ops-header">
                    <h3>Operator Flow</h3>
                    <p>Arm, take off, enable manual control, fly, then exit manual control before landing.</p>
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

                {profile.pads.map((pad) => (
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
            <div className="kbd-strip">
              {profile.keyboardHints.map((hint) => (
                <span className="kbd-group" key={hint.label}>
                  <span className="kbd-label">{hint.label}</span>
                  {hint.keys.map((k) => (
                    <kbd key={k}>{k}</kbd>
                  ))}
                </span>
              ))}
            </div>
            {lastMessage && (
              <div className="telemetry-strip">
                <span className="telem-vec">
                  <span className="telem-tag">lin</span>
                  <span className="telem-val">{lastMessage.linear.x.toFixed(1)}</span>
                  <span className="telem-val">{lastMessage.linear.y.toFixed(1)}</span>
                  <span className="telem-val">{lastMessage.linear.z.toFixed(1)}</span>
                </span>
                <span className="telem-divider" />
                <span className="telem-vec">
                  <span className="telem-tag">ang</span>
                  <span className="telem-val">{lastMessage.angular.x.toFixed(1)}</span>
                  <span className="telem-val">{lastMessage.angular.y.toFixed(1)}</span>
                  <span className="telem-val">{lastMessage.angular.z.toFixed(1)}</span>
                </span>
              </div>
            )}
          </div>
        )}

        <details className="advanced-config">
          <summary><span>Command Mapping</span></summary>
          <div className="mapping-grid">
            {profile.buttonDefinitions.map((definition) => (
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
                      <option key={option.value} value={option.value}>{option.label}</option>
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
      </div>
    </ConnectionSettingsProvider>
  );
}
