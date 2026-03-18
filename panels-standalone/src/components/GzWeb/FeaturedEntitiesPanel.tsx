import React, { useState, useEffect, useCallback, useRef } from 'react';
import { ros2Bridge } from '../../ros2-bridge';
import { fetchFeaturedEntities, FeaturedEntityData } from 'tensorfleet-util/ros/fetchFeaturedEntities';
import './ESimViewPanel.css';
import {
  EntityClickMessage,
  EntityManagedEntitiesMessage,
  EntityNudgeStatusMessage,
  EntityResetAllPosesMessage,
  EntitySelectMessage,
  SceneSetupTraceConfigMessage,
  ScenePresetListMessage,
  ScenePresetListRequestMessage,
  ScenePresetLoadMessage,
  ScenePresetSaveMessage,
  CARD_MESSAGES,
  ENTITY_CONTROL_MESSAGES,
} from './EntityCardData';
import {
  EntityInfoPopupMessage,
  ENTITY_INFO_POPUP_MESSAGES,
} from './EntityInfoPopup';
import { EntityInfoData } from './EntityInfoPopup';
import { EntityCard } from './EntityCard';
import { getGazeboEntityName } from './posePolicy';

// Adapt FeaturedEntityData to EntityCardData
type EntityCardData = FeaturedEntityData;

type MoveStatusTone = 'pending' | 'success' | 'error' | 'warning';

type MoveStatusUi = {
  tone: MoveStatusTone;
  message: string;
};

const toNudgeStatusMessage = (data: unknown): EntityNudgeStatusMessage | null => {
  if (!data || typeof data !== 'object') return null;
  const candidate = data as Partial<EntityNudgeStatusMessage>;
  if (candidate.type !== ENTITY_CONTROL_MESSAGES.NUDGE_STATUS || !candidate.payload) {
    return null;
  }
  const payload = candidate.payload as Partial<EntityNudgeStatusMessage['payload']>;
  if (
    typeof payload.entity !== 'string' ||
    typeof payload.message !== 'string' ||
    (payload.state !== 'pending' &&
      payload.state !== 'success' &&
      payload.state !== 'error' &&
      payload.state !== 'warning')
  ) {
    return null;
  }
  return {
    type: ENTITY_CONTROL_MESSAGES.NUDGE_STATUS,
    payload: {
      entity: payload.entity,
      message: payload.message,
      state: payload.state,
      timestamp: typeof payload.timestamp === 'number' ? payload.timestamp : Date.now(),
      requestId: typeof payload.requestId === 'string' ? payload.requestId : undefined,
      attempt: typeof payload.attempt === 'number' ? payload.attempt : undefined,
      maxAttempts: typeof payload.maxAttempts === 'number' ? payload.maxAttempts : undefined,
    },
  };
};

const toScenePresetListMessage = (data: unknown): ScenePresetListMessage | null => {
  if (!data || typeof data !== 'object') return null;
  const candidate = data as Partial<ScenePresetListMessage>;
  if (candidate.type !== ENTITY_CONTROL_MESSAGES.SCENE_PRESET_LIST || !candidate.payload) {
    return null;
  }
  const payload = candidate.payload as Partial<ScenePresetListMessage['payload']>;
  if (!Array.isArray(payload.names)) return null;
  const names = payload.names.filter((name): name is string => typeof name === 'string');
  return {
    type: ENTITY_CONTROL_MESSAGES.SCENE_PRESET_LIST,
    payload: {
      names,
      timestamp: typeof payload.timestamp === 'number' ? payload.timestamp : Date.now(),
    },
  };
};

const getStoredTraceEnabled = (): boolean => {
  if (typeof window === 'undefined') return false;
  try {
    const raw = window.localStorage.getItem('tf.move.trace');
    if (!raw) return false;
    const normalized = raw.trim().toLowerCase();
    return normalized === '1' || normalized === 'true' || normalized === 'yes' || normalized === 'on';
  } catch {
    return false;
  }
};

const getStoredTraceFilter = (): string => {
  if (typeof window === 'undefined') return '';
  try {
    return window.localStorage.getItem('tf.move.trace.filter')?.trim() ?? '';
  } catch {
    return '';
  }
};

// ============================================================================
// FeaturedEntitiesPanel Component
// ============================================================================

export const FeaturedEntitiesPanel: React.FC = () => {
  const [featuredEntities, setFeaturedEntities] = useState<EntityCardData[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [isConnected, setIsConnected] = useState(() => ros2Bridge.isConnected());

  // Track active card state for styling and selection sync
  const [activeCard, setActiveCard] = useState<string | null>(null);
  const [selectedEntity, setSelectedEntity] = useState<EntityCardData | null>(null);
  const [moveStatus, setMoveStatus] = useState<MoveStatusUi | null>(null);
  const [scenePresetName, setScenePresetName] = useState('');
  const [scenePresetNames, setScenePresetNames] = useState<string[]>([]);
  const [selectedScenePreset, setSelectedScenePreset] = useState('');
  const [traceEnabled, setTraceEnabled] = useState(() => getStoredTraceEnabled());
  const [traceFilter, setTraceFilter] = useState(() => getStoredTraceFilter());
  const pendingScenePresetSaveNameRef = useRef<string | null>(null);
  const activeMoveRequestIdRef = useRef<string | null>(null);

  const publishTraceConfig = useCallback((enabled: boolean, filter: string) => {
    try {
      window.localStorage.setItem('tf.move.trace', enabled ? '1' : '0');
      const trimmedFilter = filter.trim();
      if (trimmedFilter.length > 0) {
        window.localStorage.setItem('tf.move.trace.filter', trimmedFilter);
      } else {
        window.localStorage.removeItem('tf.move.trace.filter');
      }
    } catch {
      // Ignore localStorage access failures.
    }
    const payload: SceneSetupTraceConfigMessage['payload'] = {
      enabled,
      filter: filter.trim() || undefined,
      timestamp: Date.now(),
    };
    window.parent.postMessage(
      {
        type: ENTITY_CONTROL_MESSAGES.SCENE_SETUP_TRACE_CONFIG,
        payload,
      } satisfies SceneSetupTraceConfigMessage,
      '*',
    );
  }, []);

  const refreshScenePresetNames = useCallback((preferredName?: string) => {
    if (preferredName) {
      pendingScenePresetSaveNameRef.current = preferredName;
    }
    const message: ScenePresetListRequestMessage = {
      type: ENTITY_CONTROL_MESSAGES.SCENE_PRESET_LIST_REQUEST,
      payload: {
        timestamp: Date.now(),
      },
    };
    window.parent.postMessage(message, '*');
  }, []);

  // Monitor connection status like other components do
  useEffect(() => {
    const connectionInterval = setInterval(() => {
      const wasConnected = isConnected;
      const nowConnected = ros2Bridge.isConnected();
      if (wasConnected !== nowConnected) {
        console.log(`FeaturedEntitiesPanel: Connection status changed: ${wasConnected} -> ${nowConnected}`);
      }
      setIsConnected(nowConnected);
    }, 1000);

    return () => clearInterval(connectionInterval);
  }, [isConnected]);

  useEffect(() => {
    if (!isConnected) return;

    const loadFeaturedEntities = async () => {
      console.log(`FeaturedEntitiesPanel: Starting fetch, connection status: ${isConnected}`);
      try {
        const featured = await fetchFeaturedEntities(ros2Bridge);
        setFeaturedEntities(featured);
        setError(null);
      } catch (err) {
        console.error('Failed to fetch featured entities:', err);
        setError('Failed to load featured entities');
      } finally {
        setLoading(false);
      }
    };

    loadFeaturedEntities();
  }, [isConnected]);

  useEffect(() => {
    refreshScenePresetNames();
  }, [refreshScenePresetNames]);

  useEffect(() => {
    if (!isConnected) return;
    refreshScenePresetNames();
  }, [isConnected, refreshScenePresetNames]);

  // Handle window messages for popup close + move status updates.
  useEffect(() => {
    const handleMessage = (
      event: MessageEvent<
      EntityInfoPopupMessage |
      EntityNudgeStatusMessage |
      EntitySelectMessage |
      ScenePresetListMessage
      >,
    ) => {
      const presetListMessage = toScenePresetListMessage(event.data);
      if (presetListMessage) {
        const names = [...presetListMessage.payload.names];
        setScenePresetNames(names);
        setSelectedScenePreset((current) => {
          const preferredName = pendingScenePresetSaveNameRef.current;
          if (preferredName && names.includes(preferredName)) {
            pendingScenePresetSaveNameRef.current = null;
            return preferredName;
          }
          pendingScenePresetSaveNameRef.current = null;
          return names.includes(current) ? current : (names[0] ?? '');
        });
        return;
      }

      if (event.data.type === ENTITY_CONTROL_MESSAGES.SELECT) {
        const incoming = event.data.payload?.entity;
        if (incoming && typeof incoming === 'object') {
          const candidate = incoming as Partial<EntityCardData>;
          const incomingName = typeof candidate.name === 'string' ? candidate.name : '';
          const incomingTarget = typeof candidate.target === 'string' ? candidate.target : '';
          const match = featuredEntities.find((entity) => {
            const gazeboName = getGazeboEntityName(entity);
            if (gazeboName && (gazeboName === incomingTarget || gazeboName === incomingName)) {
              return true;
            }
            if (entity.name === incomingName || entity.target === incomingTarget) {
              return true;
            }
            const modelNames = entity.getModelNames?.() ?? [];
            return modelNames.includes(incomingTarget) || modelNames.includes(incomingName);
          });

          if (match) {
            setSelectedEntity(match);
            setActiveCard(match.name);
          } else {
            // Ignore viewport/non-featured selections so only featured cards
            // can drive movement controls.
            return;
          }
          setMoveStatus(null);
          activeMoveRequestIdRef.current = null;
        }
        return;
      }

      if (event.data.type === ENTITY_INFO_POPUP_MESSAGES.CLOSE) {
        return;
      }

      const statusMessage = toNudgeStatusMessage(event.data);
      if (!statusMessage) {
        return;
      }

      const selectedGazeboEntity = getGazeboEntityName(selectedEntity);
      const isSceneWideStatus = statusMessage.payload.entity === '__scene__';
      const isFromSelectedEntity = selectedGazeboEntity
        ? statusMessage.payload.entity === selectedGazeboEntity
        : true;
      const activeRequestId = activeMoveRequestIdRef.current;
      const isActiveRequest = activeRequestId
        ? statusMessage.payload.requestId === activeRequestId
        : false;

      if (!isSceneWideStatus && !isFromSelectedEntity && !isActiveRequest) {
        return;
      }

      setMoveStatus({
        tone: statusMessage.payload.state,
        message: statusMessage.payload.message,
      });

      if (statusMessage.payload.state !== 'pending') {
        activeMoveRequestIdRef.current = null;
      }

      if (isSceneWideStatus && statusMessage.payload.state !== 'pending') {
        const pendingSaveName = pendingScenePresetSaveNameRef.current;
        if (statusMessage.payload.state === 'success') {
          refreshScenePresetNames(pendingSaveName ?? undefined);
        } else {
          pendingScenePresetSaveNameRef.current = null;
        }
      }
    };

    window.addEventListener('message', handleMessage);
    return () => window.removeEventListener('message', handleMessage);
  }, [featuredEntities, refreshScenePresetNames, selectedEntity]);

  useEffect(() => {
    const message: EntityManagedEntitiesMessage = {
      type: ENTITY_CONTROL_MESSAGES.MANAGED_ENTITIES,
      payload: {
        entities: featuredEntities,
        timestamp: Date.now(),
      },
    };
    window.parent.postMessage(message, '*');
  }, [featuredEntities]);

  // Handle main card click - sends window message for modularity
  const handleCardClick = useCallback((entity: EntityCardData) => {
    setActiveCard(entity.name);
    setSelectedEntity(entity);
    setMoveStatus(null);
    activeMoveRequestIdRef.current = null;

    const message: EntityClickMessage = {
      type: CARD_MESSAGES.CLICK,
      payload: {
        entity,
        timestamp: Date.now(),
      },
    };
    window.parent.postMessage(message, '*');

    const selectMessage: EntitySelectMessage = {
      type: ENTITY_CONTROL_MESSAGES.SELECT,
      payload: {
        entity,
        timestamp: Date.now(),
      },
    };
    window.parent.postMessage(selectMessage, '*');
    console.log(`FeaturedEntitiesPanel: Card selected - ${entity.name}`, selectMessage);
  }, []);

  // Handle info button click - sends window message to open popup
  const handleInfoClick = useCallback((entity: EntityCardData) => {
    const popupData: EntityInfoData = {
      ...entity,
      timestamp: Date.now(),
    };

    const message: EntityInfoPopupMessage = {
      type: ENTITY_INFO_POPUP_MESSAGES.OPEN,
      payload: popupData,
    };

    window.parent.postMessage(message, '*');

    const clickMessage: EntityClickMessage = {
      type: CARD_MESSAGES.INFO_CLICK,
      payload: {
        entity,
        timestamp: Date.now(),
      },
    };

    window.parent.postMessage(clickMessage, '*');
  }, []);

  const handleResetAllPoses = useCallback(() => {
    setMoveStatus({ tone: 'pending', message: 'Resetting all scene objects to session start...' });
    const message: EntityResetAllPosesMessage = {
      type: ENTITY_CONTROL_MESSAGES.RESET_ALL_POSES,
      payload: {
        entities: featuredEntities,
        timestamp: Date.now(),
      },
    };
    window.parent.postMessage(message, '*');
  }, [featuredEntities]);

  const handleSaveScenePreset = useCallback(() => {
    const name = scenePresetName.trim();
    if (!name) {
      setMoveStatus({ tone: 'error', message: 'Preset name is required.' });
      return;
    }

    setMoveStatus({ tone: 'pending', message: `Saving preset "${name}"...` });
    const message: ScenePresetSaveMessage = {
      type: ENTITY_CONTROL_MESSAGES.SCENE_PRESET_SAVE,
      payload: {
        name,
        timestamp: Date.now(),
      },
    };
    pendingScenePresetSaveNameRef.current = name;
    window.parent.postMessage(message, '*');
  }, [scenePresetName]);

  const handleLoadScenePreset = useCallback(() => {
    const name = selectedScenePreset.trim();
    if (!name) {
      setMoveStatus({ tone: 'error', message: 'Select a preset to load.' });
      return;
    }

    setMoveStatus({ tone: 'pending', message: `Loading preset "${name}"...` });
    const message: ScenePresetLoadMessage = {
      type: ENTITY_CONTROL_MESSAGES.SCENE_PRESET_LOAD,
      payload: {
        name,
        timestamp: Date.now(),
      },
    };
    window.parent.postMessage(message, '*');
  }, [selectedScenePreset]);

  const handleTraceToggle = useCallback((enabled: boolean) => {
    setTraceEnabled(enabled);
    publishTraceConfig(enabled, traceFilter);
  }, [publishTraceConfig, traceFilter]);

  const handleApplyTraceFilter = useCallback(() => {
    publishTraceConfig(traceEnabled, traceFilter);
  }, [publishTraceConfig, traceEnabled, traceFilter]);

  const bindCardCallbacks = useCallback(
    (entity: EntityCardData): EntityCardData => {
      const cardEntity = Object.create(entity) as EntityCardData;
      cardEntity.onCardClick = () => handleCardClick(entity);
      cardEntity.onInfoClick = () => handleInfoClick(entity);
      return cardEntity;
    },
    [handleCardClick, handleInfoClick],
  );

  if (loading) {
    return (
      <div className="featured-entities-container">
        <h3>Featured entities</h3>
        <div className="loading">Loading featured entities...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="featured-entities-container">
        <h3>Featured entities</h3>
        <div className="error">{error}</div>
      </div>
    );
  }

  return (
    <div className="featured-entities-container">
      <h3>Featured entities</h3>
      <p className="featured-entities-subtitle">
        Entity cards are for selection and info. Scene controls below apply to the whole world.
      </p>
      <div className="featured-entities-list">
        {featuredEntities.length === 0 ? (
          <div className="no-entities">No featured entities found</div>
        ) : (
          featuredEntities.map((entity) => (
            <EntityCard
              key={entity.name}
              entity={bindCardCallbacks(entity)}
              isActive={activeCard === entity.name}
            />
          ))
        )}
      </div>

      <div className="entity-nudge-controls">
        <div className="nudge-header">
          Scene Controls
        </div>

        <div className="scene-setup-trace">
          <div className="scene-setup-section-title">Trace</div>
          <label className="scene-setup-trace-toggle">
            <input
              type="checkbox"
              checked={traceEnabled}
              onChange={(event) => handleTraceToggle(event.target.checked)}
            />
            <span>Enable move trace</span>
          </label>
          <label className="nudge-step-label">
            <span>Filter</span>
            <input
              type="text"
              value={traceFilter}
              onChange={(event) => setTraceFilter(event.target.value)}
              placeholder="simple_bot_include / move.confirmed / requestId"
            />
          </label>
          <button type="button" onClick={handleApplyTraceFilter}>
            Apply trace filter
          </button>
        </div>

        <div className="scene-preset-controls">
          <label className="nudge-step-label">
            <span>Preset name</span>
            <input
              type="text"
              value={scenePresetName}
              onChange={(event) => setScenePresetName(event.target.value)}
              placeholder="kitchen-clutter-a"
            />
          </label>
          <div className="nudge-grid nudge-grid-secondary">
            <button type="button" onClick={handleSaveScenePreset}>Save scene</button>
            <button type="button" onClick={() => refreshScenePresetNames()}>Refresh</button>
          </div>
          <label className="scene-preset-select">
            <span>Saved presets (session)</span>
            <select
              value={selectedScenePreset}
              onChange={(event) => setSelectedScenePreset(event.target.value)}
              disabled={scenePresetNames.length === 0}
            >
              {scenePresetNames.length === 0 && <option value="">No presets yet</option>}
              {scenePresetNames.map((name) => (
                <option key={name} value={name}>{name}</option>
              ))}
            </select>
          </label>
          <button type="button" onClick={handleLoadScenePreset} disabled={!selectedScenePreset}>
            Load scene
          </button>
          <button type="button" onClick={handleResetAllPoses} disabled={!isConnected}>
            Reset all objects
          </button>
        </div>

        <div className="nudge-hint">
          Presets and reset operate on the full scene and are not tied to the selected entity.
        </div>

        {moveStatus && (
          <div className={`nudge-status ${moveStatus.tone}`}>
            {moveStatus.message}
          </div>
        )}
      </div>
    </div>
  );
};

export default FeaturedEntitiesPanel;
