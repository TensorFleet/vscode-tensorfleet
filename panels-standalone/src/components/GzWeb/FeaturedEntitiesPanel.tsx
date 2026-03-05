import React, { useState, useEffect, useCallback, useMemo, useRef } from 'react';
import { ros2Bridge } from '../../ros2-bridge';
import { fetchFeaturedEntities, FeaturedEntityData } from 'tensorfleet-util/ros/fetchFeaturedEntities';
import './ESimViewPanel.css';
import {
  EntityClickMessage,
  EntityNudgeMessage,
  EntityNudgeStatusMessage,
  EntityResetAllPosesMessage,
  EntityResetPoseMessage,
  EntitySelectMessage,
  EntityUndoMessage,
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
import { getGazeboEntityName, getPoseEditAccess } from './posePolicy';

// Adapt FeaturedEntityData to EntityCardData
type EntityCardData = FeaturedEntityData;

type MoveStatusTone = 'pending' | 'success' | 'error';

type MoveStatusUi = {
  tone: MoveStatusTone;
  message: string;
};

const DEFAULT_XY_STEP_METERS = 0.25;
const DEFAULT_Z_STEP_METERS = 0.1;
const STEP_PRESETS_METERS = [0.05, 0.1, 0.25, 1, 4] as const;

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
    (payload.state !== 'pending' && payload.state !== 'success' && payload.state !== 'error')
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

const clampStep = (value: number, fallback: number): number => {
  if (!Number.isFinite(value)) return fallback;
  return Math.min(25, Math.max(0.01, value));
};

const isEditableElement = (target: EventTarget | null): boolean => {
  const el = target as HTMLElement | null;
  if (!el) return false;
  const tag = el.tagName;
  return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || el.isContentEditable;
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

// ============================================================================
// FeaturedEntitiesPanel Component
// ============================================================================

export const FeaturedEntitiesPanel: React.FC = () => {
  const [featuredEntities, setFeaturedEntities] = useState<EntityCardData[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [isConnected, setIsConnected] = useState(() => ros2Bridge.isConnected());

  // Track active card state for styling + setup controls
  const [activeCard, setActiveCard] = useState<string | null>(null);
  const [selectedEntity, setSelectedEntity] = useState<EntityCardData | null>(null);
  const [moveStatus, setMoveStatus] = useState<MoveStatusUi | null>(null);
  const [xyStepMeters, setXyStepMeters] = useState(DEFAULT_XY_STEP_METERS);
  const [zStepMeters, setZStepMeters] = useState(DEFAULT_Z_STEP_METERS);
  const [scenePresetName, setScenePresetName] = useState('');
  const [scenePresetNames, setScenePresetNames] = useState<string[]>([]);
  const [selectedScenePreset, setSelectedScenePreset] = useState('');
  const pendingScenePresetSaveNameRef = useRef<string | null>(null);
  const activeMoveRequestIdRef = useRef<string | null>(null);
  const poseEditAccess = useMemo(() => getPoseEditAccess(selectedEntity), [selectedEntity]);
  const nudgeControlsDisabled = !selectedEntity || !poseEditAccess.enabled;

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
      EntityInfoPopupMessage | EntityNudgeStatusMessage | EntitySelectMessage | ScenePresetListMessage
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
            const fallbackEntity = ({
              name: incomingName || incomingTarget || 'selected_entity',
              type: typeof candidate.type === 'string' ? candidate.type : 'object',
              target: incomingTarget || incomingName,
              params: (candidate.params && typeof candidate.params === 'object')
                ? candidate.params as Record<string, unknown>
                : {},
            } as unknown as EntityCardData);
            setSelectedEntity(fallbackEntity);
            setActiveCard(null);
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

  const handleNudge = useCallback((delta: { x: number; y: number; z: number }) => {
    if (!selectedEntity || !poseEditAccess.enabled) return;
    const requestId = `nudge-${Date.now().toString(36)}-${Math.random().toString(16).slice(2, 8)}`;
    activeMoveRequestIdRef.current = requestId;
    setMoveStatus({ tone: 'pending', message: 'Sending move request...' });

    const appliedDelta = {
      x: delta.x * xyStepMeters,
      y: delta.y * xyStepMeters,
      z: delta.z * zStepMeters,
    };

    const message: EntityNudgeMessage = {
      type: ENTITY_CONTROL_MESSAGES.NUDGE,
      payload: {
        entity: selectedEntity,
        requestId,
        delta: appliedDelta,
        step: Math.max(xyStepMeters, zStepMeters),
        timestamp: Date.now(),
      },
    };

    window.parent.postMessage(message, '*');
  }, [poseEditAccess.enabled, selectedEntity, xyStepMeters, zStepMeters]);

  const handleUndo = useCallback(() => {
    if (!selectedEntity) return;
    setMoveStatus({ tone: 'pending', message: 'Undoing last move...' });
    const message: EntityUndoMessage = {
      type: ENTITY_CONTROL_MESSAGES.UNDO_LAST_MOVE,
      payload: {
        entity: selectedEntity,
        timestamp: Date.now(),
      },
    };
    window.parent.postMessage(message, '*');
  }, [selectedEntity]);

  const handleResetEntityPose = useCallback(() => {
    if (!selectedEntity) return;
    setMoveStatus({ tone: 'pending', message: 'Resetting entity pose...' });
    const message: EntityResetPoseMessage = {
      type: ENTITY_CONTROL_MESSAGES.RESET_ENTITY_POSE,
      payload: {
        entity: selectedEntity,
        timestamp: Date.now(),
      },
    };
    window.parent.postMessage(message, '*');
  }, [selectedEntity]);

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

  useEffect(() => {
    const handleKeyboardMove = (event: KeyboardEvent) => {
      if (nudgeControlsDisabled) return;
      if (isEditableElement(event.target)) return;

      const key = event.key.toLowerCase();
      if ((event.ctrlKey || event.metaKey) && key === 'z') {
        event.preventDefault();
        handleUndo();
        return;
      }

      switch (event.key) {
        case 'ArrowRight':
          event.preventDefault();
          handleNudge({ x: 1, y: 0, z: 0 });
          break;
        case 'ArrowLeft':
          event.preventDefault();
          handleNudge({ x: -1, y: 0, z: 0 });
          break;
        case 'ArrowUp':
          event.preventDefault();
          handleNudge({ x: 0, y: 1, z: 0 });
          break;
        case 'ArrowDown':
          event.preventDefault();
          handleNudge({ x: 0, y: -1, z: 0 });
          break;
        case 'PageUp':
          event.preventDefault();
          handleNudge({ x: 0, y: 0, z: 1 });
          break;
        case 'PageDown':
          event.preventDefault();
          handleNudge({ x: 0, y: 0, z: -1 });
          break;
      }
    };

    window.addEventListener('keydown', handleKeyboardMove);
    return () => window.removeEventListener('keydown', handleKeyboardMove);
  }, [handleNudge, handleUndo, nudgeControlsDisabled]);

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
        Select an entity and use Scene Setup to position it for training data.
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

      <div className={`entity-nudge-controls ${nudgeControlsDisabled ? 'locked' : ''}`}>
        <div className="nudge-header">
          Scene Setup
          <div className="nudge-header-right">
            {!nudgeControlsDisabled && <span className="nudge-control-pill enabled">enabled</span>}
            {nudgeControlsDisabled && selectedEntity && <span className="nudge-control-pill locked">locked</span>}
            <span className="nudge-target">
              {selectedEntity?.name ?? 'Select an entity'}
            </span>
          </div>
        </div>

        <div className="nudge-target-meta">
          <span>Entity</span>
          <code>{getGazeboEntityName(selectedEntity) || 'unset'}</code>
        </div>

        {selectedEntity && !poseEditAccess.enabled && (
          <div className="nudge-warning">
            {poseEditAccess.reason ?? 'Pose edits are disabled for this entity.'}
          </div>
        )}

        <label className="nudge-step-label">
          <span>XY step (m)</span>
          <input
            type="number"
            min={0.01}
            max={25}
            step={0.01}
            value={xyStepMeters}
            onChange={(event) => setXyStepMeters(clampStep(Number(event.target.value), DEFAULT_XY_STEP_METERS))}
          />
        </label>

        <label className="nudge-step-label">
          <span>Z step (m)</span>
          <input
            type="number"
            min={0.01}
            max={25}
            step={0.01}
            value={zStepMeters}
            onChange={(event) => setZStepMeters(clampStep(Number(event.target.value), DEFAULT_Z_STEP_METERS))}
          />
        </label>

        <div className="nudge-step-presets">
          {STEP_PRESETS_METERS.map((preset) => {
            const isActive = Math.abs(xyStepMeters - preset) < 0.0001 && Math.abs(zStepMeters - preset) < 0.0001;
            return (
              <button
                key={preset}
                type="button"
                className={isActive ? 'active' : ''}
                onClick={() => {
                  setXyStepMeters(preset);
                  setZStepMeters(preset);
                }}
              >
                {preset}m
              </button>
            );
          })}
        </div>

        <div className="nudge-grid">
          <button type="button" onClick={() => handleNudge({ x: 1, y: 0, z: 0 })} disabled={nudgeControlsDisabled}>
            Move +X ({xyStepMeters}m)
          </button>
          <button type="button" onClick={() => handleNudge({ x: -1, y: 0, z: 0 })} disabled={nudgeControlsDisabled}>
            Move -X ({xyStepMeters}m)
          </button>
          <button type="button" onClick={() => handleNudge({ x: 0, y: 1, z: 0 })} disabled={nudgeControlsDisabled}>
            Move +Y ({xyStepMeters}m)
          </button>
          <button type="button" onClick={() => handleNudge({ x: 0, y: -1, z: 0 })} disabled={nudgeControlsDisabled}>
            Move -Y ({xyStepMeters}m)
          </button>
          <button type="button" onClick={() => handleNudge({ x: 0, y: 0, z: 1 })} disabled={nudgeControlsDisabled}>
            Lift +Z ({zStepMeters}m)
          </button>
          <button type="button" onClick={() => handleNudge({ x: 0, y: 0, z: -1 })} disabled={nudgeControlsDisabled}>
            Drop -Z ({zStepMeters}m)
          </button>
        </div>

        <div className="nudge-grid nudge-grid-secondary">
          <button type="button" onClick={handleUndo} disabled={!selectedEntity}>
            Undo move
          </button>
          <button type="button" onClick={handleResetEntityPose} disabled={!selectedEntity}>
            Reset entity
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

        <div className={`nudge-hint ${nudgeControlsDisabled ? 'locked' : ''}`}>
          {nudgeControlsDisabled
            ? 'Move disabled for current selection.'
            : 'Axis map: +X/-X, +Y/-Y, +Z/-Z. Shortcuts: arrows = XY, PgUp/PgDn = Z, Ctrl/Cmd+Z = undo.'}
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
