import React, { useState, useEffect, useCallback, useMemo } from 'react';
import { ros2Bridge } from '../../ros2-bridge';
import { fetchFeaturedEntities, FeaturedEntityData } from 'tensorfleet-util/ros/fetchFeaturedEntities';
import './ESimViewPanel.css';
import {
  EntityClickMessage,
  EntityNudgeMessage,
  EntitySelectMessage,
  CARD_MESSAGES,
  ENTITY_CONTROL_MESSAGES,
} from './EntityCardData';
import {
  EntityInfoPopupMessage,
  ENTITY_INFO_POPUP_MESSAGES,
} from './EntityInfoPopup';
import { EntityInfoData } from './EntityInfoPopup';
import { EntityCard } from './EntityCard';

// Adapt FeaturedEntityData to EntityCardData
type EntityCardData = FeaturedEntityData;
const POC_MOVE_DISTANCE_XY = 4.0;
const POC_MOVE_DISTANCE_Z = 2.0;

const getGazeboEntityName = (entity: EntityCardData | null): string => {
  if (!entity) return '';
  const mapped = entity.params?.gazebo_entity;
  if (typeof mapped === 'string' && mapped.trim().length > 0) {
    return mapped.trim();
  }
  return entity.target;
};

type PoseEditAccess = {
  enabled: boolean;
  reason?: string;
};

const getPoseEditAccess = (entity: EntityCardData | null): PoseEditAccess => {
  if (!entity) return { enabled: false };

  const editable = entity.params?.runtime_pose_editable;
  const policy = entity.params?.pose_edit_policy;
  const note = entity.params?.pose_edit_note;
  const gazeboEntity = getGazeboEntityName(entity);

  if (typeof editable === 'boolean') {
    return {
      enabled: editable,
      reason: !editable && typeof note === 'string' ? note : undefined,
    };
  }

  if (typeof policy === 'string' && policy.toLowerCase() === 'locked') {
    return {
      enabled: false,
      reason: typeof note === 'string' ? note : 'Pose edits are disabled for this entity.',
    };
  }

  // Fallback until every featured entity explicitly declares its pose policy.
  if (entity.type.toLowerCase() === 'arm' || gazeboEntity.startsWith('so101')) {
    return {
      enabled: false,
      reason: 'Arm base is fixed in this simulation, so runtime nudging is disabled.',
    };
  }

  return { enabled: true };
};

// ============================================================================
// FeaturedEntitiesPanel Component
// ============================================================================

export const FeaturedEntitiesPanel: React.FC = () => {
  const [featuredEntities, setFeaturedEntities] = useState<EntityCardData[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [isConnected, setIsConnected] = useState(() => ros2Bridge.isConnected());

  // Track active card state for styling + nudge controls
  const [activeCard, setActiveCard] = useState<string | null>(null);
  const [selectedEntity, setSelectedEntity] = useState<EntityCardData | null>(null);
  const poseEditAccess = useMemo(() => getPoseEditAccess(selectedEntity), [selectedEntity]);
  const nudgeControlsDisabled = !selectedEntity || !poseEditAccess.enabled;

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
        console.log('Starting to fetch featured entities...');
        const featured = await fetchFeaturedEntities(ros2Bridge);
        console.log(`Fetched ${featured.length} featured entities`);
        
        console.log('Setting featured entities state...');
        setFeaturedEntities(featured);
        setError(null);
        console.log('Featured entities state set successfully');
      } catch (err) {
        console.error('Failed to fetch featured entities:', err);
        console.error('Error details:', {
          message: err instanceof Error ? err.message : String(err),
          name: err instanceof Error ? err.name : 'Unknown',
          stack: err instanceof Error ? err.stack : undefined
        });
        setError('Failed to load featured entities');
      } finally {
        console.log('Setting loading to false...');
        setLoading(false);
        console.log('Loading complete');
      }
    };

    loadFeaturedEntities();
  }, [isConnected]);

// Handle window messages for popup close from parent
  useEffect(() => {
    const handleMessage = (event: MessageEvent<EntityInfoPopupMessage>) => {
      if (event.data.type === ENTITY_INFO_POPUP_MESSAGES.CLOSE) {
        // Popup was closed externally - nothing to do here
      }
    };

    window.addEventListener('message', handleMessage);
    return () => window.removeEventListener('message', handleMessage);
  }, []);

  // Handle main card click - sends window message for modularity
  const handleCardClick = useCallback((entity: EntityCardData) => {
    setActiveCard(entity.name);
    setSelectedEntity(entity);

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
  const handleInfoClick = useCallback((entity: EntityCardData, e: React.MouseEvent) => {
    e.stopPropagation();
    
    const popupData: EntityInfoData = {
      ...entity,
      timestamp: Date.now(),
    };
    
    // Send message to parent window to open the popup
    const message: EntityInfoPopupMessage = {
      type: ENTITY_INFO_POPUP_MESSAGES.OPEN,
      payload: popupData,
    };
    
    window.parent.postMessage(message, '*');
    
    // Also send card info click message
    const clickMessage: EntityClickMessage = {
      type: CARD_MESSAGES.INFO_CLICK,
      payload: {
        entity,
        timestamp: Date.now(),
      },
    };
    
    window.parent.postMessage(clickMessage, '*');
    console.log(`FeaturedEntitiesPanel: Info button clicked - ${entity.name}`, message);
  }, []);

  const handlePocMove = useCallback((delta: { x: number; y: number; z: number }) => {
    if (!selectedEntity || !poseEditAccess.enabled) return;

    const appliedDelta = {
      x: delta.x * POC_MOVE_DISTANCE_XY,
      y: delta.y * POC_MOVE_DISTANCE_XY,
      z: delta.z * POC_MOVE_DISTANCE_Z,
    };

    const message: EntityNudgeMessage = {
      type: ENTITY_CONTROL_MESSAGES.NUDGE,
      payload: {
        entity: selectedEntity,
        delta: appliedDelta,
        step: POC_MOVE_DISTANCE_XY,
        timestamp: Date.now(),
      },
    };

    window.parent.postMessage(message, '*');
    console.log(`FeaturedEntitiesPanel: POC move requested - ${selectedEntity.name}`, message);
  }, [poseEditAccess.enabled, selectedEntity]);

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
        Select an entity, then send a large POC move in Gazebo.
      </p>
      <div className="featured-entities-list">
        {featuredEntities.length === 0 ? (
          <div className="no-entities">No featured entities found</div>
        ) : (
          featuredEntities.map((entity) => (
            <EntityCard
              key={entity.name}
              entity={entity}
              isActive={activeCard === entity.name}
              onCardClick={handleCardClick}
              onInfoClick={handleInfoClick}
            />
          ))
        )}
      </div>

      <div className={`entity-nudge-controls ${nudgeControlsDisabled ? 'locked' : ''}`}>
        <div className="nudge-header">
          POC Move
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

        <div className="nudge-grid">
          <button type="button" onClick={() => handlePocMove({ x: 1, y: 0, z: 0 })} disabled={nudgeControlsDisabled}>
            Move +X (4m)
          </button>
          <button type="button" onClick={() => handlePocMove({ x: -1, y: 0, z: 0 })} disabled={nudgeControlsDisabled}>
            Move -X (4m)
          </button>
          <button type="button" onClick={() => handlePocMove({ x: 0, y: 1, z: 0 })} disabled={nudgeControlsDisabled}>
            Move +Y (4m)
          </button>
          <button type="button" onClick={() => handlePocMove({ x: 0, y: -1, z: 0 })} disabled={nudgeControlsDisabled}>
            Move -Y (4m)
          </button>
          <button type="button" onClick={() => handlePocMove({ x: 0, y: 0, z: 1 })} disabled={nudgeControlsDisabled}>
            Lift +Z (2m)
          </button>
          <button type="button" onClick={() => handlePocMove({ x: 0, y: 0, z: -1 })} disabled={nudgeControlsDisabled}>
            Drop -Z (2m)
          </button>
        </div>
        <div className={`nudge-hint ${nudgeControlsDisabled ? 'locked' : ''}`}>
          {nudgeControlsDisabled ? 'Move disabled for current selection.' : 'Each click sends a large delta for clear visual proof of movement.'}
        </div>
      </div>
    </div>
  );
};

export default FeaturedEntitiesPanel;
