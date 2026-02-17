import React, { useState, useEffect, useCallback } from 'react';
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
const DEFAULT_NUDGE_STEP = 0.1;

const parseStep = (raw: unknown): number => {
  const parsed = Number(raw);
  return Number.isFinite(parsed) && parsed > 0 ? parsed : DEFAULT_NUDGE_STEP;
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
  const [nudgeStep, setNudgeStep] = useState(DEFAULT_NUDGE_STEP);

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
    setNudgeStep(parseStep(entity.params?.nudge_step));

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

  const handleNudge = useCallback((delta: { x: number; y: number; z: number }) => {
    if (!selectedEntity) return;

    const message: EntityNudgeMessage = {
      type: ENTITY_CONTROL_MESSAGES.NUDGE,
      payload: {
        entity: selectedEntity,
        delta: {
          x: delta.x * nudgeStep,
          y: delta.y * nudgeStep,
          z: delta.z * nudgeStep,
        },
        step: nudgeStep,
        timestamp: Date.now(),
      },
    };

    window.parent.postMessage(message, '*');
    console.log(`FeaturedEntitiesPanel: Entity nudged - ${selectedEntity.name}`, message);
  }, [nudgeStep, selectedEntity]);

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

      <div className="entity-nudge-controls">
        <div className="nudge-header">
          Runtime Nudge
          <span className="nudge-target">{selectedEntity?.name ?? 'Select an entity'}</span>
        </div>

        <label className="nudge-step-label">
          Step
          <input
            type="number"
            min="0.01"
            step="0.01"
            value={nudgeStep}
            onChange={(event) => setNudgeStep(parseStep(event.target.value))}
            disabled={!selectedEntity}
          />
        </label>

        <div className="nudge-grid">
          <button type="button" onClick={() => handleNudge({ x: 0, y: 1, z: 0 })} disabled={!selectedEntity}>
            +Y
          </button>
          <button type="button" onClick={() => handleNudge({ x: -1, y: 0, z: 0 })} disabled={!selectedEntity}>
            -X
          </button>
          <button type="button" onClick={() => handleNudge({ x: 1, y: 0, z: 0 })} disabled={!selectedEntity}>
            +X
          </button>
          <button type="button" onClick={() => handleNudge({ x: 0, y: -1, z: 0 })} disabled={!selectedEntity}>
            -Y
          </button>
          <button type="button" onClick={() => handleNudge({ x: 0, y: 0, z: 1 })} disabled={!selectedEntity}>
            +Z
          </button>
          <button type="button" onClick={() => handleNudge({ x: 0, y: 0, z: -1 })} disabled={!selectedEntity}>
            -Z
          </button>
        </div>
      </div>
    </div>
  );
};

export default FeaturedEntitiesPanel;
