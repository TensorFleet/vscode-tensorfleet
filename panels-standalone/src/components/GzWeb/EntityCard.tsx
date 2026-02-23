import React, { useCallback } from 'react';
import { EntityCardData, CARD_MESSAGES } from './EntityCardData';

// ============================================================================
// EntityCard Component
// ============================================================================

interface EntityCardProps {
  entity: EntityCardData;
  isActive: boolean;
  onCardClick: (entity: EntityCardData) => void;
  onInfoClick: (entity: EntityCardData, e: React.MouseEvent) => void;
}

export const EntityCard: React.FC<EntityCardProps> = ({
  entity,
  isActive,
  onCardClick,
  onInfoClick,
}) => {
  const getEntityIcon = (type: string) => {
    switch (type.toLowerCase()) {
      case 'drone':
        return '🚁';
      case 'robot':
        return '🤖';
      default:
        return '📦';
    }
  };

  const getStatusIcon = (_entity: EntityCardData) => {
    return 'status-active';
  };

  const handleInfoMouseDown = useCallback((e: React.MouseEvent) => {
    e.stopPropagation();
  }, []);

  const handleInfoMouseUp = useCallback((e: React.MouseEvent) => {
    e.stopPropagation();
  }, []);

  const handleMouseEnter = useCallback(() => {
    const message = {
      type: CARD_MESSAGES.HOVER_START,
      payload: {
        entityName: entity.name,
        timestamp: Date.now(),
      },
    };
    
    window.parent.postMessage(message, '*');
    console.log(`EntityCard: Hover start - ${entity.name}`, message);
  }, [entity.name]);

  const handleMouseLeave = useCallback(() => {
    const message = {
      type: CARD_MESSAGES.HOVER_END,
      payload: {
        entityName: entity.name,
        modelNames: entity.getModelNames(),
        timestamp: Date.now(),
      },
    };
    
    window.parent.postMessage(message, '*');
    
    console.log(`EntityCard: Hover end - ${entity.name} -> ${message.payload.modelNames}`, message);
  }, [entity.name]);

  return (
    <div
      className={`entity-card ${isActive ? 'active' : ''}`}
      onClick={() => onCardClick(entity)}
      onMouseEnter={handleMouseEnter}
      onMouseLeave={handleMouseLeave}
      role="button"
      tabIndex={0}
      onKeyDown={(e) => {
        if (e.key === 'Enter' || e.key === ' ') {
          e.preventDefault();
          onCardClick(entity);
        }
      }}
    >
      <div className="entity-header">
        <span className="entity-icon">{getEntityIcon(entity.type)}</span>
        <span className="entity-name">{entity.name}</span>
        <span className={`entity-status ${getStatusIcon(entity)}`}>Active</span>
        <button
          className="info-button"
          onMouseDown={handleInfoMouseDown}
          onMouseUp={handleInfoMouseUp}
          onClick={(e) => onInfoClick(entity, e)}
          aria-label={`View info for ${entity.name}`}
          title="View detailed information"
        >
          <span className="codicon-info"></span>
        </button>
      </div>
      <div className="entity-details">
        <div className="entity-metric">
          <span className="metric-label">Type:</span>
          <span className="metric-value">{entity.type}</span>
        </div>
        <div className="entity-metric">
          <span className="metric-label">Target:</span>
          <span className="metric-value">{entity.target}</span>
        </div>
      </div>
    </div>
  );
};

export default EntityCard;
