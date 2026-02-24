import React, { useCallback } from 'react';
import { EntityCardData, CARD_MESSAGES } from './EntityCardData';
import { ENTITY_INFO_POPUP_MESSAGES } from './EntityInfoPopup';

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

  const stopPropagation = useCallback((e: React.SyntheticEvent) => {
    e.stopPropagation();
  }, []);

  const handleInfoClick = useCallback(
    (e: React.MouseEvent) => {
      e.preventDefault();
      e.stopPropagation();
      console.log('EntityCard info button requesting popup open');

      const popupMessage = {
        type: ENTITY_INFO_POPUP_MESSAGES.OPEN,
        payload: {
          name: entity.name,
          type: entity.type,
          target: entity.target,
          params: entity.params,
          timestamp: Date.now(),
        },
      };

      window.postMessage(popupMessage, '*');
      if (window.parent && window.parent !== window) {
        window.parent.postMessage(popupMessage, '*');
      }

      onInfoClick(entity, e);
    },
    [entity, onInfoClick]
  );

  const handleCardClick = useCallback(
    (e: React.MouseEvent<HTMLDivElement>) => {
      const target = e.target as HTMLElement | null;
      if (target?.closest('.info-button')) return;
      onCardClick(entity);
    },
    [entity, onCardClick]
  );

  const handleMouseEnter = useCallback(() => {
    const message = {
      type: CARD_MESSAGES.HOVER_START,
      payload: {
        entityName: entity.name,
        modelNames: entity.getModelNames(),
        timestamp: Date.now(),
      },
    };

    window.parent.postMessage(message, '*');
    console.log(`EntityCard: Hover start - ${entity.name}`, message);
  }, [entity]);

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
  }, [entity]);

  return (
    <div
      className={`entity-card ${isActive ? 'active' : ''}`}
      onClick={handleCardClick}
      onMouseEnter={handleMouseEnter}
      onMouseLeave={handleMouseLeave}
      role="button"
      tabIndex={0}
      onKeyDown={(e) => {
        const target = e.target as HTMLElement | null;
        if (target?.closest('.info-button')) return;
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
          type="button"
          className="info-button"
          onPointerDownCapture={stopPropagation}
          onPointerUpCapture={stopPropagation}
          onMouseDownCapture={stopPropagation}
          onMouseUpCapture={stopPropagation}
          onTouchStartCapture={stopPropagation}
          onTouchEndCapture={stopPropagation}
          onClick={handleInfoClick}
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