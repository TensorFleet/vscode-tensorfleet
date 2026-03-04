import React from 'react';
import { EntityInfoData } from './EntityInfoPopup';

// ============================================================================
// EntityInfoContent Component - Reusable entity information display
// Can be used in popups, side panels, or full-page views
// ============================================================================

interface EntityInfoContentProps {
  entity: EntityInfoData;
  onAction?: (actionType: string, target: string) => void;
}

export const EntityInfoContent: React.FC<EntityInfoContentProps> = ({ 
  entity, 
  onAction 
}) => {
  // Format params for display
  const formatParams = (params: Record<string, any>) => {
    return Object.entries(params).map(([key, value]) => ({
      key,
      value: typeof value === 'object' ? JSON.stringify(value, null, 2) : String(value)
    }));
  };

  const formattedParams = formatParams(entity.params);

  // Get entity icon based on type
  const getEntityIcon = (type: string) => {
    switch (type.toLowerCase()) {
      case 'drone': return '🚁';
      case 'robot': return '🤖';
      default: return '📦';
    }
  };

  // Handle action with optional callback
  const handleAction = (actionType: string) => {
    if (onAction) {
      onAction(actionType, entity.target);
    } else {
      window.parent.postMessage({ type: actionType, payload: { target: entity.target } }, '*');
    }
  };

  return (
    <div className="info-popup-content">
      {/* Header Section */}
      <div className="info-popup-header">
        <div className="info-popup-title-section">
          <span className="info-popup-icon">{getEntityIcon(entity.type)}</span>
          <div className="info-popup-title-wrapper">
            <h3 className="info-popup-title">{entity.name}</h3>
            <span className="info-popup-subtitle">{entity.type}</span>
          </div>
        </div>
      </div>

      {/* Info Section - Key entity information */}
      <div className="info-popup-section info-section">
        <h4 className="section-title">Entity Information</h4>
        <div className="info-grid">
          <div className="info-item">
            <span className="info-label">Entity Name</span>
            <span className="info-value">{entity.name}</span>
          </div>
          <div className="info-item">
            <span className="info-label">Entity Type</span>
            <span className="info-value">{entity.type}</span>
          </div>
          <div className="info-item">
            <span className="info-label">Target ID</span>
            <span className="info-value">{entity.target}</span>
          </div>
          <div className="info-item">
            <span className="info-label">Status</span>
            <span className="info-value status-active-badge">Active</span>
          </div>
          <div className="info-item">
            <span className="info-label">Last Updated</span>
            <span className="info-value">{new Date(entity.timestamp).toLocaleString()}</span>
          </div>
          <div className="info-item">
            <span className="info-label">Node ID</span>
            <span className="info-value mono">{entity.target}</span>
          </div>
        </div>
      </div>

      {/* Params Section - Large scrollable area for extensive info */}
      <div className="info-popup-section params-section">
        <h4 className="section-title">Parameters & Configuration</h4>
        <div className="params-list">
          {formattedParams.length > 0 ? (
            formattedParams.map((param, index) => (
              <div key={index} className="param-item">
                <span className="param-key">{param.key}</span>
                <span className="param-value">{param.value}</span>
              </div>
            ))
          ) : (
            <p className="no-params">No parameters available for this entity</p>
          )}
        </div>
      </div>

      {/* Actions Section */}
      <div className="info-popup-section actions-section">
        <button 
          className="info-action-btn primary"
          onClick={() => handleAction('ENTITY_SELECT')}
        >
          <span className="codicon codicon-target"></span>
          Select Entity
        </button>
        <button 
          className="info-action-btn secondary"
          onClick={() => handleAction('ENTITY_TELEMETRY')}
        >
          <span className="codicon codicon-graph"></span>
          View Telemetry
        </button>
        <button 
          className="info-action-btn secondary"
          onClick={() => handleAction('ENTITY_DIAGNOSTICS')}
        >
          <span className="codicon codicon-checklist"></span>
          Diagnostics
        </button>
        <button 
          className="info-action-btn secondary"
          onClick={() => handleAction('ENTITY_CONTROLS')}
        >
          <span className="codicon codicon-gear"></span>
          Controls
        </button>
      </div>
    </div>
  );
};

export default EntityInfoContent;
