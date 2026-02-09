import React, { useEffect, useCallback } from 'react';

// ============================================================================
// Window Message Types for Modular Communication
// ============================================================================

export interface EntityInfoData {
  name: string;
  type: string;
  target: string;
  params: Record<string, any>;
  timestamp: number;
  // Reserved for future tabs:
  // telemetry?: any;
  // diagnostics?: any;
  // controls?: any;
}

export interface InfoPopupMessage {
  type: 'POPUP_OPEN' | 'POPUP_CLOSE' | 'POPUP_READY';
  payload?: EntityInfoData;
}

// Message type constants for external modules
export const POPUP_MESSAGES = {
  OPEN: 'POPUP_OPEN' as const,
  CLOSE: 'POPUP_CLOSE' as const,
  READY: 'POPUP_READY' as const,
};

// ============================================================================
// InfoPopup Component - Large popup ready for sub-tabs
// ============================================================================

interface InfoPopupProps {
  data: EntityInfoData | null;
  onClose: () => void;
}

export const InfoPopup: React.FC<InfoPopupProps> = ({ data, onClose }) => {
  // Handle window messaging for modular communication
  const handleMessage = useCallback((event: MessageEvent<InfoPopupMessage>) => {
    if (event.data.type === POPUP_MESSAGES.CLOSE) {
      onClose();
    }
  }, [onClose]);

  useEffect(() => {
    // Subscribe to window messages for cross-module communication
    window.addEventListener('message', handleMessage);
    
    // Notify parent that popup is ready
    window.parent.postMessage({ type: POPUP_MESSAGES.READY }, '*');
    
    return () => {
      window.removeEventListener('message', handleMessage);
    };
  }, [handleMessage]);

  // Handle escape key to close
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        onClose();
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [onClose]);

  if (!data) return null;

  // Format params for display
  const formatParams = (params: Record<string, any>) => {
    return Object.entries(params).map(([key, value]) => ({
      key,
      value: typeof value === 'object' ? JSON.stringify(value, null, 2) : String(value)
    }));
  };

  const formattedParams = formatParams(data.params);

  // Get entity icon based on type
  const getEntityIcon = (type: string) => {
    switch (type.toLowerCase()) {
      case 'drone': return '🚁';
      case 'robot': return '🤖';
      default: return '📦';
    }
  };

  return (
    <div className="info-popup-overlay" onClick={onClose}>
      <div 
        className="info-popup" 
        onClick={(e) => e.stopPropagation()}
        role="dialog"
        aria-modal="true"
        aria-label={`Information about ${data.name}`}
      >
        {/* Header Section */}
        <div className="info-popup-header">
          <div className="info-popup-title-section">
            <span className="info-popup-icon">{getEntityIcon(data.type)}</span>
            <div className="info-popup-title-wrapper">
              <h3 className="info-popup-title">{data.name}</h3>
              <span className="info-popup-subtitle">{data.type}</span>
            </div>
          </div>
          <button 
            className="info-popup-close"
            onClick={onClose}
            aria-label="Close popup"
            title="Close (Esc)"
          >
            <span className="codicon codicon-close"></span>
          </button>
        </div>

        {/* Popup Content Container */}
        <div className="info-popup-content">
          {/* Info Section - Key entity information */}
          <div className="info-popup-section info-section">
            <h4 className="section-title">Entity Information</h4>
            <div className="info-grid">
              <div className="info-item">
                <span className="info-label">Entity Name</span>
                <span className="info-value">{data.name}</span>
              </div>
              <div className="info-item">
                <span className="info-label">Entity Type</span>
                <span className="info-value">{data.type}</span>
              </div>
              <div className="info-item">
                <span className="info-label">Target ID</span>
                <span className="info-value">{data.target}</span>
              </div>
              <div className="info-item">
                <span className="info-label">Status</span>
                <span className="info-value status-active-badge">Active</span>
              </div>
              <div className="info-item">
                <span className="info-label">Last Updated</span>
                <span className="info-value">{new Date(data.timestamp).toLocaleString()}</span>
              </div>
              <div className="info-item">
                <span className="info-label">Node ID</span>
                <span className="info-value mono">{data.target}</span>
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
              onClick={() => {
                window.parent.postMessage(
                  { type: 'ENTITY_SELECT', payload: { target: data.target } }, 
                  '*'
                );
                onClose();
              }}
            >
              <span className="codicon codicon-target"></span>
              Select Entity
            </button>
            <button 
              className="info-action-btn secondary"
              onClick={() => {
                window.parent.postMessage(
                  { type: 'ENTITY_TELEMETRY', payload: { target: data.target } }, 
                  '*'
                );
                onClose();
              }}
            >
              <span className="codicon codicon-graph"></span>
              View Telemetry
            </button>
            <button 
              className="info-action-btn secondary"
              onClick={() => {
                window.parent.postMessage(
                  { type: 'ENTITY_DIAGNOSTICS', payload: { target: data.target } }, 
                  '*'
                );
              }}
            >
              <span className="codicon codicon-checklist"></span>
              Diagnostics
            </button>
            <button 
              className="info-action-btn secondary"
              onClick={() => {
                window.parent.postMessage(
                  { type: 'ENTITY_CONTROLS', payload: { target: data.target } }, 
                  '*'
                );
              }}
            >
              <span className="codicon codicon-gear"></span>
              Controls
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default InfoPopup;
