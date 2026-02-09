import React, { useState, useEffect, useCallback } from 'react';
import { EntityInfoContent } from './EntityInfoContent';

// ============================================================================
// Window Message Types for Modular Communication
// ============================================================================

export interface EntityInfoData {
  name: string;
  type: string;
  target: string;
  params: Record<string, any>;
  timestamp: number;
}

export interface EntityInfoPopupMessage {
  type: 'ENTITY_INFO_POPUP_OPEN' | 'ENTITY_INFO_POPUP_CLOSE' | 'ENTITY_INFO_POPUP_READY';
  payload?: EntityInfoData;
}

// Message type constants for external modules
export const ENTITY_INFO_POPUP_MESSAGES = {
  OPEN: 'ENTITY_INFO_POPUP_OPEN' as const,
  CLOSE: 'ENTITY_INFO_POPUP_CLOSE' as const,
  READY: 'ENTITY_INFO_POPUP_READY' as const,
};

// ============================================================================
// EntityInfoPopup Component - Popup wrapper for EntityInfoContent
// Handles window message communication and popup lifecycle
// ============================================================================

export const EntityInfoPopup: React.FC = () => {
  const [data, setData] = useState<EntityInfoData | null>(null);
  const [isVisible, setIsVisible] = useState(false);

  // Handle window messages for self-contained popup control
  const handleMessage = useCallback((event: MessageEvent<EntityInfoPopupMessage>) => {
    if (event.data.type === ENTITY_INFO_POPUP_MESSAGES.OPEN && event.data.payload) {
      setData(event.data.payload);
      setIsVisible(true);
    } else if (event.data.type === ENTITY_INFO_POPUP_MESSAGES.CLOSE) {
      setIsVisible(false);
      setData(null);
    }
  }, []);

  useEffect(() => {
    window.addEventListener('message', handleMessage);
    return () => window.removeEventListener('message', handleMessage);
  }, [handleMessage]);

  // Handle close
  const handleClose = useCallback(() => {
    setIsVisible(false);
    setData(null);
    // Notify parent that popup is closed
    window.parent.postMessage({ type: ENTITY_INFO_POPUP_MESSAGES.CLOSE }, '*');
  }, []);

  // Handle escape key to close
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape' && isVisible) {
        handleClose();
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isVisible, handleClose]);

  if (!isVisible || !data) return null;

  return (
    <div className="info-popup-overlay" onClick={handleClose}>
      <div 
        className="info-popup" 
        onClick={(e) => e.stopPropagation()}
        role="dialog"
        aria-modal="true"
        aria-label={`Information about ${data.name}`}
      >
        <button 
          className="info-popup-close"
          onClick={handleClose}
          aria-label="Close popup"
          title="Close (Esc)"
        >
          <span className="codicon codicon-close"></span>
        </button>
        <EntityInfoContent entity={data} />
      </div>
    </div>
  );
};

export default EntityInfoPopup;
