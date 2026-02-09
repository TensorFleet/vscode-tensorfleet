import React, { useState, useEffect, useCallback } from 'react';
import { ros2Bridge } from '../../ros2-bridge';
import './ESimViewPanel.css';
import { EntityInfoData, ENTITY_INFO_POPUP_MESSAGES } from './EntityInfoPopup';

// ============================================================================
// Window Message Types for External Module Communication
// ============================================================================

export interface EntityCardData {
  name: string;
  type: string;
  target: string;
  params: Record<string, any>;
}

export interface EntityClickMessage {
  type: 'ENTITY_CLICK' | 'ENTITY_INFO_CLICK';
  payload: {
    entity: EntityCardData;
    timestamp: number;
  };
}

export interface EntityInfoPopupMessage {
  type: 'ENTITY_INFO_POPUP_OPEN' | 'ENTITY_INFO_POPUP_CLOSE';
  payload?: EntityInfoData;
}

// Message type constants
export const CARD_MESSAGES = {
  CLICK: 'ENTITY_CLICK' as const,
  INFO_CLICK: 'ENTITY_INFO_CLICK' as const,
};

// ============================================================================
// FeaturedEntitiesPanel Component
// ============================================================================

export const FeaturedEntitiesPanel: React.FC = () => {
  const [featuredEntities, setFeaturedEntities] = useState<EntityCardData[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [isConnected, setIsConnected] = useState(() => ros2Bridge.isConnected());

  // Track active card state for styling
  const [hoveredCard, setHoveredCard] = useState<string | null>(null);
  const [activeCard, setActiveCard] = useState<string | null>(null);

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

    const fetchFeaturedEntities = async () => {
      console.log(`FeaturedEntitiesPanel: Starting fetch, connection status: ${isConnected}`);
      try {
        console.log('Starting to fetch featured entities...');
        console.log('Calling ros2Bridge.getAllROSParameters()...');
        const allParams = await ros2Bridge.getAllROSParameters();
        console.log('Received allParams:', Object.keys(allParams).length, 'parameters');

        const featured: EntityCardData[] = [];
        console.log('Starting to process parameters for featured entities...');

        // Find all nodes with proxy_featured=true
        let processedCount = 0;
        let featuredCount = 0;
        for (const [key, value] of Object.entries(allParams)) {
          processedCount++;
          if (key.endsWith('.proxy_featured') && value === true) {
            console.log(`Found proxy_featured=true for key: ${key}`);
            featuredCount++;
            const nodeName = key.replace('.proxy_featured', '');
            const target = allParams[`${nodeName}.proxy_target`] as string;
            const paramsStr = allParams[`${nodeName}.params`] as string;
            console.log(`Processing node ${nodeName}: target=${target}, paramsStr length=${paramsStr?.length || 0}`);

            if (target && paramsStr) {
              try {
                const params = JSON.parse(paramsStr);
                console.log(`Successfully parsed params for ${nodeName}:`, params);
                featured.push({
                  name: target,
                  type: params.type || 'unknown',
                  target: target,
                  params: params,
                });
                console.log(`Added featured entity: ${target}`);
              } catch (parseError) {
                console.warn(`Failed to parse params for ${nodeName}:`, parseError);
              }
            } else {
              console.log(`Missing target or params for ${nodeName}: target=${!!target}, paramsStr=${!!paramsStr}`);
            }
          }
        }
        console.log(`Processed ${processedCount} total parameters, found ${featuredCount} featured entities, added ${featured.length} to list`);

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

    fetchFeaturedEntities();
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

  const getStatusIcon = (entity: EntityCardData) => {
    // For now, assume all are active. You could add logic to check actual status
    return 'status-active';
  };

  // Handle main card click - sends window message for modularity
  const handleCardClick = useCallback((entity: EntityCardData) => {
    const message: EntityClickMessage = {
      type: CARD_MESSAGES.CLICK,
      payload: {
        entity,
        timestamp: Date.now(),
      },
    };
    
    // Send message to parent window for external module communication
    window.parent.postMessage(message, '*');
    
    // Log for debugging
    console.log(`FeaturedEntitiesPanel: Card clicked - ${entity.name}`, message);
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
          featuredEntities.map((entity, index) => (
            <div
              key={index}
              className={`entity-card ${activeCard === entity.name ? 'active' : ''}`}
              onClick={() => handleCardClick(entity)}
              onMouseEnter={() => setHoveredCard(entity.name)}
              onMouseLeave={() => setHoveredCard(null)}
              onMouseDown={() => setActiveCard(entity.name)}
              onMouseUp={() => setActiveCard(null)}
              onMouseOut={() => setActiveCard(null)}
              role="button"
              tabIndex={0}
              onKeyDown={(e) => {
                if (e.key === 'Enter' || e.key === ' ') {
                  e.preventDefault();
                  handleCardClick(entity);
                }
              }}
            >
              <div className="entity-header">
                <span className="entity-icon">{getEntityIcon(entity.type)}</span>
                <span className="entity-name">{entity.name}</span>
                <span className={`entity-status ${getStatusIcon(entity)}`}>Active</span>
                {/* Info Button with $(info) icon - blocks main card click */}
                <button
                  className="info-button"
                  onClick={(e) => handleInfoClick(entity, e)}
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
          ))
        )}
      </div>
    </div>
  );
};

export default FeaturedEntitiesPanel;
