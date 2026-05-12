import React, { useState, useEffect, useCallback } from 'react';
import { ros2Bridge } from 'tensorfleet-ros';
import { fetchFeaturedEntities, FeaturedEntityData } from 'tensorfleet-util';
import './ESimViewPanel.css';
import {
  EntityClickMessage,
  CARD_MESSAGES,
} from './EntityCardData';
import {
  EntityInfoPopupMessage,
  ENTITY_INFO_POPUP_MESSAGES,
} from './EntityInfoPopup';
import { EntityInfoData } from './EntityInfoPopup';
import { EntityCard } from './EntityCard';

// Adapt FeaturedEntityData to EntityCardData
type EntityCardData = FeaturedEntityData;

// ============================================================================
// FeaturedEntitiesPanel Component
// ============================================================================

export const FeaturedEntitiesPanel: React.FC = () => {
  const [featuredEntities, setFeaturedEntities] = useState<EntityCardData[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [isConnected, setIsConnected] = useState(() => ros2Bridge.isConnected());

  // Track active card state for styling
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
            />
          ))
        )}
      </div>
    </div>
  );
};

export default FeaturedEntitiesPanel;
