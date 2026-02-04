import React, { useState, useEffect } from 'react';
import { ros2Bridge } from '../../ros2-bridge';

interface FeaturedEntity {
  name: string;
  type: string;
  target: string;
  params: any;
}

export const FeaturedEntitiesPanel: React.FC = () => {
  const [featuredEntities, setFeaturedEntities] = useState<FeaturedEntity[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [isConnected, setIsConnected] = useState(() => ros2Bridge.isConnected());

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

        const featured: FeaturedEntity[] = [];
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

  const getStatusIcon = (entity: FeaturedEntity) => {
    // For now, assume all are active. You could add logic to check actual status
    return 'status-active';
  };

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
            <div key={index} className="entity-card">
              <div className="entity-header">
                <span className="entity-icon">{getEntityIcon(entity.type)}</span>
                <span className="entity-name">{entity.name}</span>
                <span className={`entity-status ${getStatusIcon(entity)}`}>Active</span>
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