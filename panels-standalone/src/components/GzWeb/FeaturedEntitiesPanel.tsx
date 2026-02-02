import React from 'react';

export const FeaturedEntitiesPanel: React.FC = () => {
  return (
    <div className="featured-entities-container">
      <h3>Featured entities</h3>
      <div className="featured-entities-list">
        {/* Sample entity cards */}
        <div className="entity-card">
          <div className="entity-header">
            <span className="entity-icon">🚁</span>
            <span className="entity-name">Drone Alpha</span>
            <span className="entity-status status-active">Active</span>
          </div>
          <div className="entity-details">
            <div className="entity-metric">
              <span className="metric-label">Battery:</span>
              <span className="metric-value">87%</span>
            </div>
            <div className="entity-metric">
              <span className="metric-label">Altitude:</span>
              <span className="metric-value">15.2m</span>
            </div>
          </div>
        </div>
        <div className="entity-card">
          <div className="entity-header">
            <span className="entity-icon">📦</span>
            <span className="entity-name">Package Delivery</span>
            <span className="entity-status status-pending">Pending</span>
          </div>
          <div className="entity-details">
            <div className="entity-metric">
              <span className="metric-label">Weight:</span>
              <span className="metric-value">2.3kg</span>
            </div>
            <div className="entity-metric">
              <span className="metric-label">Priority:</span>
              <span className="metric-value">High</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};
