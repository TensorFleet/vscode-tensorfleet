import React from 'react';
import { GzWebPanel } from './GzWebPanel';
import { FeaturedEntitiesPanel } from './FeaturedEntitiesPanel';
import './ESimViewPanel.css';

export const ESimViewPanel: React.FC = () => {
  return (
    <div className="esim-wrapper">
      <div className="esim-main-view">
        <GzWebPanel />
      </div>
      <div className="esim-sidebar">
        <FeaturedEntitiesPanel />
      </div>
    </div>
  );
};
