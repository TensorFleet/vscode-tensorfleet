import React from 'react';
import { GzWebPanel } from './GzWebPanel';
import { FeaturedEntitiesPanel } from './FeaturedEntitiesPanel';
import { InfoPopup } from './InfoPopup';
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
      {/* Self-contained InfoPopup - renders when window message is received */}
      <InfoPopup />
    </div>
  );
};
