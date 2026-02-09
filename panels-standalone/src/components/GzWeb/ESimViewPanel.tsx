import React from 'react';
import { GzWebPanel } from './GzWebPanel';
import { FeaturedEntitiesPanel } from './FeaturedEntitiesPanel';
import { EntityInfoPopup } from './EntityInfoPopup';
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
      {/* Self-contained EntityInfoPopup - renders when window message is received */}
      <EntityInfoPopup />
    </div>
  );
};
