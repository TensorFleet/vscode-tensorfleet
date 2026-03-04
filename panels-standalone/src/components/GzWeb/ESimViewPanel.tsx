import React from 'react';
import { GzWebPanel } from './GzWebPanel';
import { FeaturedEntitiesPanel } from './FeaturedEntitiesPanel';
import { EntityInfoPopup } from './EntityInfoPopup';
import SidePanel from './SidePanel';
import './ESimViewPanel.css';
import './SidePanel.css';

export const ESimViewPanel: React.FC = () => {
  return (
    <div className="esim-wrapper">
      <div className="esim-main-view">
        <GzWebPanel />
      </div>
      <SidePanel side="right">
        <FeaturedEntitiesPanel />
      </SidePanel>
      {/* Self-contained EntityInfoPopup - renders when window message is received */}
      <EntityInfoPopup />
    </div>
  );
};
