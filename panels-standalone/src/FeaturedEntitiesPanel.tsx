import React from 'react';
import ReactDOM from 'react-dom/client';
import './global.css';
import { FeaturedEntitiesPanel } from './components/GzWeb/FeaturedEntitiesPanel';
import './components/GzWeb/ESimViewPanel.css';

const FeaturedEntitiesMainView: React.FC = () => {
  return (
    <div className="esim-main-content">
      <h1 className="esim-main-title">
        Featured Entities Dashboard
      </h1>
      <p className="esim-main-description">
        Monitor and manage important simulation entities in real-time.
        The sidebar shows detailed information about active entities including
        their status, metrics, and controls.
      </p>
    </div>
  );
};

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <div className="esim-wrapper">
      <div className="esim-main-view">
        <FeaturedEntitiesMainView />
      </div>
      <div className="esim-sidebar">
        <FeaturedEntitiesPanel />
      </div>
    </div>
  </React.StrictMode>,
);
