import React from 'react';
import ReactDOM from 'react-dom/client';
import './global.css';
import { FeaturedEntitiesPanel } from './components/GzWeb/FeaturedEntitiesPanel';
import './components/GzWeb/ESimViewPanel.css';

const FeaturedEntitiesMainView: React.FC = () => {
  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'center',
      justifyContent: 'center',
      height: '100%',
      padding: '40px',
      backgroundColor: '#ffffff'
    }}>
      <h1 style={{
        fontSize: '2rem',
        color: '#333',
        marginBottom: '20px',
        textAlign: 'center'
      }}>
        Featured Entities Dashboard
      </h1>
      <p style={{
        fontSize: '1.1rem',
        color: '#666',
        textAlign: 'center',
        maxWidth: '600px',
        lineHeight: '1.6'
      }}>
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
