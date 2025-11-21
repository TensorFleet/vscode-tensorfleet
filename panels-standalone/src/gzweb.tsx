import React from 'react';
import ReactDOM from 'react-dom/client';
import './global.css';
import { GzWebPanel } from './components/GzWeb/GzWebPanel';

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <GzWebPanel />
  </React.StrictMode>,
);
