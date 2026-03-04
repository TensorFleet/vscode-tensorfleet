import React from 'react';
import ReactDOM from 'react-dom/client';
import './global.css';
import { ESimViewPanel } from './components/GzWeb/ESimViewPanel';

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <ESimViewPanel />
  </React.StrictMode>,
);
