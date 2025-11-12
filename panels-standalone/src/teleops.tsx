import React from 'react';
import ReactDOM from 'react-dom/client';
import { TeleopPanel } from './components/Teleop/TeleopPanel';
import './global.css';

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <TeleopPanel />
  </React.StrictMode>
);

