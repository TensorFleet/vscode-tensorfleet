import React from 'react';
import ReactDOM from 'react-dom/client';
import './global.css';
import { MissionControlPanel } from './components/MissionControl/MissionControl';

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <MissionControlPanel/>
  </React.StrictMode>
);

