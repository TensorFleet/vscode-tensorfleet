import React from 'react';
import ReactDOM from 'react-dom/client';
import { LeRobotMonitor } from './components/LeRobot/LeRobotMonitor';
import './global.css';

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <LeRobotMonitor />
  </React.StrictMode>
);
