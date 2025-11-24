import React from 'react';
import ReactDOM from 'react-dom/client';
import './global.css';
import { SensorView3DPanel } from './components/SensorView3D/SensorView3DPanel';

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <SensorView3DPanel></SensorView3DPanel>
  </React.StrictMode>
);

