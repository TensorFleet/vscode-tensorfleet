import React from 'react';
import ReactDOM from 'react-dom/client';
import { ImagePanel } from './components/ImagePanel';
import './global.css';

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <ImagePanel />
  </React.StrictMode>
);

