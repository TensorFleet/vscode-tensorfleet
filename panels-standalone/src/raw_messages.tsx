import React from 'react';
import ReactDOM from 'react-dom/client';
import { RawMessagesPanel } from './components/RawMessages/RawMessagesPanel';
import './global.css';

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <RawMessagesPanel />
  </React.StrictMode>,
);
