import "./global.css";
import React from 'react';
import ReactDOM from 'react-dom/client';

// ---- Browser polyfills for Node/Lichtblick globals ----
const g = globalThis as any;

if (g.global === undefined) g.global = g;
if (g.__filename === undefined) g.__filename = "browser";
if (g.__dirname === undefined) g.__dirname = "/";
if (g.ReactNull === undefined) g.ReactNull = null;
// -------------------------------------------------------

import { Sensor3DViewPanel } from "./components/SensorView3D/SensorView3DPanel";


ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    {/* <div
        style={{
          width: "100vw",
          height: "100vh",
          backgroundColor: "blue",
        }}
      /> */}
      <Sensor3DViewPanel />
  </React.StrictMode>
);
