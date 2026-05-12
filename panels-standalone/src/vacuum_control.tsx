import React from "react";
import ReactDOM from "react-dom/client";
import { ConnectionSettingsProvider } from "./components/ConnectionSettingsProvider";
import { VacuumControlPanel } from "./components/VacuumControl/VacuumControlPanel";
import "./global.css";

ReactDOM.createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <ConnectionSettingsProvider>
      <VacuumControlPanel />
    </ConnectionSettingsProvider>
  </React.StrictMode>,
);
