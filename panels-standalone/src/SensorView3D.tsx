import "./global.css";

// ---- Browser polyfills for Node/Lichtblick globals ----
const g = globalThis as any;

if (g.global === undefined) g.global = g;
if (g.__filename === undefined) g.__filename = "browser";
if (g.__dirname === undefined) g.__dirname = "/";
if (g.ReactNull === undefined) g.ReactNull = null;
// -------------------------------------------------------

// Defer imports until after polyfills exist
(async () => {
  const React = await import("react");
  const ReactDOM = await import("react-dom/client");

  // ✅ IMPORTANT: expose React globally for Lichtblick internals
  (globalThis as any).React = React;

  const { Sensor3DViewPanel } = await import("./components/SensorView3D/SensorView3DPanel");

  const rootEl = document.getElementById("root");
  if (!rootEl) {
    throw new Error("#root element not found");
  }

  const root = ReactDOM.createRoot(rootEl);

  root.render(
    <React.StrictMode>
      <Sensor3DViewPanel />
    </React.StrictMode>
  );
})();
