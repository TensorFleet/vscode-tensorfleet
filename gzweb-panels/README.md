# gzweb-panels

Minimal, standalone gzweb webview you can drop anywhere. No build step; loads gzweb from a CDN. A tiny Bun server is included for local dev.

## Requirements
- Bun ≥ 1.0 for the static server.
- A browser/WebView with WebGL and WebSocket support.

## Layout
- `index.html` – full-screen viewer; pass your `ws://` URL via `?ws=` query param.
- `serve.ts` – Bun static server for local testing (`PORT` env var optional).

## Run locally (with Bun)
```bash
cd gzweb-panels
bun serve.ts           # defaults to http://localhost:3000
# or
PORT=4000 bun serve.ts
```
Open `http://localhost:3000/?ws=ws://YOUR_VM_IP:PORT`.

## What this does
- Uses gzweb from NPM via CDN: `https://unpkg.com/gzweb@2.0.14/dist/gzweb.module.js`.
- Creates a `SceneManager` targeting the div `#gz-scene`.
- Handles resize and cleanup on unload.

## Notes for future VS Code extension wiring
- The HTML here becomes your webview content; in the extension, swap the CDN URL for a `webview.asWebviewUri` to a vendored `gzweb.min.js` if you prefer offline.
- Set an appropriate CSP and pass the websocket URL from the extension host (e.g., `panel.webview.postMessage`).

## VM Manager WebSocket cheat sheet (for wiring the panel)
- Endpoint: `ws://<host>:<port>/ws` (same port as the HTTP API, e.g. `http://localhost:8080` → `ws://localhost:8080/ws`).
- First message (mandatory login): `{"type":"login","token":"<jwt-or-empty-in-dev>","nodeId":"<vm-id>"}`. Response is `loginResponse {success: boolean, message?: string}`; on failure the server closes the socket.
- Dev vs prod auth:
  - `SKIP_JWT_VALIDATION=true` → token can be empty; user becomes `dev-user`.
  - `SKIP_JWT_VALIDATION=false` → valid JWT required.
- VM rules: `nodeId` must exist and be `running`.
- After login: proxy opens a second WS to the VM’s Gazebo WS and forwards messages. Messages VM → client are untouched. Client → VM JSON with `op` is translated to Gazebo WS format (fills IDs, etc.). Examples:
  - `{"op":"ping"}`
  - `{"op":"subscribe","topic":"/gazebo/model_states","type":"gazebo_msgs/ModelStates"}`
  - `{"op":"publish","topic":"/cmd_vel","msg":{...}}`
  - `{"op":"call_service","service":"/gazebo/pause_physics","args":{}}`
- Errors/close: any side error closes both sockets; no auto-reconnect—client should handle it.

### Tiny Bun + TS client outline
```ts
// vm-ws-client.ts
type LoginMsg = { type: 'login'; token: string; nodeId: string };

export async function connect(baseUrl: string, nodeId: string, token = '') {
  const wsUrl = baseUrl.replace(/^http/, 'ws') + '/ws';
  return new Promise<WebSocket>((resolve, reject) => {
    const ws = new WebSocket(wsUrl);
    ws.onopen = () => ws.send(JSON.stringify({ type: 'login', token, nodeId } satisfies LoginMsg));
    ws.onmessage = (ev) => {
      const msg = JSON.parse(ev.data.toString());
      if (msg.type === 'loginResponse') {
        return msg.success ? resolve(ws) : reject(new Error(msg.message || 'login failed'));
      }
    };
    ws.onerror = (err) => reject(err instanceof Error ? err : new Error('ws error'));
  });
}

export const sendCmd = (ws: WebSocket, cmd: object) => ws.send(JSON.stringify(cmd));
```
Usage:
```ts
const ws = await connect('http://localhost:8080', 'vm-id', process.env.JWT_TOKEN ?? '');
sendCmd(ws, { op: 'ping' });
sendCmd(ws, { op: 'subscribe', topic: '/gazebo/model_states', type: 'gazebo_msgs/ModelStates' });
sendCmd(ws, { op: 'publish', topic: '/cmd_vel', msg: { linear: { x: 1 }, angular: { z: 0 } } });
```
