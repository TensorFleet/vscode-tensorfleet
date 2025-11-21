# gzweb panel (static)

Minimal standalone gzweb webview served from the Vite `publicDir`.

- Uses gzweb from CDN: `https://esm.sh/gzweb@2.0.14?bundle` (no bundling needed)
- Open `/gzweb/index.html?ws=ws://HOST:PORT` for a direct Gazebo websocket
- Or pass `vmBase=http://HOST:API_PORT&nodeId=...&token=...` to log in through the VM manager proxy (sends the login message before gzweb traffic)
- `auto=0` skips auto-connect on load; omit to auto-connect with the provided defaults/params
- Works in dev (`bun run dev` → http://localhost:5173/gzweb/) or as a static asset in the VS Code webview build

If you need a standalone server without Vite, run `bun scripts/gzweb-serve.ts` from the `panels-standalone/` directory or copy these files into any static host.
