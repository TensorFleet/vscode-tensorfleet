import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { ExpandLess, ExpandMore, Wifi, WifiOff } from '@mui/icons-material';
import { IconButton, Tooltip } from '@mui/material';
import './GzWebPanel.css';

type SceneManagerTransport = { root?: unknown };
type SceneManagerInstance = {
  destroy: () => void;
  resize: () => void;
  transport?: SceneManagerTransport;
};

type SceneManagerConstructor = new (args: {
  elementId: string;
  websocketUrl: string;
  enableLights?: boolean;
}) => SceneManagerInstance;

type LoginStatus = 'muted' | 'pending' | 'ok' | 'error';

const GZWEB_MODULE_URL = 'https://esm.sh/gzweb@2.0.14?bundle';
const SCENE_ELEMENT_ID = 'gz-scene';

const pixelFormatEnumJson = {
  gz: {
    nested: {
      msgs: {
        nested: {
          PixelFormatType: {
            values: {
              UNKNOWN_PIXEL_FORMAT: 0,
              L_INT8: 1,
              L_INT16: 2,
              RGB_INT8: 3,
              RGBA_INT8: 4,
              BGRA_INT8: 5,
              RGB_INT16: 6,
              RGB_INT32: 7,
              BGR_INT8: 8,
              BGR_INT16: 9,
              BGR_INT32: 10,
              R_FLOAT16: 11,
              RGB_FLOAT16: 12,
              R_FLOAT32: 13,
              RGB_FLOAT32: 14,
              BAYER_RGGB8: 15,
              BAYER_BGGR8: 16,
              BAYER_GBRG8: 17,
              BAYER_GRBG8: 18,
              BAYER_RGGB16: 19,
              BAYER_BGGR16: 20,
              BAYER_GBRG16: 21,
              BAYER_GRBG16: 22,
              COMPRESSED_RGBA_INT8: 23,
              RAW16: 24,
              RAW8: 25,
            },
          },
        },
      },
    },
  },
};

const textFromData = async (data: unknown) => {
  if (typeof data === 'string') return data;
  if (data instanceof Blob) return await data.text();
  if (data instanceof ArrayBuffer) return new TextDecoder().decode(data);
  if (ArrayBuffer.isView(data)) return new TextDecoder().decode(data.buffer);
  return String(data ?? '');
};

const ensurePixelFormatEnum = (root: any) => {
  if (!root) return false;
  try {
    if (root.lookup('gz.msgs.PixelFormatType')) {
      return false;
    }
  } catch {
    // lookup throws if enum is missing; we fall through to add it.
  }

  try {
    root.addJSON(pixelFormatEnumJson);
    if (typeof root.resolveAll === 'function') {
      root.resolveAll();
    }
    console.log('Injected fallback PixelFormatType enum into protobuf root');
    return true;
  } catch (err) {
    console.warn('Failed to inject PixelFormatType enum', err);
    return false;
  }
};

const patchTransportRoot = (transport: any) => {
  if (!transport || transport.__pixelFormatPatched) return;
  transport.__pixelFormatPatched = true;

  let currentRoot = transport.root;
  Object.defineProperty(transport, 'root', {
    configurable: true,
    enumerable: true,
    get() {
      return currentRoot;
    },
    set(value) {
      currentRoot = value;
      ensurePixelFormatEnum(currentRoot);
    },
  });

  ensurePixelFormatEnum(currentRoot);
};

type VmManagerShimOptions = {
  token: string;
  nodeId: string;
  onStatus: (status: { text: string; tone: LoginStatus }) => void;
};

const installVmManagerWebSocketShim = (
  NativeWebSocket: typeof WebSocket,
  { token, nodeId, onStatus }: VmManagerShimOptions,
) => {
  class VmManagerWebSocket {
    private _ws: WebSocket;
    private _ready = false;
    private _queue: unknown[] = [];
    private _listeners: { [K in keyof WebSocketEventMap]: Array<(ev: WebSocketEventMap[K]) => void> } = {
      open: [],
      message: [],
      close: [],
      error: [],
    };

    public onopen: ((this: WebSocket, ev: Event) => any) | null = null;
    public onmessage: ((this: WebSocket, ev: MessageEvent) => any) | null = null;
    public onclose: ((this: WebSocket, ev: CloseEvent) => any) | null = null;
    public onerror: ((this: WebSocket, ev: Event) => any) | null = null;

    public binaryType: BinaryType;
    public readonly url: string;
    public readonly protocol: string;
    public readonly extensions: string;

    constructor(url: string | URL, protocols?: string | string[]) {
      this._ws = new NativeWebSocket(url, protocols);
      this.url = this._ws.url;
      this.protocol = this._ws.protocol;
      this.extensions = this._ws.extensions;
      this.binaryType = this._ws.binaryType;

      this._ws.addEventListener('open', (ev) => {
        const loginMsg = { type: 'login', token: token ?? '', nodeId };
        this._ws.send(JSON.stringify(loginMsg));
        this._emit('open', ev);
      });

      this._ws.addEventListener('message', (ev) => {
        if (!this._ready) {
          void (async () => {
            try {
              const msg = JSON.parse(await textFromData(ev.data));
              if (msg.type === 'loginResponse') {
                if (msg.success) {
                  this._ready = true;
                  onStatus({ text: 'Login accepted', tone: 'ok' });
                  this._flushQueue();
                } else {
                  onStatus({ text: msg.message || 'Login failed', tone: 'error' });
                  this._ws.close();
                }
              }
            } catch (err) {
              console.warn('Failed to parse login response', err);
              onStatus({ text: 'Login response parse failed', tone: 'error' });
            }
          })();
          return;
        }

        // Ensure gzweb receives Blob data (it expects to FileReader.readAsText on event.data).
        let dataForClient: MessageEvent['data'] = ev.data;
        if (!(dataForClient instanceof Blob)) {
          if (dataForClient instanceof ArrayBuffer || ArrayBuffer.isView(dataForClient)) {
            dataForClient = new Blob([dataForClient]);
          } else {
            dataForClient = new Blob([String(dataForClient)]);
          }
        }

        const wrappedEvent = new MessageEvent('message', { data: dataForClient });
        this._emit('message', wrappedEvent);
      });

      this._ws.addEventListener('error', (ev) => this._emit('error', ev));
      this._ws.addEventListener('close', (ev) => this._emit('close', ev));
    }

    get readyState() {
      return this._ws.readyState;
    }

    get bufferedAmount() {
      return this._ws.bufferedAmount;
    }

    send(data: unknown) {
      if (!this._ready) {
        this._queue.push(data);
        return;
      }
      this._ws.send(data as any);
    }

    close(code?: number, reason?: string) {
      this._ws.close(code, reason);
    }

    addEventListener<K extends keyof WebSocketEventMap>(type: K, listener: (ev: WebSocketEventMap[K]) => void) {
      this._listeners[type]?.push(listener);
    }

    removeEventListener<K extends keyof WebSocketEventMap>(type: K, listener: (ev: WebSocketEventMap[K]) => void) {
      if (!this._listeners[type]) return;
      this._listeners[type] = this._listeners[type].filter((l) => l !== listener);
    }

    private _emit<K extends keyof WebSocketEventMap>(type: K, event: WebSocketEventMap[K]) {
      const handler = this[`on${type}` as const] as ((event: WebSocketEventMap[K]) => void) | null;
      if (typeof handler === 'function') {
        handler.call(this as unknown as WebSocket, event);
      }
      for (const listener of this._listeners[type] ?? []) {
        try {
          listener.call(this as unknown as WebSocket, event);
        } catch (err) {
          console.error(err);
        }
      }
    }

    private _flushQueue() {
      while (this._queue.length > 0) {
        const msg = this._queue.shift();
        this._ws.send(msg as any);
      }
    }
  }

  window.WebSocket = VmManagerWebSocket as unknown as typeof WebSocket;
};

const getInitialValue = (key: string, fallback: string) => {
  if (typeof window === 'undefined') return fallback;
  const params = new URLSearchParams(window.location.search);
  return params.get(key) ?? fallback;
};

export const GzWebPanel: React.FC = () => {
  const originalWebSocket = useRef<typeof WebSocket | null>(null);
  const sceneManagerRef = useRef<SceneManagerInstance | null>(null);
  const resizeHandlerRef = useRef<(() => void) | null>(null);
  const hasAutoConnected = useRef(false);

  const [vmBase, setVmBase] = useState(() => getInitialValue('vm', 'http://localhost:8080'));
  const [nodeId, setNodeId] = useState(() => getInitialValue('nodeId', ''));
  const [token, setToken] = useState(() => getInitialValue('token', ''));
  const [directWs, setDirectWs] = useState(() => getInitialValue('ws', 'ws://localhost:7681'));

  const [activeWsUrl, setActiveWsUrl] = useState('');
  const [statusText, setStatusText] = useState('');
  const [statusTone, setStatusTone] = useState<LoginStatus>('muted');
  const [isConnecting, setIsConnecting] = useState(false);
  const [isConnected, setIsConnected] = useState(false);
  const [isCollapsed, setIsCollapsed] = useState(false);

  const connectThroughVmManager = vmBase.trim() !== '' && nodeId.trim() !== '';

  const resetWebSocketToNative = useCallback(() => {
    if (originalWebSocket.current) {
      window.WebSocket = originalWebSocket.current;
    }
  }, []);

  const destroyScene = useCallback(() => {
    if (resizeHandlerRef.current) {
      window.removeEventListener('resize', resizeHandlerRef.current);
      resizeHandlerRef.current = null;
    }
    if (sceneManagerRef.current) {
      sceneManagerRef.current.destroy();
      sceneManagerRef.current = null;
    }
    setIsConnected(false);
    setStatusTone('muted');
    setStatusText('');
  }, []);

  const bindResize = useCallback((sceneMgr: SceneManagerInstance) => {
    const handler = () => sceneMgr?.resize();
    if (resizeHandlerRef.current) {
      window.removeEventListener('resize', resizeHandlerRef.current);
    }
    resizeHandlerRef.current = handler;
    window.addEventListener('resize', handler);
  }, []);

  const connect = useCallback(async () => {
    setIsConnecting(true);
    setStatusText(connectThroughVmManager ? 'Awaiting login...' : 'Connecting...');
    setStatusTone(connectThroughVmManager ? 'pending' : 'muted');

    destroyScene();

    if (!originalWebSocket.current) {
      originalWebSocket.current = window.WebSocket;
    }
    resetWebSocketToNative();

    let websocketUrl = directWs.trim() || 'ws://localhost:7681';
    if (connectThroughVmManager) {
      websocketUrl = vmBase.replace(/^http/, 'ws').replace(/\/$/, '') + '/ws';
      installVmManagerWebSocketShim(originalWebSocket.current, {
        token: token.trim(),
        nodeId: nodeId.trim(),
        onStatus: ({ text, tone }) => {
          setStatusText(text);
          setStatusTone(tone);
        },
      });
    }

    setActiveWsUrl(websocketUrl);

    try {
      const { SceneManager } = (await import(
        /* @vite-ignore */ GZWEB_MODULE_URL
      )) as { SceneManager: SceneManagerConstructor };

      const manager = new SceneManager({
        elementId: SCENE_ELEMENT_ID,
        websocketUrl,
        enableLights: true,
      });

      patchTransportRoot(manager.transport);
      sceneManagerRef.current = manager;
      bindResize(manager);
      setStatusTone(connectThroughVmManager ? 'pending' : 'ok');
      setStatusText(connectThroughVmManager ? 'Awaiting login...' : 'Connected to websocket');
      setIsConnected(true);
      setIsCollapsed(true); // Auto-collapse on successful connection start
    } catch (err) {
      console.error('Failed to load gzweb', err);
      setStatusTone('error');
      setStatusText('Failed to load gzweb module');
      setIsConnected(false);
    } finally {
      setIsConnecting(false);
    }
  }, [bindResize, connectThroughVmManager, destroyScene, directWs, nodeId, resetWebSocketToNative, token, vmBase]);

  const handleDisconnect = useCallback(() => {
    destroyScene();
    resetWebSocketToNative();
    setIsConnected(false);
    setStatusText('Disconnected');
    setStatusTone('muted');
    setIsCollapsed(false); // Auto-expand on disconnect
  }, [destroyScene, resetWebSocketToNative]);

  useEffect(() => {
    if (!hasAutoConnected.current) {
      hasAutoConnected.current = true;
      void connect();
    }
  }, [connect]);

  useEffect(() => {
    return () => {
      destroyScene();
      resetWebSocketToNative();
    };
  }, [destroyScene, resetWebSocketToNative]);

  const statusClass = useMemo(() => {
    switch (statusTone) {
      case 'ok':
        return 'gzweb-status-ok';
      case 'pending':
        return 'gzweb-status-pending';
      case 'error':
        return 'gzweb-status-error';
      default:
        return 'gzweb-status-muted';
    }
  }, [statusTone]);

  return (
    <div className="gzweb-root">
      <div className={`gzweb-overlay ${isCollapsed ? 'collapsed' : ''}`}>
        <div className="gzweb-header">
          <div className="gzweb-header-left">
            <div className={`gzweb-connection-dot ${statusTone}`} />
            <div className="gzweb-title">gzweb viewer</div>
          </div>
          <div className="gzweb-header-right">
            <Tooltip title={isConnected ? "Connected" : "Disconnected"}>
              {isConnected ? <Wifi fontSize="small" className="status-icon-connected" /> : <WifiOff fontSize="small" className="status-icon-disconnected" />}
            </Tooltip>
            <IconButton
              size="small"
              onClick={() => setIsCollapsed(!isCollapsed)}
              className="gzweb-collapse-btn"
            >
              {isCollapsed ? <ExpandMore /> : <ExpandLess />}
            </IconButton>
          </div>
        </div>

        {!isCollapsed && (
          <div className="gzweb-content">
            <p className="gzweb-subtitle">
              Connect directly to a Gazebo websocket or through a VM manager login handshake.
            </p>

            <div className="gzweb-meta">
              <div>
                WS: <code>{activeWsUrl || 'unset'}</code>
              </div>
              {connectThroughVmManager && (
                <div>
                  VM: <code>{nodeId.trim() || 'unset'}</code>
                </div>
              )}
              {statusText && (
                <div className={`gzweb-status ${statusClass}`}>
                  Status: <span>{statusText}</span>
                </div>
              )}
            </div>

            <form
              className="gzweb-controls"
              onSubmit={(event) => {
                event.preventDefault();
                if (isConnected) {
                  handleDisconnect();
                } else {
                  void connect();
                }
              }}
            >
              <label>
                VM Base
                <input
                  value={vmBase}
                  onChange={(e) => setVmBase(e.target.value)}
                  placeholder="http://localhost:8080"
                  spellCheck={false}
                  disabled={isConnected}
                />
              </label>

              <label>
                VM ID
                <input
                  value={nodeId}
                  onChange={(e) => setNodeId(e.target.value)}
                  placeholder="vm id from /vms/status"
                  spellCheck={false}
                  disabled={isConnected}
                />
              </label>

              <label>
                Token
                <input
                  value={token}
                  onChange={(e) => setToken(e.target.value)}
                  placeholder="optional (if SKIP_JWT_VALIDATION=true)"
                  spellCheck={false}
                  disabled={isConnected}
                />
              </label>

              <label>
                WS (direct)
                <input
                  value={directWs}
                  onChange={(e) => setDirectWs(e.target.value)}
                  placeholder="ws://localhost:7681"
                  spellCheck={false}
                  disabled={isConnected}
                />
              </label>

              <button
                type="submit"
                disabled={isConnecting}
                className={isConnected ? 'btn-disconnect' : 'btn-connect'}
              >
                {isConnecting ? 'Connecting...' : isConnected ? 'Disconnect' : 'Connect'}
              </button>
            </form>
          </div>
        )}
      </div>

      <div id={SCENE_ELEMENT_ID} className="gzweb-scene" />
    </div>
  );
};
