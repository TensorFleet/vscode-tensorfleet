/**
 * WebSocket Proxy Client
 *
 * Handles connection through vm-manager WebSocket proxy with login handshake.
 * This allows ROS/Foxglove WebSocket connections to go through the vm-manager
 * proxy for authentication and access control.
 *
 * Protocol:
 * 1. Connect to vm-manager WebSocket endpoint
 * 2. Send login message with JWT token and target port
 * 3. Wait for loginResponse
 * 4. On success, proxy is established and normal messages flow through
 */

export interface ProxyConnectionConfig {
  /** VM Manager WebSocket proxy URL (e.g., wss://eu.vm.tensorfleet.net/ws) */
  proxyUrl: string;
  /** JWT auth token */
  token: string;
  /** Node/VM ID to connect to */
  nodeId: string;
  /** Target port inside the VM (8765 for Foxglove, 9091 for ROSbridge) */
  targetPort: number;
  /** WebSocket subprotocols to request */
  subprotocols?: string[];
}

export interface LoginMessage {
  type: 'login';
  token: string;
  nodeId: string;
  targetPort: number;
}

export interface LoginResponse {
  type: 'loginResponse';
  success: boolean;
  sessionId?: string;
  nodeId?: string;
  targetPort?: number;
  message?: string;
}

export type ProxyConnectionState =
  | 'connecting'
  | 'authenticating'
  | 'connected'
  | 'disconnected'
  | 'error';

export interface ProxyClientEvents {
  onStateChange?: (state: ProxyConnectionState) => void;
  onOpen?: () => void;
  onClose?: (event: CloseEvent) => void;
  onError?: (error: Event | Error) => void;
  onMessage?: (data: ArrayBuffer | string) => void;
  onLoginSuccess?: (response: LoginResponse) => void;
  onLoginFailed?: (response: LoginResponse) => void;
}

declare global {
  interface Window {
    OriginalWebSocket?: typeof WebSocket;
  }
}

/**
 * WebSocket client that connects through vm-manager proxy with authentication
 */
export class ProxyWebSocketClient {
  private ws: WebSocket | null = null;
  private config: ProxyConnectionConfig;
  private events: ProxyClientEvents;
  private state: ProxyConnectionState = 'disconnected';
  private sessionId: string | null = null;
  private messageQueue: (ArrayBuffer | string)[] = [];
  private reconnectTimer: number | null = null;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;
  private reconnectDelay = 3000;

  // Keep a reference to the *actual* constructor used, so global WebSocket patches
  // can't break OPEN/CONNECTING constants.
  private WSConstructor: typeof WebSocket;

  constructor(config: ProxyConnectionConfig, events: ProxyClientEvents = {}) {
    this.config = config;
    this.events = events;
    this.WSConstructor = (window.OriginalWebSocket ?? WebSocket) as typeof WebSocket;
  }

  /**
   * Replace event handlers (used by proxy shims to wrap native WebSocket events).
   */
  setEventHandlers(handlers: ProxyClientEvents): void {
    this.events = handlers;
  }

  /**
   * Get current event handlers (useful when composing/wrapping handlers).
   */
  getEventHandlers(): ProxyClientEvents {
    return this.events;
  }

  /**
   * Connect to the proxy
   */
  connect(): void {
    if (this.ws) {
      this.close();
    }

    this.setState('connecting');
    console.log('[ProxyWsClient] Connecting to proxy:', this.config.proxyUrl);

    try {
      // Use OriginalWebSocket if available (e.g., when WebSocket is patched by extension)
      this.WSConstructor = (window.OriginalWebSocket ?? WebSocket) as typeof WebSocket;

      // Connect with optional subprotocols
      this.ws = this.config.subprotocols?.length
        ? new this.WSConstructor(this.config.proxyUrl, this.config.subprotocols)
        : new this.WSConstructor(this.config.proxyUrl);

      this.ws.binaryType = 'arraybuffer';

      this.ws.onopen = () => {
        console.log('[ProxyWsClient] WebSocket connected, sending login...');
        this.setState('authenticating');
        this.sendLogin();
      };

      this.ws.onmessage = (event) => {
        this.handleMessage(event);
      };

      this.ws.onclose = (event) => {
        console.log('[ProxyWsClient] WebSocket closed:', event.code, event.reason);
        this.setState('disconnected');
        this.events?.onClose?.(event);

        // Attempt reconnect if not intentionally closed
        if (event.code !== 1000 && this.reconnectAttempts < this.maxReconnectAttempts) {
          this.scheduleReconnect();
        }
      };

      this.ws.onerror = (event) => {
        console.error('[ProxyWsClient] WebSocket error:', event);
        this.setState('error');
        this.events?.onError?.(event);
      };
    } catch (error) {
      console.error('[ProxyWsClient] Failed to create WebSocket:', error);
      this.setState('error');
      this.events?.onError?.(error instanceof Error ? error : new Error(String(error)));
    }
  }

  /**
   * Close the connection
   */
  close(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }

    if (this.ws) {
      this.ws.onclose = null; // Prevent reconnect on intentional close
      this.ws.close(1000, 'Client closed');
      this.ws = null;
    }

    this.setState('disconnected');
    this.sessionId = null;
    this.messageQueue = [];
    this.reconnectAttempts = 0;
  }

  /**
   * Send data through the proxy (queued if not yet authenticated)
   */
  send(data: ArrayBuffer | string): void {
    if (this.state !== 'connected') {
      // Queue messages until connected
      this.messageQueue.push(data);
      console.log('[ProxyWsClient] Message queued (not yet connected)');
      return;
    }

    if (this.ws && this.ws.readyState === this.WSConstructor.OPEN) {
      this.ws.send(data);
    } else {
      console.warn('[ProxyWsClient] WebSocket not ready, queuing message');
      this.messageQueue.push(data);
    }
  }

  /**
   * Send binary data
   */
  sendBinary(data: ArrayBuffer | Uint8Array | ArrayBufferView): void {
    const view =
      typeof ArrayBuffer.isView === 'function' && ArrayBuffer.isView(data)
        ? new Uint8Array(
            (data as ArrayBufferView).buffer,
            (data as ArrayBufferView).byteOffset,
            (data as ArrayBufferView).byteLength,
          )
        : new Uint8Array(data as ArrayBuffer);
    const buffer = view.slice().buffer; // ensure a standalone ArrayBuffer (not Shared)
    this.send(buffer);
  }

  /**
   * Get current connection state
   */
  getState(): ProxyConnectionState {
    return this.state;
  }

  /**
   * Check if connected and authenticated
   */
  isConnected(): boolean {
    return this.state === 'connected' && this.ws?.readyState === this.WSConstructor.OPEN;
  }

  /**
   * Get session ID (set after successful login)
   */
  getSessionId(): string | null {
    return this.sessionId;
  }

  /**
   * Update authentication token (for token refresh)
   */
  updateToken(token: string): void {
    this.config.token = token;
  }

  /**
   * Get the underlying WebSocket (use with caution)
   */
  getRawWebSocket(): WebSocket | null {
    return this.ws;
  }

  // Private methods

  private setState(state: ProxyConnectionState): void {
    if (this.state !== state) {
      console.log(`[ProxyWsClient] State: ${this.state} -> ${state}`);
      this.state = state;
      this.events?.onStateChange?.(state);
    }
  }

  private sendLogin(): void {
    if (!this.ws || this.ws.readyState !== this.WSConstructor.OPEN) {
      console.error('[ProxyWsClient] Cannot send login: WebSocket not open');
      return;
    }

    const loginMsg: LoginMessage = {
      type: 'login',
      token: this.config.token,
      nodeId: this.config.nodeId,
      targetPort: this.config.targetPort,
    };

    console.log(
      '[ProxyWsClient] Sending login for node:',
      this.config.nodeId,
      'port:',
      this.config.targetPort,
    );
    this.ws.send(JSON.stringify(loginMsg));
  }

  private handleMessage(event: MessageEvent): void {
    const data = event.data;

    // If still authenticating, check for login response
    if (this.state === 'authenticating') {
      if (typeof data === 'string') {
        try {
          const response = JSON.parse(data) as LoginResponse;

          if (response.type === 'loginResponse') {
            this.handleLoginResponse(response);
            return;
          }
        } catch (e) {
          console.warn('[ProxyWsClient] Failed to parse login response:', e);
        }
      }

      // If we get non-login response while authenticating, something is wrong
      console.warn('[ProxyWsClient] Received unexpected message while authenticating');
      return;
    }

    // Forward to client handler
    this.events.onMessage?.(data);
  }

  private handleLoginResponse(response: LoginResponse): void {
    if (response.success) {
      console.log('[ProxyWsClient] Login successful, session:', response.sessionId);
      this.sessionId = response.sessionId ?? null;
      this.setState('connected');
      this.reconnectAttempts = 0;
      this.events?.onLoginSuccess?.(response);
      this.events?.onOpen?.();

      // Flush queued messages
      this.flushMessageQueue();
    } else {
      console.error('[ProxyWsClient] Login failed:', response.message);
      this.setState('error');
      this.events?.onLoginFailed?.(response);
      this.events?.onError?.(new Error(response.message || 'Login failed'));

      // Close connection on auth failure (don't reconnect with same credentials)
      this.close();
    }
  }

  private flushMessageQueue(): void {
    if (this.messageQueue.length === 0) return;

    console.log('[ProxyWsClient] Flushing', this.messageQueue.length, 'queued messages');

    while (this.messageQueue.length > 0) {
      const msg = this.messageQueue.shift();
      if (msg && this.ws && this.ws.readyState === this.WSConstructor.OPEN) {
        this.ws.send(msg);
      }
    }
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer) return;

    this.reconnectAttempts++;
    const delay = this.reconnectDelay * Math.pow(1.5, this.reconnectAttempts - 1);

    console.log(
      `[ProxyWsClient] Scheduling reconnect attempt ${this.reconnectAttempts} in ${delay}ms`,
    );

    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, delay);
  }
}

/**
 * Create a proxy WebSocket that implements the standard WebSocket interface
 * This allows drop-in replacement of direct WebSocket connections
 */
export function createProxyWebSocket(config: ProxyConnectionConfig): WebSocket {
  const proxyClient = new ProxyWebSocketClient(config);

  const WSConstructor = (window.OriginalWebSocket ?? WebSocket) as typeof WebSocket;

  // Create a mock WebSocket object that delegates to the proxy client
  // This is a simplified shim - for full compatibility, use the ProxyWebSocketClient directly
  const ws = {
    binaryType: 'arraybuffer' as BinaryType,

    get readyState() {
      return proxyClient.isConnected() ? WSConstructor.OPEN : WSConstructor.CONNECTING;
    },

    send(data: string | ArrayBuffer | ArrayBufferView) {
      if (data instanceof ArrayBuffer) {
        proxyClient.send(data);
      } else if (ArrayBuffer.isView(data)) {
        proxyClient.sendBinary(data as Uint8Array);
      } else {
        proxyClient.send(data);
      }
    },

    close(code?: number, reason?: string) {
      proxyClient.close();
    },

    // Event handlers (set by FoxgloveClient)
    onopen: null as ((ev: Event) => void) | null,
    onclose: null as ((ev: CloseEvent) => void) | null,
    onerror: null as ((ev: Event) => void) | null,
    onmessage: null as ((ev: MessageEvent) => void) | null,

    // Constants (mirror from the real constructor we captured)
    CONNECTING: WSConstructor.CONNECTING,
    OPEN: WSConstructor.OPEN,
    CLOSING: WSConstructor.CLOSING,
    CLOSED: WSConstructor.CLOSED,

    // Additional properties for compatibility
    bufferedAmount: 0,
    extensions: '',
    protocol: '',
    url: config.proxyUrl,
  } as unknown as WebSocket;

  // Set up event forwarding
  proxyClient.connect();

  // Store proxy client reference for access
  (ws as any)._proxyClient = proxyClient;

  return ws;
}

/**
 * Helper to get WebSocket proxy URL from vm-manager base URL
 */
export function getProxyWebSocketUrl(vmManagerUrl: string): string {
  const url = new URL(vmManagerUrl);

  // If already a websocket URL, just normalize the path
  if (url.protocol === 'ws:' || url.protocol === 'wss:') {
    if (!url.pathname || url.pathname === '/') {
      url.pathname = '/ws';
    }
    return url.toString();
  }

  // Convert http(s) to ws(s) and ensure /ws path
  const protocol = url.protocol === 'https:' ? 'wss:' : 'ws:';
  const basePath = url.pathname.endsWith('/') ? url.pathname.slice(0, -1) : url.pathname;
  const path = basePath.endsWith('/ws') ? basePath : `${basePath}/ws`;

  return `${protocol}//${url.host}${path}`;
}

/**
 * Standard ports for ROS services
 */
export const ROS_PORTS = {
  FOXGLOVE_BRIDGE: 8765,
  ROSBRIDGE: 9091,
} as const;
