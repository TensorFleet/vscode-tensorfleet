import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { ExpandLess, ExpandMore, Wifi, WifiOff } from '@mui/icons-material';
import { IconButton, Tooltip } from '@mui/material';
import { EntityCardData, ENTITY_CONTROL_MESSAGES } from './EntityCardData';
import './GzWebPanel.css';

declare global {
  interface Window {
    OriginalWebSocket?: typeof WebSocket;
  }
}

type PoseVector = { x: number; y: number; z: number };
type PoseQuaternion = { x: number; y: number; z: number; w: number };
type GazeboPose = { name: string; position: PoseVector; orientation: PoseQuaternion; id?: number };

type SceneManagerTransport = {
  root?: unknown;
  getWorld?: () => string;
  getAvailableTopics?: () => Array<{ topic?: string; msg_type?: string }>;
  subscribe?: (topic: { name: string; cb: (msg: any) => void }) => void;
  unsubscribe?: (topicName: string) => void;
  requestService?: (service: string, msgType: string, msgObj: unknown) => void;
};

type SceneManagerInstance = {
  destroy: () => void;
  resize: () => void;
  select?: (entityName: string) => void;
  transport?: SceneManagerTransport;
};

type SceneManagerConstructor = new (args: {
  elementId: string;
  websocketUrl: string;
  enableLights?: boolean;
}) => SceneManagerInstance;

type LoginStatus = 'muted' | 'pending' | 'ok' | 'error';
type VmStatusResponse = {
  vm_id?: string;
  vmId?: string;
  status?: string;
  ip_address?: string;
};

type HostVmInfo = {
  vmId?: string;
  vmBase?: string;
  token?: string;
  status?: string;
  error?: string;
};

type VsCodeApi = {
  postMessage: (message: any) => void;
};

declare global {
  interface Window {
    acquireVsCodeApi?: () => VsCodeApi;
    TENSORFLEET_VM_MANAGER_URL?: string;
    TENSORFLEET_NODE_ID?: string;
    TENSORFLEET_JWT?: string;
  }
}

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
  onConnected: () => void;
  onDisconnected: () => void;
};

const installVmManagerWebSocketShim = (
  NativeWebSocket: typeof WebSocket,
  { token, nodeId, onStatus, onConnected, onDisconnected }: VmManagerShimOptions,
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
                  onConnected(); // Notify that we're actually connected
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

      this._ws.addEventListener('error', (ev) => {
        this._emit('error', ev);
        onDisconnected();
      });

      this._ws.addEventListener('close', (ev) => {
        this._emit('close', ev);
        onDisconnected();
      });
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

  if (!window.OriginalWebSocket) {
    window.OriginalWebSocket = window.WebSocket;
  }

  window.WebSocket = VmManagerWebSocket as unknown as typeof WebSocket;
};

const getInitialValue = (key: string, fallback: string) => {
  if (typeof window === 'undefined') return fallback;
  const params = new URLSearchParams(window.location.search);
  return params.get(key) ?? fallback;
};

const getInitialVmBase = () => {
  // 1. Extension-injected value (highest priority for remote server support)
  if (typeof window !== 'undefined' && window.TENSORFLEET_VM_MANAGER_URL) {
    return window.TENSORFLEET_VM_MANAGER_URL;
  }
  // 2. Query string override
  const fromQuery = getInitialValue('vm', '');
  if (fromQuery) return fromQuery;
  // 3. Dev mode same-origin proxy
  if (typeof window !== 'undefined' && window.location.hostname === 'localhost' && window.location.port === '5173') {
    return window.location.origin;
  }
  return 'http://localhost:8080';
};

const getInitialFlag = (key: string, fallback: boolean) => {
  const raw = getInitialValue(key, fallback ? '1' : '0').toLowerCase();
  return raw === '1' || raw === 'true' || raw === 'yes' || raw === 'on';
};

const getGazeboEntityName = (entity: EntityCardData): string => {
  const mapped = entity.params?.gazebo_entity;
  if (typeof mapped === 'string' && mapped.trim().length > 0) {
    return mapped.trim();
  }
  return entity.target;
};

const toEntityCardData = (payload: unknown): EntityCardData | null => {
  if (!payload || typeof payload !== 'object') return null;
  const entity = (payload as { entity?: unknown }).entity;
  if (entity && typeof entity === 'object') {
    const candidate = entity as Partial<EntityCardData>;
    if (typeof candidate.name === 'string' && typeof candidate.target === 'string') {
      return {
        name: candidate.name,
        type: typeof candidate.type === 'string' ? candidate.type : 'unknown',
        target: candidate.target,
        params: (candidate.params ?? {}) as Record<string, unknown>,
      };
    }
  }

  const target = (payload as { target?: unknown }).target;
  if (typeof target === 'string' && target.trim().length > 0) {
    return {
      name: target,
      type: 'unknown',
      target,
      params: {},
    };
  }
  return null;
};

const toPoseVector = (value: unknown): PoseVector | null => {
  if (!value || typeof value !== 'object') return null;
  const vector = value as Partial<PoseVector>;
  if (typeof vector.x !== 'number' || typeof vector.y !== 'number' || typeof vector.z !== 'number') {
    return null;
  }
  return { x: vector.x, y: vector.y, z: vector.z };
};

const toPoseQuaternion = (value: unknown): PoseQuaternion | null => {
  if (!value || typeof value !== 'object') return null;
  const q = value as Partial<PoseQuaternion>;
  if (typeof q.x !== 'number' || typeof q.y !== 'number' || typeof q.z !== 'number' || typeof q.w !== 'number') {
    return null;
  }
  return { x: q.x, y: q.y, z: q.z, w: q.w };
};

const POSE_MESSAGE_TYPE_CANDIDATES = [
  'gz.msgs.Pose',
  'ignition.msgs.Pose',
  'gazebo.msgs.Pose',
] as const;

const POSE_VECTOR_MESSAGE_TYPE_CANDIDATES = [
  'gz.msgs.Pose_V',
  'ignition.msgs.Pose_V',
  'gazebo.msgs.Pose_V',
] as const;

const getOrderedPoseMessageTypes = (transport: SceneManagerTransport, world: string): string[] => {
  const defaultOrder = [...POSE_MESSAGE_TYPE_CANDIDATES];
  const topicName = `/world/${world}/dynamic_pose/info`;
  const availableTopics = transport.getAvailableTopics?.();
  if (!Array.isArray(availableTopics)) return defaultOrder;

  const topicMeta = availableTopics.find((topic) => topic?.topic === topicName);
  const msgType = typeof topicMeta?.msg_type === 'string' ? topicMeta.msg_type : '';
  const preferredPrefix = msgType.startsWith('ignition.msgs.')
    ? 'ignition.msgs.'
    : msgType.startsWith('gazebo.msgs.')
      ? 'gazebo.msgs.'
      : msgType.startsWith('gz.msgs.')
        ? 'gz.msgs.'
        : '';

  if (!preferredPrefix) return defaultOrder;
  return defaultOrder.sort((a, b) => Number(b.startsWith(preferredPrefix)) - Number(a.startsWith(preferredPrefix)));
};

const getOrderedPoseVectorMessageTypes = (transport: SceneManagerTransport, world: string): string[] => {
  const defaultOrder = [...POSE_VECTOR_MESSAGE_TYPE_CANDIDATES];
  const topicName = `/world/${world}/dynamic_pose/info`;
  const availableTopics = transport.getAvailableTopics?.();
  if (!Array.isArray(availableTopics)) return defaultOrder;

  const topicMeta = availableTopics.find((topic) => topic?.topic === topicName);
  const msgType = typeof topicMeta?.msg_type === 'string' ? topicMeta.msg_type : '';
  const preferredPrefix = msgType.startsWith('ignition.msgs.')
    ? 'ignition.msgs.'
    : msgType.startsWith('gazebo.msgs.')
      ? 'gazebo.msgs.'
      : msgType.startsWith('gz.msgs.')
        ? 'gz.msgs.'
        : '';

  if (!preferredPrefix) return defaultOrder;
  return defaultOrder.sort((a, b) => Number(b.startsWith(preferredPrefix)) - Number(a.startsWith(preferredPrefix)));
};

const getEntityNameCandidates = (entity: EntityCardData): string[] => {
  const mapped = getGazeboEntityName(entity);
  const includeBase = mapped.endsWith('_include')
    ? mapped.slice(0, -'_include'.length)
    : undefined;
  const targetBase = entity.target.endsWith('_include')
    ? entity.target.slice(0, -'_include'.length)
    : undefined;
  return unique([
    mapped,
    includeBase,
    entity.target,
    targetBase,
    entity.name,
  ]);
};

const resolvePoseEntry = (
  poses: Map<string, GazeboPose>,
  requestedName: string,
): { poseName: string; pose: GazeboPose } | null => {
  const direct = poses.get(requestedName);
  if (direct) {
    return { poseName: requestedName, pose: direct };
  }

  const candidates: Array<{ poseName: string; pose: GazeboPose; score: number }> = [];

  for (const [poseName, pose] of poses.entries()) {
    let score = -1;
    if (poseName.endsWith(`::${requestedName}`)) {
      // world::model
      score = 30;
    } else if (poseName.startsWith(`${requestedName}::`)) {
      // model::link
      score = 20;
    } else if (poseName.includes(`::${requestedName}::`)) {
      // world::model::link
      score = 10;
    }
    if (score >= 0) {
      candidates.push({ poseName, pose, score });
    }
  }

  if (candidates.length === 0) return null;
  candidates.sort((a, b) => {
    if (a.score !== b.score) return b.score - a.score;
    return a.poseName.length - b.poseName.length;
  });
  return { poseName: candidates[0].poseName, pose: candidates[0].pose };
};

const toOptionalNumber = (value: unknown): number | undefined => {
  if (typeof value === 'number' && Number.isFinite(value)) return value;
  if (typeof value === 'string') {
    const parsed = Number(value);
    if (Number.isFinite(parsed)) return parsed;
  }
  if (value && typeof value === 'object') {
    const maybeWithToNumber = value as { toNumber?: () => number };
    if (typeof maybeWithToNumber.toNumber === 'function') {
      const n = maybeWithToNumber.toNumber();
      if (Number.isFinite(n)) return n;
    }
    const asRecord = value as Record<string, unknown>;
    if (typeof asRecord.low === 'number') {
      return asRecord.low;
    }
  }
  return undefined;
};

const addPoseVector = (a: PoseVector, b: PoseVector): PoseVector => ({
  x: a.x + b.x,
  y: a.y + b.y,
  z: a.z + b.z,
});

const resolveVisualOffset = (
  offsets: Map<string, PoseVector>,
  world: string,
  poseName: string,
): PoseVector | null => {
  const unscopedName = poseName.includes('::')
    ? poseName.split('::').slice(1).join('::')
    : undefined;
  return (
    offsets.get(poseName) ??
    offsets.get(`${world}::${poseName}`) ??
    (unscopedName ? offsets.get(unscopedName) : undefined) ??
    null
  );
};

const toPoseFromSceneObject = (obj: any): GazeboPose | null => {
  if (!obj || typeof obj !== 'object') return null;

  const position = obj.position as Partial<PoseVector> | undefined;
  const quaternion = obj.quaternion as Partial<PoseQuaternion> | undefined;
  if (
    !position || typeof position.x !== 'number' || typeof position.y !== 'number' || typeof position.z !== 'number' ||
    !quaternion || typeof quaternion.x !== 'number' || typeof quaternion.y !== 'number' ||
    typeof quaternion.z !== 'number' || typeof quaternion.w !== 'number'
  ) {
    return null;
  }

  return {
    name: typeof obj.name === 'string' ? obj.name : '',
    position: { x: position.x, y: position.y, z: position.z },
    orientation: { x: quaternion.x, y: quaternion.y, z: quaternion.z, w: quaternion.w },
  };
};

const resolvePoseEntryFromScene = (
  manager: SceneManagerInstance,
  world: string,
  requestedNames: string[],
): { poseName: string; pose: GazeboPose } | null => {
  const sceneManagerAny = manager as any;
  const sceneApi = sceneManagerAny?.scene;
  const getByName = typeof sceneApi?.getByName === 'function'
    ? sceneApi.getByName.bind(sceneApi)
    : null;

  if (!getByName) return null;

  const candidates = unique([
    ...requestedNames,
    ...requestedNames.map((name) => `${world}::${name}`),
  ]);

  for (const candidate of candidates) {
    try {
      const obj = getByName(candidate);
      const pose = toPoseFromSceneObject(obj);
      if (pose && pose.name) {
        return { poseName: pose.name, pose };
      }
    } catch {
      // Ignore lookup errors and continue with next candidate.
    }
  }

  return null;
};

const applyPoseToScene = (
  manager: SceneManagerInstance,
  world: string,
  poseNames: string[],
  pose: GazeboPose,
): boolean => {
  const sceneApi = (manager as any)?.scene;
  const getByName = typeof sceneApi?.getByName === 'function'
    ? sceneApi.getByName.bind(sceneApi)
    : null;
  if (!getByName) return false;

  const candidates = unique([
    ...poseNames,
    ...poseNames.map((name) => (name.includes('::') ? name : `${world}::${name}`)),
  ]);

  for (const candidate of candidates) {
    try {
      const obj = getByName(candidate);
      if (!obj) continue;
      const position = obj.position as { set?: (x: number, y: number, z: number) => void; x?: number; y?: number; z?: number } | undefined;
      const quaternion = obj.quaternion as { set?: (x: number, y: number, z: number, w: number) => void; x?: number; y?: number; z?: number; w?: number } | undefined;
      if (!position || !quaternion) continue;

      if (typeof position.set === 'function') {
        position.set(pose.position.x, pose.position.y, pose.position.z);
      } else {
        position.x = pose.position.x;
        position.y = pose.position.y;
        position.z = pose.position.z;
      }

      if (typeof quaternion.set === 'function') {
        quaternion.set(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      } else {
        quaternion.x = pose.orientation.x;
        quaternion.y = pose.orientation.y;
        quaternion.z = pose.orientation.z;
        quaternion.w = pose.orientation.w;
      }

      if (typeof obj.updateMatrixWorld === 'function') {
        obj.updateMatrixWorld(true);
      }
      return true;
    } catch {
      // Ignore lookup errors and continue with next candidate.
    }
  }

  return false;
};

const unique = (items: Array<string | undefined>) => {
  return [...new Set(items.filter((value): value is string => Boolean(value && value.trim().length > 0)))];
};

export const GzWebPanel: React.FC = () => {
  const originalWebSocket = useRef<typeof WebSocket | null>(null);
  const sceneManagerRef = useRef<SceneManagerInstance | null>(null);
  const resizeHandlerRef = useRef<(() => void) | null>(null);
  const hasAutoConnected = useRef(false);
  const vmIdAbortRef = useRef<AbortController | null>(null);
  const userEditedNodeId = useRef(false);
  const nodeIdRef = useRef('');
  const vscodeApiRef = useRef<VsCodeApi | null>(null);
  const hostVmInfoRef = useRef<HostVmInfo | null>(null);
  const hostVmResolversRef = useRef<Array<(info: HostVmInfo | null) => void>>([]);
  const vmBaseRef = useRef('');
  const tokenRef = useRef('');
  const userEditedVmBase = useRef(false);
  const userEditedToken = useRef(false);

  const [vmBase, setVmBase] = useState(getInitialVmBase);
  const [nodeId, setNodeId] = useState(() => {
    // Extension-injected nodeId takes priority
    if (typeof window !== 'undefined' && window.TENSORFLEET_NODE_ID) {
      return window.TENSORFLEET_NODE_ID;
    }
    return getInitialValue('nodeId', '');
  });
  const [token, setToken] = useState(() => {
    // Extension-injected JWT takes priority
    if (typeof window !== 'undefined' && window.TENSORFLEET_JWT) {
      return window.TENSORFLEET_JWT;
    }
    return getInitialValue('token', '');
  });
  const [autoVmId, setAutoVmId] = useState<string | null>(null);
  const [vmIdFetchState, setVmIdFetchState] = useState<'idle' | 'loading' | 'success' | 'error'>('idle');
  const [vmIdFetchError, setVmIdFetchError] = useState('');
  const [hasVsCodeBridge, setHasVsCodeBridge] = useState(
    () => typeof window !== 'undefined' && typeof window.acquireVsCodeApi === 'function',
  );

  if (!vmBaseRef.current) {
    vmBaseRef.current = vmBase;
  }

  if (!tokenRef.current) {
    tokenRef.current = token;
  }

  const [showConfig, setShowConfig] = useState(() => getInitialFlag('showConfig', false));
  const [activeWsUrl, setActiveWsUrl] = useState('');
  const [statusText, setStatusText] = useState('');
  const [statusTone, setStatusTone] = useState<LoginStatus>('muted');
  const [isConnecting, setIsConnecting] = useState(false);
  const [isConnected, setIsConnected] = useState(false);
  const [isCollapsed, setIsCollapsed] = useState(false);
  const [selectedEntity, setSelectedEntity] = useState<EntityCardData | null>(null);
  const [entityControlStatus, setEntityControlStatus] = useState('');
  const dynamicPoseTopicRef = useRef<string | null>(null);
  const dynamicPoseOriginalCbRef = useRef<((msg: { pose?: Array<{ name?: string; position?: unknown; orientation?: unknown; id?: unknown }> }) => void) | null>(null);
  const dynamicPoseWrappedCbRef = useRef<((msg: { pose?: Array<{ name?: string; position?: unknown; orientation?: unknown; id?: unknown }> }) => void) | null>(null);
  const entityPoseRef = useRef<Map<string, GazeboPose>>(new Map());
  const entityVisualOffsetRef = useRef<Map<string, PoseVector>>(new Map());

  const resetWebSocketToNative = useCallback(() => {
    if (originalWebSocket.current) {
      window.WebSocket = originalWebSocket.current;
    }
  }, []);

  useEffect(() => {
    nodeIdRef.current = nodeId;
  }, [nodeId]);

  useEffect(() => {
    vmBaseRef.current = vmBase;
  }, [vmBase]);

  useEffect(() => {
    tokenRef.current = token;
  }, [token]);

  const nudgeEntity = useCallback(
    (entity: EntityCardData, delta: PoseVector) => {
      const manager = sceneManagerRef.current;
      const transport = manager?.transport;
      if (!manager || !transport) {
        setEntityControlStatus('Move ignored: scene not ready');
        return;
      }
      if (!transport.requestService) {
        setEntityControlStatus('Move failed: transport service calls are unavailable');
        return;
      }

      const world = transport.getWorld?.();
      if (!world) {
        setEntityControlStatus('Move ignored: world is unavailable');
        return;
      }

      const entityNameCandidates = getEntityNameCandidates(entity);
      let resolvedPoseEntry: { poseName: string; pose: GazeboPose } | null = null;
      let resolvedEntityName = '';
      for (const entityName of entityNameCandidates) {
        resolvedPoseEntry = resolvePoseEntry(entityPoseRef.current, entityName);
        if (resolvedPoseEntry) {
          resolvedEntityName = entityName;
          break;
        }
      }

      if (!resolvedPoseEntry) {
        const fromScene = resolvePoseEntryFromScene(manager, world, entityNameCandidates);
        if (fromScene) {
          resolvedPoseEntry = fromScene;
          resolvedEntityName = fromScene.poseName;
          entityPoseRef.current.set(fromScene.poseName, fromScene.pose);
          console.info('[GzWebPanel] Using scene pose fallback', {
            world,
            entityNameCandidates,
            resolvedPoseName: fromScene.poseName,
          });
        }
      }

      if (!resolvedPoseEntry) {
        const hint =
          entityPoseRef.current.size === 0
            ? 'Move ignored: waiting for dynamic poses'
            : `Move ignored: no pose for ${entityNameCandidates.join(' / ')}`;
        setEntityControlStatus(hint);
        console.warn('[GzWebPanel] No pose entry found for entity candidates', entityNameCandidates, [
          ...entityPoseRef.current.keys(),
        ]);
        return;
      }
      const msgTypeCandidates = getOrderedPoseMessageTypes(transport, world);
      const msgVectorTypeCandidates = getOrderedPoseVectorMessageTypes(transport, world);

      let requested = false;
      const poseServiceName = `/world/${world}/set_pose`;
      const poseVectorServiceName = `/world/${world}/set_pose_vector`;
      const { poseName, pose: currentPose } = resolvedPoseEntry;
      const canonicalNextPose: GazeboPose = {
        name: poseName,
        position: {
          x: currentPose.position.x + delta.x,
          y: currentPose.position.y + delta.y,
          z: currentPose.position.z + delta.z,
        },
        orientation: currentPose.orientation,
        id: currentPose.id,
      };
      const worldScopedPoseName = poseName.includes('::') ? poseName : `${world}::${poseName}`;
      const unscopedPoseName = poseName.includes('::')
        ? poseName.split('::').slice(1).join('::')
        : undefined;
      const visualOffsetAliases = unique([
        poseName,
        worldScopedPoseName,
        unscopedPoseName,
        getGazeboEntityName(entity),
        resolvedEntityName,
      ]);
      for (const alias of visualOffsetAliases) {
        const prior = entityVisualOffsetRef.current.get(alias) ?? { x: 0, y: 0, z: 0 };
        entityVisualOffsetRef.current.set(alias, addPoseVector(prior, delta));
      }

      const requestNames = unique([
        poseName,
        poseName.includes('::') ? undefined : `${world}::${poseName}`,
        getGazeboEntityName(entity),
        resolvedEntityName,
      ]);

      for (const poseRequestName of requestNames) {
        const nextPose: GazeboPose = {
          name: poseRequestName,
          position: {
            x: canonicalNextPose.position.x,
            y: canonicalNextPose.position.y,
            z: canonicalNextPose.position.z,
          },
          orientation: canonicalNextPose.orientation,
        };
        if (typeof currentPose.id === 'number') {
          nextPose.id = currentPose.id;
        }

        for (const msgType of msgTypeCandidates) {
          try {
            transport.requestService(poseServiceName, msgType, nextPose);
            requested = true;
            console.info('[GzWebPanel] Requested set_pose', {
              world,
              serviceName: poseServiceName,
              msgType,
              poseName: poseRequestName,
              mappedEntity: getGazeboEntityName(entity),
              resolvedEntityName,
              resolvedPoseName: poseName,
              poseId: currentPose.id,
            });
          } catch (error) {
            const message = error instanceof Error ? error.message : String(error);
            if (!message.includes('no such type')) {
              console.warn(`set_pose request failed for ${msgType}`, error);
            }
          }
        }

        for (const msgType of msgVectorTypeCandidates) {
          try {
            transport.requestService(poseVectorServiceName, msgType, { pose: [nextPose] });
            requested = true;
            console.info('[GzWebPanel] Requested set_pose_vector', {
              world,
              serviceName: poseVectorServiceName,
              msgType,
              poseName: poseRequestName,
              mappedEntity: getGazeboEntityName(entity),
              resolvedEntityName,
              resolvedPoseName: poseName,
              poseId: currentPose.id,
            });
          } catch (error) {
            const message = error instanceof Error ? error.message : String(error);
            if (!message.includes('no such type')) {
              console.warn(`set_pose_vector request failed for ${msgType}`, error);
            }
          }
        }
      }

      if (!requested) {
        setEntityControlStatus('Move failed: no compatible Pose / Pose_V protobuf message types in current gzweb root');
        return;
      }

      // Optimistically update pose cache so repeated clicks move from latest requested pose.
      entityPoseRef.current.set(poseName, canonicalNextPose);
      applyPoseToScene(
        manager,
        world,
        unique([
          poseName,
          worldScopedPoseName,
          getGazeboEntityName(entity),
          resolvedEntityName,
        ]),
        canonicalNextPose,
      );
      setEntityControlStatus(
        `Move requested for ${resolvedEntityName || getGazeboEntityName(entity)} by (${delta.x.toFixed(2)}, ${delta.y.toFixed(2)}, ${delta.z.toFixed(2)})`,
      );
    },
    [],
  );

  const handleEntityControlMessage = useCallback(
    (event: MessageEvent) => {
      const message = event.data as { type?: string; payload?: unknown };
      if (!message?.type) return;

      if (message.type === ENTITY_CONTROL_MESSAGES.SELECT) {
        const entity = toEntityCardData(message.payload);
        if (!entity) return;
        setSelectedEntity(entity);
        const gazeboEntity = getGazeboEntityName(entity);
        sceneManagerRef.current?.select?.(gazeboEntity);
        setEntityControlStatus(`Selected ${gazeboEntity}`);
        return;
      }

      if (message.type !== ENTITY_CONTROL_MESSAGES.NUDGE) return;

      const entityFromMessage = toEntityCardData(message.payload);
      const entity = entityFromMessage ?? selectedEntity;
      if (!entity) {
        setEntityControlStatus('Move ignored: no selected entity');
        return;
      }

      const payload = (message.payload ?? {}) as { delta?: unknown };
      const delta = toPoseVector(payload.delta);
      if (!delta) {
        setEntityControlStatus('Move ignored: invalid delta payload');
        return;
      }

      nudgeEntity(entity, delta);
    },
    [nudgeEntity, selectedEntity],
  );

  const destroyScene = useCallback(() => {
    if (resizeHandlerRef.current) {
      window.removeEventListener('resize', resizeHandlerRef.current);
      resizeHandlerRef.current = null;
    }
    const activeTopic = dynamicPoseTopicRef.current;
    const transport = sceneManagerRef.current?.transport as { topicMap?: Map<string, { cb: (msg: any) => void }> } | undefined;
    if (activeTopic && transport?.topicMap) {
      const topic = transport.topicMap.get(activeTopic);
      if (
        topic &&
        dynamicPoseWrappedCbRef.current &&
        topic.cb === dynamicPoseWrappedCbRef.current &&
        dynamicPoseOriginalCbRef.current
      ) {
        topic.cb = dynamicPoseOriginalCbRef.current;
      }
    }
    dynamicPoseTopicRef.current = null;
    dynamicPoseOriginalCbRef.current = null;
    dynamicPoseWrappedCbRef.current = null;
    entityPoseRef.current.clear();
    entityVisualOffsetRef.current.clear();
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

  const fetchVmIdFromManager = useCallback(
    async (
      options?: { quiet?: boolean },
    ): Promise<{
      vmId: string | null;
      error?: string;
    }> => {
      const trimmedVmBase = vmBase.trim();
      if (!trimmedVmBase) {
        if (!options?.quiet) {
          setVmIdFetchState('idle');
          setVmIdFetchError('');
        }
        return { vmId: null };
      }

      vmIdAbortRef.current?.abort();
      const controller = new AbortController();
      vmIdAbortRef.current = controller;

      if (!options?.quiet) {
        setVmIdFetchState('loading');
        setVmIdFetchError('');
      }

      try {
        const response = await fetch(trimmedVmBase.replace(/\/$/, '') + '/vms/self/status', {
          headers: token.trim() ? { Authorization: `Bearer ${token.trim()}` } : undefined,
          signal: controller.signal,
        });

        if (!response.ok) {
          throw new Error(`VM status returned ${response.status}`);
        }

        const payload = (await response.json()) as VmStatusResponse;
        const detectedVmId = payload.vm_id || payload.vmId;
        if (!detectedVmId) {
          throw new Error('VM ID missing from status response');
        }

        setVmIdFetchState('success');
        setVmIdFetchError('');
        setAutoVmId(detectedVmId);

        if (!userEditedNodeId.current || !nodeIdRef.current.trim()) {
          setNodeId(detectedVmId);
        }

        return { vmId: detectedVmId };
      } catch (error) {
        if ((error as Error)?.name === 'AbortError') {
          return { vmId: null };
        }
        setVmIdFetchState('error');
        const message = error instanceof Error ? error.message : 'VM ID lookup failed';
        setVmIdFetchError(message);
        return { vmId: null, error: message };
      }
    },
    [token, vmBase],
  );

  useEffect(() => {
    if (typeof window === 'undefined' || typeof window.acquireVsCodeApi !== 'function') return;

    const api = window.acquireVsCodeApi();
    vscodeApiRef.current = api;
    setHasVsCodeBridge(true);
    setVmIdFetchState((prev) => (prev === 'success' ? prev : 'loading'));
    setVmIdFetchError('');

    const handleMessage = (event: MessageEvent) => {
      const msg = event.data;
      if (!msg || msg.command !== 'tensorfleet.vmManagerInfo') return;
      const payload = (msg.payload ?? {}) as HostVmInfo;
      hostVmInfoRef.current = payload;

      if (payload.vmBase && !userEditedVmBase.current) {
        setVmBase(payload.vmBase);
      }
      if (payload.token && !userEditedToken.current) {
        setToken(payload.token);
      }

      if (payload.vmId) {
        setAutoVmId(payload.vmId);
        setVmIdFetchState('success');
        setVmIdFetchError('');
        if (!userEditedNodeId.current || !nodeIdRef.current.trim()) {
          setNodeId(payload.vmId);
        }
      } else if (payload.error) {
        setAutoVmId(null);
        setVmIdFetchState('error');
        setVmIdFetchError(payload.error);
      }

      const pendingResolvers = hostVmResolversRef.current.splice(0);
      for (const resolve of pendingResolvers) {
        resolve(payload);
      }
    };

    window.addEventListener('message', handleMessage);
    api.postMessage({
      command: 'tensorfleet.vmManagerInfo',
      payload: {
        vmBase: vmBaseRef.current.trim() || undefined,
        token: tokenRef.current.trim(),
      },
    });

    return () => {
      window.removeEventListener('message', handleMessage);
      hostVmResolversRef.current = [];
    };
  }, [setVmBase, setToken]);

  const requestHostVmInfo = useCallback(async (): Promise<HostVmInfo | null> => {
    const api = vscodeApiRef.current;
    if (!api) return null;

    return new Promise<HostVmInfo | null>((resolve) => {
      let timeout: ReturnType<typeof setTimeout>;
      const wrappedResolver = (info: HostVmInfo | null) => {
        clearTimeout(timeout);
        resolve(info);
      };

      timeout = setTimeout(() => {
        const idx = hostVmResolversRef.current.indexOf(wrappedResolver);
        if (idx !== -1) {
          hostVmResolversRef.current.splice(idx, 1);
        }
        resolve(hostVmInfoRef.current);
      }, 4000);

      hostVmResolversRef.current.push(wrappedResolver);
      api.postMessage({
        command: 'tensorfleet.vmManagerInfo',
        payload: {
          vmBase: vmBaseRef.current.trim() || undefined,
          token: tokenRef.current.trim(),
        },
      });
    });
  }, []);

  const connect = useCallback(async () => {
    let effectiveVmBase = vmBaseRef.current.trim();
    if (!effectiveVmBase && hasVsCodeBridge && hostVmInfoRef.current?.vmBase) {
      effectiveVmBase = hostVmInfoRef.current.vmBase.trim();
    }

    if (!effectiveVmBase) {
      setIsConnecting(false);
      setStatusTone('error');
      setStatusText('VM base URL is required');
      setIsConnected(false);
      setActiveWsUrl('');
      return;
    }

    setIsConnecting(true);
    setStatusTone('pending');
    setStatusText(nodeIdRef.current.trim() ? 'Preparing connection...' : 'Fetching VM ID...');

    let resolvedNodeId = nodeIdRef.current.trim();
    let vmIdError: string | undefined;

    if (!resolvedNodeId) {
      if (hasVsCodeBridge) {
        setVmIdFetchState((prev) => (prev === 'success' ? prev : 'loading'));
        setVmIdFetchError('');
        const hostInfo = hostVmInfoRef.current ?? (await requestHostVmInfo());

        if (hostInfo?.vmBase) {
          effectiveVmBase = hostInfo.vmBase.trim();
          if (!userEditedVmBase.current) {
            setVmBase(hostInfo.vmBase);
          }
        }

        if (hostInfo?.token && !userEditedToken.current) {
          setToken(hostInfo.token);
        }

        resolvedNodeId = hostInfo?.vmId?.trim() ?? '';
        vmIdError = hostInfo?.error;

        if (resolvedNodeId) {
          setAutoVmId(resolvedNodeId);
          setVmIdFetchState('success');
          setVmIdFetchError('');
        } else if (vmIdError) {
          setVmIdFetchState('error');
          setVmIdFetchError(vmIdError);
        }
      }

      if (!resolvedNodeId && !hasVsCodeBridge) {
        const { vmId, error } = await fetchVmIdFromManager();
        resolvedNodeId = vmId?.trim() ?? '';
        vmIdError = error;
      }
    }

    if (!resolvedNodeId) {
      setVmIdFetchState('error');
      setVmIdFetchError(vmIdError || 'VM ID unavailable');
      setIsConnecting(false);
      setStatusTone('error');
      setStatusText(vmIdError ? `VM ID unavailable: ${vmIdError}` : 'VM ID unavailable');
      setIsConnected(false);
      setActiveWsUrl('');
      return;
    }

    destroyScene();

    if (!originalWebSocket.current) {
      originalWebSocket.current = window.WebSocket;
    }
    resetWebSocketToNative();

    const websocketUrl = effectiveVmBase.replace(/^http/, 'ws').replace(/\/$/, '') + '/ws';
    installVmManagerWebSocketShim(originalWebSocket.current, {
      token: token.trim(),
      nodeId: resolvedNodeId,
      onStatus: ({ text, tone }) => {
        setStatusText(text);
        setStatusTone(tone);
      },
      onConnected: () => {
        setIsConnected(true);
        setIsCollapsed(true); // Auto-collapse on successful connection
      },
      onDisconnected: () => {
        setIsConnected(false);
        setStatusText('Connection lost');
        setStatusTone('error');
        setIsCollapsed(false); // Auto-expand on disconnect
      },
    });

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

      let frameCount = 0;
      let lastFrameLog = Date.now();

      const monitorPerformance = () => {
        frameCount++;
        const now = Date.now();
        if (now - lastFrameLog > 5000) {
          console.log(`📊 Client rendering: ${frameCount} frames/5s (${(frameCount / 5).toFixed(1)} fps)`);
          frameCount = 0;
          lastFrameLog = now;
        }
        requestAnimationFrame(monitorPerformance);
      };
      requestAnimationFrame(monitorPerformance);

      patchTransportRoot(manager.transport);
      sceneManagerRef.current = manager;
      bindResize(manager);

      // Keep status as pending until login succeeds
      setStatusTone('pending');
      setStatusText('Awaiting login...');
    } catch (err) {
      console.error('Failed to load gzweb', err);
      setStatusTone('error');
      setStatusText('Failed to load gzweb module');
      setIsConnected(false);
    } finally {
      setIsConnecting(false);
    }
  }, [bindResize, destroyScene, fetchVmIdFromManager, hasVsCodeBridge, requestHostVmInfo, resetWebSocketToNative, token]);

  const handleDisconnect = useCallback(() => {
    destroyScene();
    resetWebSocketToNative();
    setIsConnected(false);
    setStatusText('Disconnected');
    setStatusTone('muted');
    setIsCollapsed(false); // Auto-expand on disconnect
  }, [destroyScene, resetWebSocketToNative]);

  useEffect(() => {
    window.addEventListener('message', handleEntityControlMessage);
    return () => window.removeEventListener('message', handleEntityControlMessage);
  }, [handleEntityControlMessage]);

  useEffect(() => {
    if (!isConnected) return;

    const interval = setInterval(() => {
      const transport = sceneManagerRef.current?.transport;
      if (!transport) return;

      const world = transport.getWorld?.();
      if (!world) return;

      const topicName = `/world/${world}/dynamic_pose/info`;
      const transportWithMap = transport as unknown as {
        topicMap?: Map<string, { cb: (msg: { pose?: Array<{ name?: string; position?: unknown; orientation?: unknown; id?: unknown }> }) => void }>;
      };
      const topic = transportWithMap.topicMap?.get(topicName);
      if (!topic || typeof topic.cb !== 'function') return;
      if (dynamicPoseTopicRef.current === topicName && dynamicPoseWrappedCbRef.current === topic.cb) return;

      // Restore previously wrapped callback before wrapping a new world topic.
      if (
        dynamicPoseTopicRef.current &&
        dynamicPoseOriginalCbRef.current &&
        dynamicPoseWrappedCbRef.current
      ) {
        const prevTopic = transportWithMap.topicMap?.get(dynamicPoseTopicRef.current);
        if (prevTopic && prevTopic.cb === dynamicPoseWrappedCbRef.current) {
          prevTopic.cb = dynamicPoseOriginalCbRef.current;
        }
      }

      const originalCb = topic.cb;
      const wrappedCb = (msg: { pose?: Array<{ name?: string; position?: unknown; orientation?: unknown; id?: unknown }> }) => {
        let hasAdjustedPose = false;
        const adjustedPoseEntries = (msg.pose ?? []).map((poseEntry) => {
          if (!poseEntry.name) return poseEntry;
          const position = toPoseVector(poseEntry.position);
          if (!position) return poseEntry;
          const offset = resolveVisualOffset(entityVisualOffsetRef.current, world, poseEntry.name);
          if (!offset) return poseEntry;
          hasAdjustedPose = true;
          return {
            ...poseEntry,
            position: addPoseVector(position, offset),
          };
        });

        const msgForRender = hasAdjustedPose
          ? { ...msg, pose: adjustedPoseEntries }
          : msg;

        originalCb(msgForRender);

        for (const poseEntry of msgForRender.pose ?? []) {
          if (!poseEntry.name) continue;
          const position = toPoseVector(poseEntry.position);
          const orientation = toPoseQuaternion(poseEntry.orientation);
          if (!position || !orientation) continue;
          const id = toOptionalNumber(poseEntry.id);
          entityPoseRef.current.set(poseEntry.name, {
            name: poseEntry.name,
            position,
            orientation,
            id,
          });
        }
      };

      topic.cb = wrappedCb;
      dynamicPoseTopicRef.current = topicName;
      dynamicPoseOriginalCbRef.current = originalCb;
      dynamicPoseWrappedCbRef.current = wrappedCb;
    }, 250);

    return () => clearInterval(interval);
  }, [isConnected]);

  useEffect(() => {
    if (hasAutoConnected.current) return;
    if (!vmBase.trim()) return;
    if (!nodeId.trim() && vmIdFetchState !== 'success') return;

    hasAutoConnected.current = true;
    void connect();
  }, [connect, nodeId, vmBase, vmIdFetchState]);

  useEffect(() => {
    return () => {
      destroyScene();
      resetWebSocketToNative();
      vmIdAbortRef.current?.abort();
    };
  }, [destroyScene, resetWebSocketToNative]);

  useEffect(() => {
    if (hasVsCodeBridge) return;
    if (typeof window !== 'undefined' && typeof window.acquireVsCodeApi === 'function') return;
    if (!vmBase.trim()) return;
    void fetchVmIdFromManager({ quiet: true });
    return () => vmIdAbortRef.current?.abort();
  }, [fetchVmIdFromManager, hasVsCodeBridge, vmBase]);

  useEffect(() => {
    if (!hasVsCodeBridge) return;
    const trimmedVmBase = vmBase.trim();
    if (!trimmedVmBase) return;
    if (hostVmInfoRef.current?.vmBase?.trim() === trimmedVmBase) return;
    void requestHostVmInfo();
  }, [hasVsCodeBridge, requestHostVmInfo, vmBase]);

  useEffect(() => {
    if (vmBase.trim()) return;
    vmIdAbortRef.current?.abort();
    setAutoVmId(null);
    setVmIdFetchState('idle');
    setVmIdFetchError('');
  }, [vmBase]);

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

  const statusLabel = useMemo(() => {
    if (statusText) return statusText;
    if (isConnecting) return 'Connecting...';
    if (isConnected) return 'Connected';
    return 'Disconnected';
  }, [isConnected, isConnecting, statusText]);

  return (
    <div className="gzweb-root">
      {!showConfig && (
        <div className={`gzweb-status-pill ${statusTone}`}>
          <div className={`gzweb-connection-dot ${statusTone}`} />
          <div className="gzweb-status-pill-text">
            <span className="gzweb-pill-label">gzweb</span>
            <span className="gzweb-pill-value">{statusLabel}</span>
          </div>
          <button
            type="button"
            className="gzweb-pill-button"
            onClick={isConnected ? handleDisconnect : () => setShowConfig(true)}
            disabled={isConnecting}
          >
            {isConnected ? 'Disconnect' : 'Configure'}
          </button>
        </div>
      )}

      {showConfig && (
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
              <button
                className="gzweb-hide-config-btn"
                type="button"
                onClick={() => {
                  setShowConfig(false);
                  setIsCollapsed(false);
                }}
              >
                Hide
              </button>
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
                Connect to Gazebo through the VM manager websocket proxy (/ws) with login handshake.
              </p>

              <div className="gzweb-meta">
                <div>
                  Proxy WS: <code>{activeWsUrl || 'unset'}</code>
                </div>
                <div>
                  VM ID: <code>{nodeId.trim() || 'unset'}</code>
                </div>
                <div>
                  Selected: <code>{selectedEntity ? getGazeboEntityName(selectedEntity) : 'none'}</code>
                </div>
                <div className={`gzweb-status ${statusClass}`}>
                  Status: <span>{statusLabel}</span>
                </div>
                {entityControlStatus && (
                  <div className="gzweb-hint">
                    {entityControlStatus}
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
                    onChange={(e) => {
                      userEditedVmBase.current = e.target.value.trim().length > 0;
                      setVmBase(e.target.value);
                    }}
                    placeholder="http://localhost:8080"
                    spellCheck={false}
                    disabled={isConnected}
                  />
                </label>

                <label>
                  VM ID
                  <input
                    value={nodeId}
                    onChange={(e) => {
                      const value = e.target.value;
                      userEditedNodeId.current = value.trim().length > 0;
                      setNodeId(value);
                    }}
                    placeholder="vm id from /vms/status"
                    spellCheck={false}
                    disabled={isConnected}
                  />
                  <div className="gzweb-hint">
                    {vmIdFetchState === 'loading' && 'Auto-detecting from VM Manager...'}
                    {vmIdFetchState === 'success' &&
                      `Auto-detected ${autoVmId ?? nodeId.trim()}`}
                    {vmIdFetchState === 'error' && (
                      <span className="gzweb-hint-error">
                        Auto-detect failed: {vmIdFetchError}
                      </span>
                    )}
                    {vmIdFetchState === 'idle' && 'We will fetch your VM ID automatically.'}
                  </div>
                </label>

                <label>
                  Token
                  <input
                    value={token}
                    onChange={(e) => {
                      userEditedToken.current = e.target.value.trim().length > 0;
                      setToken(e.target.value);
                    }}
                    placeholder="optional (if SKIP_JWT_VALIDATION=true)"
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
      )}

      <div id={SCENE_ELEMENT_ID} className="gzweb-scene" />
    </div>
  );
};
