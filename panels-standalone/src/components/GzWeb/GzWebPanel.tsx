import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { ExpandLess, ExpandMore, Wifi, WifiOff } from '@mui/icons-material';
import { IconButton, Tooltip } from '@mui/material';
import { SceneManager } from 'gzweb';
import {
  CARD_MESSAGES,
  EntityCardData,
  ENTITY_CONTROL_MESSAGES,
  EntityNudgeStatusState,
} from './EntityCardData';
import {
  addPoseVector,
  buildPoseNameAliases,
  GazeboPose,
  getEntityNameCandidates,
  isExpectedPoseObserved,
  poseNameMatchesAliases,
  resolvePoseEntry,
  resolveVisualOffset,
  toPoseQuaternion,
  toPoseVector,
  PoseQuaternion,
  PoseVector,
  unique,
} from './moveControl';
import { getGazeboEntityName } from './posePolicy';
import './GzWebPanel.css';

declare global {
  interface Window {
    OriginalWebSocket?: typeof WebSocket;
  }
}

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
  getModels?: () => Array<{ name?: string }>;
  transport?: SceneManagerTransport;
  scene?: any; // The Scene instance
  getModelByName?: (name: string) => any; // Method to get model by name
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
const MOVE_RESULT_TIMEOUT_MS = 2500;
const MOVE_MAX_ATTEMPTS = 3;
const MOVE_POSITION_TOLERANCE_METERS = 0.75;

type PendingMoveRequest = {
  requestId: string;
  entity: string;
  aliases: string[];
  expectedPosition: PoseVector;
  expectedDelta: PoseVector;
  expectedPoseId?: number;
  baselineByName: Map<string, PoseVector>;
  lastObserved?: {
    name?: string;
    id?: number;
    position: PoseVector;
    distanceToExpected: number;
  };
  attempt: number;
  maxAttempts: number;
  timeoutId?: ReturnType<typeof setTimeout>;
};

const shouldRequirePoseConfirmation = (entity: EntityCardData): boolean => {
  const explicit = entity.params?.runtime_pose_confirm_required;
  if (typeof explicit === 'boolean') {
    return explicit;
  }

  const policy = entity.params?.pose_confirm_policy;
  if (typeof policy === 'string') {
    const normalized = policy.toLowerCase();
    if (normalized === 'required' || normalized === 'strict') {
      return true;
    }
    if (normalized === 'best_effort' || normalized === 'optional' || normalized === 'disabled') {
      return false;
    }
  }

  // PX4/flight-controlled vehicles can accept set_pose but immediately be driven
  // by controllers, making dynamic_pose-based confirmation unreliable.
  const entityType = entity.type.toLowerCase();
  const mappedName = getGazeboEntityName(entity).toLowerCase();
  const target = entity.target.toLowerCase();
  if (entityType === 'drone' || mappedName.includes('x500') || target.includes('mavros')) {
    return false;
  }

  return true;
};

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
    private _listeners: { [K in keyof WebSocketEventMap]: Array<(ev: any) => void> } = {
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
        this._ws.send(JSON.stringify(loginMsg) as any);
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

    send(data: string | ArrayBufferLike | Blob | ArrayBufferView) {
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

const toEntityCardData = (payload: unknown): EntityCardData | null => {
  if (!payload || typeof payload !== 'object') return null;
  const entity = (payload as { entity?: unknown }).entity;
  if (entity && typeof entity === 'object') {
    const candidate = entity as Partial<EntityCardData>;
    const params = (candidate.params ?? {}) as Record<string, unknown>;
    const directTarget = typeof candidate.target === 'string' && candidate.target.trim().length > 0
      ? candidate.target.trim()
      : undefined;
    const modelNames = params.model_names;
    const modelTarget = Array.isArray(modelNames) && typeof modelNames[0] === 'string' && modelNames[0].trim().length > 0
      ? modelNames[0].trim()
      : undefined;
    const resolvedTarget = modelTarget ?? directTarget;
    const directName = typeof candidate.name === 'string' && candidate.name.trim().length > 0
      ? candidate.name.trim()
      : undefined;
    const displayName = typeof params.display_name === 'string' && params.display_name.trim().length > 0
      ? params.display_name.trim()
      : undefined;
    const resolvedName = directName ?? displayName ?? resolvedTarget;

    if (resolvedName && resolvedTarget) {
      return {
        name: resolvedName,
        type: typeof candidate.type === 'string' ? candidate.type : 'unknown',
        target: resolvedTarget,
        params,
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

const resolveSceneModelObject = (
  manager: SceneManagerInstance,
  requestedName: string,
): any | null => {
  const world = manager.transport?.getWorld?.();
  const sceneApi = (manager as any)?.scene;
  const getByName = typeof sceneApi?.getByName === 'function'
    ? sceneApi.getByName.bind(sceneApi)
    : null;
  const getModelByName = typeof manager.getModelByName === 'function'
    ? manager.getModelByName.bind(manager)
    : null;

  const includeBase = requestedName.endsWith('_include')
    ? requestedName.slice(0, -'_include'.length)
    : undefined;
  const includeVariant = requestedName.endsWith('_include')
    ? undefined
    : `${requestedName}_include`;
  const unscoped = requestedName.includes('::')
    ? requestedName.split('::').slice(1).join('::')
    : undefined;
  const candidates = unique([
    requestedName,
    includeBase,
    includeVariant,
    unscoped,
    world ? `${world}::${requestedName}` : undefined,
    world && includeBase ? `${world}::${includeBase}` : undefined,
  ]);

  for (const candidate of candidates) {
    const byModel = getModelByName?.(candidate);
    if (byModel) return byModel;
    const bySceneName = getByName?.(candidate);
    if (bySceneName) return bySceneName;
  }

  const models = manager.getModels?.() ?? [];
  for (const model of models) {
    const modelName = typeof model?.name === 'string' ? model.name : '';
    if (!modelName) continue;
    if (candidates.some((candidate) => modelName === candidate || modelName.endsWith(`::${candidate}`))) {
      return getModelByName?.(modelName) ?? getByName?.(modelName) ?? null;
    }
  }

  return null;
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

  const moveRequestCounterRef = useRef(0);
  const pendingMoveRef = useRef<PendingMoveRequest | null>(null);

  const clearVisualOffsets = useCallback((aliases: string[]) => {
    for (const alias of aliases) {
      entityVisualOffsetRef.current.delete(alias);
    }
  }, []);

  const clearPendingMoveTimer = useCallback((pending: PendingMoveRequest | null) => {
    if (!pending?.timeoutId) return;
    clearTimeout(pending.timeoutId);
    pending.timeoutId = undefined;
  }, []);

  const clearPendingMove = useCallback((reason?: string) => {
    const pending = pendingMoveRef.current;
    if (!pending) return;
    clearPendingMoveTimer(pending);
    clearVisualOffsets(pending.aliases);
    if (reason) {
      const payload = {
        requestId: pending.requestId,
        entity: pending.entity,
        state: 'error' as const,
        message: reason,
        attempt: pending.attempt,
        maxAttempts: pending.maxAttempts,
        timestamp: Date.now(),
      };
      setEntityControlStatus(payload.message);
      window.parent.postMessage(
        { type: ENTITY_CONTROL_MESSAGES.NUDGE_STATUS, payload },
        '*',
      );
    }
    pendingMoveRef.current = null;
  }, [clearPendingMoveTimer, clearVisualOffsets]);

  const emitNudgeStatus = useCallback((
    state: EntityNudgeStatusState,
    message: string,
    details: {
      requestId?: string;
      entity: string;
      attempt?: number;
      maxAttempts?: number;
    },
  ) => {
    const payload = {
      requestId: details.requestId,
      entity: details.entity,
      state,
      message,
      attempt: details.attempt,
      maxAttempts: details.maxAttempts,
      timestamp: Date.now(),
    };
    setEntityControlStatus(payload.message);
    window.parent.postMessage(
      { type: ENTITY_CONTROL_MESSAGES.NUDGE_STATUS, payload },
      '*',
    );
  }, []);

  const nudgeEntity = useCallback(
    (entity: EntityCardData, delta: PoseVector, requestIdFromMessage?: string) => {
      const manager = sceneManagerRef.current;
      const transport = manager?.transport;
      const mappedEntityName = getGazeboEntityName(entity);
      const requestId =
        requestIdFromMessage ??
        `nudge-${Date.now().toString(36)}-${(++moveRequestCounterRef.current).toString(36)}`;
      if (!manager || !transport) {
        emitNudgeStatus('error', 'Move ignored: scene not ready', { requestId, entity: mappedEntityName });
        return;
      }
      if (!transport.requestService) {
        emitNudgeStatus('error', 'Move failed: transport service calls are unavailable', {
          requestId,
          entity: mappedEntityName,
        });
        return;
      }

      const world = transport.getWorld?.();
      if (!world) {
        emitNudgeStatus('error', 'Move ignored: world is unavailable', { requestId, entity: mappedEntityName });
        return;
      }

      clearPendingMove();

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
        emitNudgeStatus('error', hint, { requestId, entity: mappedEntityName });
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

      const sendMoveRequests = (attempt: number): boolean => {
        let attemptRequested = false;
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
              attemptRequested = true;
              console.info('[GzWebPanel] Requested set_pose', {
                attempt,
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
              attemptRequested = true;
              console.info('[GzWebPanel] Requested set_pose_vector', {
                attempt,
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
        return attemptRequested;
      };

      requested = sendMoveRequests(1);

      if (!requested) {
        clearVisualOffsets(visualOffsetAliases);
        emitNudgeStatus(
          'error',
          'Move failed: no compatible Pose / Pose_V protobuf message types in current gzweb root',
          { requestId, entity: mappedEntityName, attempt: 1, maxAttempts: MOVE_MAX_ATTEMPTS },
        );
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
      const moveEntity = resolvedEntityName || mappedEntityName;
      const aliases = buildPoseNameAliases(world, unique([
        ...requestNames,
        poseName,
        worldScopedPoseName,
        unscopedPoseName,
      ]));
      const requirePoseConfirmation = shouldRequirePoseConfirmation(entity);

      if (!requirePoseConfirmation) {
        // Keep the visual nudge briefly, then let authoritative stream win.
        setTimeout(() => clearVisualOffsets(aliases), 800);
        emitNudgeStatus(
          'success',
          `Move request sent for ${moveEntity} (stream confirmation optional for this entity)`,
          { requestId, entity: moveEntity, attempt: 1, maxAttempts: 1 },
        );
        return;
      }

      const aliasSet = new Set(aliases);
      const baselineByName = new Map<string, PoseVector>();
      for (const [cachedPoseName, cachedPose] of entityPoseRef.current.entries()) {
        if (poseNameMatchesAliases(cachedPoseName, aliasSet)) {
          baselineByName.set(cachedPoseName, cachedPose.position);
        }
      }
      const pendingMove: PendingMoveRequest = {
        requestId,
        entity: moveEntity,
        aliases,
        expectedPosition: canonicalNextPose.position,
        expectedDelta: delta,
        expectedPoseId: currentPose.id,
        baselineByName,
        attempt: 1,
        maxAttempts: MOVE_MAX_ATTEMPTS,
      };
      pendingMoveRef.current = pendingMove;

      emitNudgeStatus(
        'pending',
        `Move requested for ${moveEntity} by (${delta.x.toFixed(2)}, ${delta.y.toFixed(2)}, ${delta.z.toFixed(2)})`,
        { requestId, entity: moveEntity, attempt: pendingMove.attempt, maxAttempts: pendingMove.maxAttempts },
      );

      const scheduleTimeout = () => {
        clearPendingMoveTimer(pendingMoveRef.current);
        pendingMove.timeoutId = setTimeout(() => {
          const active = pendingMoveRef.current;
          if (!active || active.requestId !== requestId) {
            return;
          }

          if (active.attempt >= active.maxAttempts) {
            const timeoutMessage = active.lastObserved
              ? `Move failed: timed out waiting for dynamic pose confirmation (last: ${active.lastObserved.name ?? 'unknown'} @ ${active.lastObserved.position.x.toFixed(2)}, ${active.lastObserved.position.y.toFixed(2)}, ${active.lastObserved.position.z.toFixed(2)}, dist ${active.lastObserved.distanceToExpected.toFixed(2)}m)`
              : `Move failed: timed out waiting for dynamic pose confirmation (aliases: ${active.aliases.slice(0, 4).join(', ')})`;
            clearPendingMove(timeoutMessage);
            return;
          }

          active.attempt += 1;
          emitNudgeStatus(
            'pending',
            `No pose update yet for ${active.entity}; retrying request`,
            { requestId, entity: active.entity, attempt: active.attempt, maxAttempts: active.maxAttempts },
          );
          const retried = sendMoveRequests(active.attempt);
          if (!retried) {
            clearPendingMove('Move failed: retry could not send any compatible service request');
            return;
          }
          scheduleTimeout();
        }, MOVE_RESULT_TIMEOUT_MS);
      };

      scheduleTimeout();
    },
    [clearPendingMove, clearPendingMoveTimer, clearVisualOffsets, emitNudgeStatus],
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

      const payload = (message.payload ?? {}) as { delta?: unknown; requestId?: unknown };
      const delta = toPoseVector(payload.delta);
      if (!delta) {
        setEntityControlStatus('Move ignored: invalid delta payload');
        return;
      }

      const requestId = typeof payload.requestId === 'string' ? payload.requestId : undefined;
      nudgeEntity(entity, delta, requestId);
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
    clearPendingMoveTimer(pendingMoveRef.current);
    pendingMoveRef.current = null;
    entityPoseRef.current.clear();
    entityVisualOffsetRef.current.clear();
    if (sceneManagerRef.current) {
      sceneManagerRef.current.destroy();
      sceneManagerRef.current = null;
    }
    setIsConnected(false);
    setStatusTone('muted');
    setStatusText('');
  }, [clearPendingMoveTimer]);

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
      
      // Handle hover messages for outline functionality
      if (msg && (msg.type === CARD_MESSAGES.HOVER_START || msg.type === CARD_MESSAGES.HOVER_END)) {
        handleHoverMessage(msg);
        return;
      }
      
      // Handle focus messages for camera focusing
      if (msg && msg.type === 'FOCUS_ON_MODELS') {
        handleFocusMessage(msg);
        return;
      }
      
      // Handle existing VM manager messages
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

    // Handle hover messages to add/remove objects from outline layer
  const handleHoverMessage = useCallback((msg: any) => {
    const manager = sceneManagerRef.current;
    if (!manager) {
      console.warn('SceneManager not available for hover message:', msg);
      return;
    }

    const { type, payload } = msg;
    const entityName = typeof payload?.entityName === 'string' ? payload.entityName : 'unknown';
    const modelNames = Array.isArray(payload?.modelNames)
      ? payload.modelNames.filter((name: unknown): name is string => typeof name === 'string' && name.length > 0)
      : [];
    if (modelNames.length === 0) {
      console.warn(`No model names provided for hover: ${entityName}`, payload);
      return;
    }

    // Get the 3D object from the modelMap using the entity name
    for (const modelName of modelNames) {
      try {
        const modelObject = resolveSceneModelObject(manager, modelName);
        
        if (!modelObject) {
          console.warn(`Model object not found for entity: ${entityName}/${modelName}`);
          continue;
        }

        // Get the Scene instance to access outline functions
        const scene = manager.scene;
        
        if (!scene) {
          console.warn('Scene not available for outline operations');
          continue;
        }

        if (type === CARD_MESSAGES.HOVER_START) {
          // Add object to outline layer
          if (scene.addOutlineRoot) {
            scene.addOutlineRoot(modelObject);
            console.log(`Added ${entityName} to outline layer`);
          } else if (scene.updateOutlineLayerMembership) {
            scene.updateOutlineLayerMembership(modelObject, true);
            console.log(`Enabled outline for ${entityName}`);
          }
        } else if (type === CARD_MESSAGES.HOVER_END) {
          // Remove object from outline layer
          if (scene.removeOutlineRoot) {
            scene.removeOutlineRoot(modelObject);
            console.log(`Removed ${entityName} from outline layer`);
          } else if (scene.updateOutlineLayerMembership) {
            scene.updateOutlineLayerMembership(modelObject, false);
            console.log(`Disabled outline for ${entityName}`);
          }
        }
      } catch (error) {
        console.error('Error handling hover message:', error);
      }
    }
  }, []);

  const handleFocusMessage = useCallback((msg: any) => {
    const manager = sceneManagerRef.current;
    if (!manager) {
      console.warn('SceneManager not available for focus message:', msg);
      return;
    }

    const { payload } = msg;
    const entityName = typeof payload?.entityName === 'string' ? payload.entityName : 'unknown';
    const modelNames = Array.isArray(payload?.modelNames)
      ? payload.modelNames.filter((name: unknown): name is string => typeof name === 'string' && name.length > 0)
      : [];
    if (modelNames.length === 0) {
      console.warn(`No model names provided for focus: ${entityName}`, payload);
      return;
    }

    console.log(`[GzWebPanel][FOCUS_MESSAGE]`, {
      entityName,
      modelNames,
      timestamp: payload.timestamp,
    });

    try {
      const resolvedObjects = modelNames
        .map((name: string) => resolveSceneModelObject(manager, name))
        .filter(Boolean);
      if (resolvedObjects.length === 0) {
        console.warn(`No model objects found for focus: ${entityName}`, modelNames);
        return;
      }

      const resolvedNames = resolvedObjects
        .map((obj: { name?: string }) => obj?.name)
        .filter((name: unknown): name is string => typeof name === 'string' && name.length > 0);
      const sceneManager = manager as any;

      // Prefer SceneManager helper when available, but pass resolved names.
      if (sceneManager.focusOnModels && resolvedNames.length > 0) {
        sceneManager.focusOnModels(resolvedNames, 800, 1.2);
        console.log(`Focused camera on models: ${resolvedNames.join(', ')}`);
        return;
      }

      // Fallback when helper is unavailable: focus camera directly on resolved objects.
      const scene = sceneManager.scene;
      if (scene?.cameraLerp?.focus) {
        scene.cameraLerp.focus(resolvedObjects, 800, 1.2);
        console.log(`Focused camera on model objects for: ${entityName}`);
      } else {
        console.warn('focusOnModels/cameraLerp.focus not available on SceneManager');
      }
    } catch (error) {
      console.error('Error focusing on models:', error);
    }
  }, []);

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

        const pendingMove = pendingMoveRef.current;
        if (pendingMove) {
          const aliasSet = new Set(pendingMove.aliases);
          for (const poseEntry of msg.pose ?? []) {
            const observedId = toOptionalNumber(poseEntry.id);
            const idMatch =
              typeof pendingMove.expectedPoseId === 'number' &&
              typeof observedId === 'number' &&
              observedId === pendingMove.expectedPoseId;
            const nameMatch =
              typeof poseEntry.name === 'string' &&
              poseNameMatchesAliases(poseEntry.name, aliasSet);
            if (!idMatch && !nameMatch) continue;
            const observedPosition = toPoseVector(poseEntry.position);
            if (!observedPosition) continue;
            const dx = observedPosition.x - pendingMove.expectedPosition.x;
            const dy = observedPosition.y - pendingMove.expectedPosition.y;
            const dz = observedPosition.z - pendingMove.expectedPosition.z;
            pendingMove.lastObserved = {
              name: poseEntry.name,
              id: observedId,
              position: observedPosition,
              distanceToExpected: Math.sqrt(dx * dx + dy * dy + dz * dz),
            };
          }

          const observed = isExpectedPoseObserved(
            msg.pose ?? [],
            aliasSet,
            pendingMove.expectedPosition,
            MOVE_POSITION_TOLERANCE_METERS,
            pendingMove.expectedPoseId,
            pendingMove.expectedDelta,
            pendingMove.baselineByName,
          );
          if (observed.matched) {
            clearPendingMoveTimer(pendingMove);
            clearVisualOffsets(pendingMove.aliases);
            pendingMoveRef.current = null;
            emitNudgeStatus('success', `Move confirmed for ${pendingMove.entity}`, {
              requestId: pendingMove.requestId,
              entity: pendingMove.entity,
              attempt: pendingMove.attempt,
              maxAttempts: pendingMove.maxAttempts,
            });
          }
        }

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
  }, [clearPendingMoveTimer, clearVisualOffsets, emitNudgeStatus, isConnected]);

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
