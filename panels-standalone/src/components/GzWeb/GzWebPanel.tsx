import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { ExpandLess, ExpandMore, Wifi, WifiOff } from '@mui/icons-material';
import { IconButton, Tooltip } from '@mui/material';
import { SceneManager } from 'gzweb';
import * as THREE from 'three';
import {
  CARD_MESSAGES,
  EntityCardData,
  ENTITY_CONTROL_MESSAGES,
  EntityNudgeStatusState,
  SceneSetupTraceConfigMessage,
} from './EntityCardData';
import {
  addPoseVector,
  buildPoseNameAliases,
  GazeboPose,
  getEntityNameCandidates,
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
    TENSORFLEET_MOVE_TRACE?: boolean | string;
  }
}

const GZWEB_MODULE_URL = 'https://esm.sh/gzweb@2.0.14?bundle';
const SCENE_ELEMENT_ID = 'gz-scene';
const SESSION_BASELINE_CAPTURE_WINDOW_MS = 8000;
const CONTACT_WARN_WINDOW_MS = 3000;
const CONTACT_WARN_COOLDOWN_MS = 1500;
const NUDGE_THROTTLE_MS = 120;

type MoveHistoryAction = 'append' | 'undo' | 'reset' | 'none';

type PendingThrottledNudge = {
  entity: EntityCardData;
  delta: PoseVector;
  requestId?: string;
  timerId?: ReturnType<typeof setTimeout>;
  queuedAtMs: number;
};

type EntityMoveHistory = {
  baseline: GazeboPose;
  deltas: PoseVector[];
};

type ScenePresetPose = {
  position: PoseVector;
  orientation: PoseQuaternion;
  id?: number;
};

type ScenePresetRecord = {
  name: string;
  capturedAt: number;
  world?: string;
  poses: Record<string, ScenePresetPose>;
};

type ManipAxis = 'x' | 'y' | 'z';

const DRAG_METERS_PER_PIXEL = 0.01;
const DRAG_MIN_DELTA_METERS = 0.02;

type SelectionSource = 'panel' | 'viewport' | 'none';

const isTruthyFlag = (value: unknown): boolean => {
  if (typeof value === 'boolean') return value;
  if (typeof value !== 'string') return false;
  const normalized = value.trim().toLowerCase();
  return normalized === '1' || normalized === 'true' || normalized === 'yes' || normalized === 'on';
};

const isMoveTraceEnabled = (): boolean => {
  if (typeof window === 'undefined') return false;
  if (isTruthyFlag(window.TENSORFLEET_MOVE_TRACE)) return true;

  try {
    const params = new URLSearchParams(window.location.search);
    if (
      isTruthyFlag(params.get('moveTrace')) ||
      isTruthyFlag(params.get('move_trace')) ||
      isTruthyFlag(params.get('debugMove'))
    ) {
      return true;
    }
  } catch {
    // Ignore malformed query string parsing.
  }

  try {
    if (isTruthyFlag(window.localStorage.getItem('tf.move.trace'))) {
      return true;
    }
  } catch {
    // Ignore localStorage access failures.
  }

  return false;
};

const getMoveTraceFilter = (): string => {
  if (typeof window === 'undefined') return '';

  try {
    const params = new URLSearchParams(window.location.search);
    const fromQuery =
      params.get('moveTraceFilter') ??
      params.get('move_trace_filter') ??
      params.get('traceFilter');
    if (typeof fromQuery === 'string' && fromQuery.trim().length > 0) {
      return fromQuery.trim().toLowerCase();
    }
  } catch {
    // Ignore malformed query string parsing.
  }

  try {
    const fromStorage = window.localStorage.getItem('tf.move.trace.filter');
    if (typeof fromStorage === 'string' && fromStorage.trim().length > 0) {
      return fromStorage.trim().toLowerCase();
    }
  } catch {
    // Ignore localStorage access failures.
  }

  return '';
};

const matchesMoveTraceFilter = (phase: string, payload: Record<string, unknown>, filter: string): boolean => {
  if (!filter) return true;

  const hints = [
    phase,
    typeof payload.requestId === 'string' ? payload.requestId : '',
    typeof payload.entity === 'string' ? payload.entity : '',
    typeof payload.entityName === 'string' ? payload.entityName : '',
    typeof payload.resolvedEntityName === 'string' ? payload.resolvedEntityName : '',
    typeof payload.mappedEntityName === 'string' ? payload.mappedEntityName : '',
    typeof payload.resolvedPoseName === 'string' ? payload.resolvedPoseName : '',
    typeof payload.world === 'string' ? payload.world : '',
    typeof payload.serviceName === 'string' ? payload.serviceName : '',
  ].join(' ').toLowerCase();

  if (hints.includes(filter)) return true;

  try {
    return (JSON.stringify(payload) ?? '').toLowerCase().includes(filter);
  } catch {
    return false;
  }
};

const toTracePosition = (value: PoseVector) => ({
  x: Number(value.x.toFixed(4)),
  y: Number(value.y.toFixed(4)),
  z: Number(value.z.toFixed(4)),
});

const toTraceOrientation = (value: PoseQuaternion) => ({
  x: Number(value.x.toFixed(4)),
  y: Number(value.y.toFixed(4)),
  z: Number(value.z.toFixed(4)),
  w: Number(value.w.toFixed(4)),
});

const moveTrace = (phase: string, payload: Record<string, unknown>) => {
  if (!isMoveTraceEnabled()) return;
  const filter = getMoveTraceFilter();
  if (filter && !matchesMoveTraceFilter(phase, payload, filter)) return;
  console.info('[MoveTrace]', {
    phase,
    ts: new Date().toISOString(),
    filter: filter || undefined,
    ...payload,
  });
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
    const canonicalTarget = typeof params.gazebo_entity === 'string' && params.gazebo_entity.trim().length > 0
      ? params.gazebo_entity.trim()
      : undefined;
    const modelNames = params.model_names;
    const modelTarget = Array.isArray(modelNames) && typeof modelNames[0] === 'string' && modelNames[0].trim().length > 0
      ? modelNames[0].trim()
      : undefined;
    const resolvedTarget = canonicalTarget ?? modelTarget ?? directTarget;
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

const toEntityCardDataList = (payload: unknown): EntityCardData[] => {
  if (!payload || typeof payload !== 'object') return [];
  const entities = (payload as { entities?: unknown }).entities;
  if (!Array.isArray(entities)) return [];
  const parsed: EntityCardData[] = [];
  for (const item of entities) {
    const entity = toEntityCardData({ entity: item });
    if (entity) parsed.push(entity);
  }
  return parsed;
};

const POSE_MESSAGE_TYPE_CANDIDATES = [
  'gz.msgs.Pose',
] as const;

const POSE_VECTOR_MESSAGE_TYPE_CANDIDATES = [
  'gz.msgs.Pose_V',
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

const resolveContactTopicName = (transport: SceneManagerTransport, world: string): string | null => {
  const preferred = [
    `/world/${world}/contact`,
    `/world/${world}/contacts`,
    `/world/${world}/physics/contacts`,
  ];
  const availableTopics = transport.getAvailableTopics?.();
  if (!Array.isArray(availableTopics)) {
    return preferred[0];
  }
  const availableNames = availableTopics
    .map((topic) => (typeof topic.topic === 'string' ? topic.topic : ''))
    .filter((name) => name.length > 0);
  for (const candidate of preferred) {
    if (availableNames.includes(candidate)) return candidate;
  }
  const fallback = availableNames.find(
    (name) => name.startsWith(`/world/${world}/`) && (name.includes('/contact') || name.endsWith('contact')),
  );
  return fallback ?? preferred[0];
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

  // Scene graph objects can be parented; use world transform for Gazebo set_pose.
  if (typeof obj.getWorldPosition === 'function' && typeof obj.getWorldQuaternion === 'function') {
    try {
      if (typeof obj.updateWorldMatrix === 'function') {
        obj.updateWorldMatrix(true, false);
      } else if (typeof obj.updateMatrixWorld === 'function') {
        obj.updateMatrixWorld(true);
      }
      const worldPosition = new THREE.Vector3();
      const worldQuaternion = new THREE.Quaternion();
      obj.getWorldPosition(worldPosition);
      obj.getWorldQuaternion(worldQuaternion);
      return {
        name: typeof obj.name === 'string' ? obj.name : '',
        position: { x: worldPosition.x, y: worldPosition.y, z: worldPosition.z },
        orientation: { x: worldQuaternion.x, y: worldQuaternion.y, z: worldQuaternion.z, w: worldQuaternion.w },
      };
    } catch {
      // Fall back to local transform extraction below.
    }
  }

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

const clonePose = (pose: GazeboPose): GazeboPose => ({
  name: pose.name,
  position: { ...pose.position },
  orientation: { ...pose.orientation },
  id: pose.id,
});

const invertPoseDelta = (delta: PoseVector): PoseVector => ({
  x: -delta.x,
  y: -delta.y,
  z: -delta.z,
});

const sortScenePresetNames = (store: Record<string, ScenePresetRecord>): string[] =>
  Object.entries(store)
    .sort(([, a], [, b]) => (b?.capturedAt ?? 0) - (a?.capturedAt ?? 0))
    .map(([name]) => name);

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
  const [selectedEntitySource, setSelectedEntitySource] = useState<SelectionSource>('none');
  const selectedEntityRef = useRef<EntityCardData | null>(null);
  const selectedEntitySourceRef = useRef<SelectionSource>('none');
  const [entityControlStatus, setEntityControlStatus] = useState('');
  const [directManipulationEnabled, setDirectManipulationEnabled] = useState(true);
  const [dragModeEnabled, setDragModeEnabled] = useState(false);
  const [dragAxis, setDragAxis] = useState<ManipAxis>('x');
  const [isDraggingAxis, setIsDraggingAxis] = useState(false);
  const [manipStatusText, setManipStatusText] = useState('Ready');
  const dynamicPoseTopicRef = useRef<string | null>(null);
  const dynamicPoseOriginalCbRef = useRef<((msg: { pose?: Array<{ name?: string; position?: unknown; orientation?: unknown; id?: unknown }> }) => void) | null>(null);
  const dynamicPoseWrappedCbRef = useRef<((msg: { pose?: Array<{ name?: string; position?: unknown; orientation?: unknown; id?: unknown }> }) => void) | null>(null);
  const contactTopicRef = useRef<string | null>(null);
  const contactOriginalCbRef = useRef<((msg: unknown) => void) | null>(null);
  const contactWrappedCbRef = useRef<((msg: unknown) => void) | null>(null);
  const recentMoveRef = useRef<{
    entity: string;
    aliases: string[];
    atMs: number;
    warnedAtMs?: number;
  } | null>(null);
  const entityPoseRef = useRef<Map<string, GazeboPose>>(new Map());
  const sessionInitialPoseRef = useRef<Map<string, GazeboPose>>(new Map());
  const sessionBaselineCaptureDeadlineRef = useRef(0);
  const entityVisualOffsetRef = useRef<Map<string, PoseVector>>(new Map());
  const sessionScenePresetsRef = useRef<Record<string, ScenePresetRecord>>({});

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

  useEffect(() => {
    selectedEntityRef.current = selectedEntity;
  }, [selectedEntity]);

  useEffect(() => {
    selectedEntitySourceRef.current = selectedEntitySource;
  }, [selectedEntitySource]);

  const moveRequestCounterRef = useRef(0);
  const pendingThrottledNudgesRef = useRef<Map<string, PendingThrottledNudge>>(new Map());
  const lastNudgeDispatchAtRef = useRef<Map<string, number>>(new Map());

  useEffect(() => {
    if (!isMoveTraceEnabled()) return;
    moveTrace('trace.enabled', {
      query: typeof window !== 'undefined' ? window.location.search : '',
      filter: getMoveTraceFilter(),
      vmBase: vmBaseRef.current,
      nodeId: nodeIdRef.current,
    });
  }, []);
  const moveHistoryRef = useRef<Map<string, EntityMoveHistory>>(new Map());
  const hoverOutlineRefs = useRef<Map<string, Set<any>>>(new Map());
  const dragStartPointRef = useRef<{ x: number; y: number } | null>(null);
  const dragDeltaMetersRef = useRef(0);
  const dragEntityRef = useRef<EntityCardData | null>(null);
  const isDraggingAxisRef = useRef(false);
  const controlsEnabledBeforeDragRef = useRef<boolean | null>(null);
  const dragPointerIdRef = useRef<number | null>(null);
  const setSceneControlsEnabled = useCallback((enabled: boolean) => {
    const controls = ((sceneManagerRef.current as any)?.scene?.controls as { enabled?: boolean } | undefined);
    if (typeof controls?.enabled === 'boolean') {
      controls.enabled = enabled;
    }
  }, []);

  const setManipStatus = useCallback((message: string) => {
    setManipStatusText(message);
  }, []);

  const emitScenePresetList = useCallback(() => {
    const names = sortScenePresetNames(sessionScenePresetsRef.current);
    window.parent.postMessage(
      {
        type: ENTITY_CONTROL_MESSAGES.SCENE_PRESET_LIST,
        payload: {
          names,
          timestamp: Date.now(),
        },
      },
      '*',
    );
  }, []);

  const recordSessionInitialPose = useCallback((pose: GazeboPose, options?: { force?: boolean }) => {
    if (!pose.name || sessionInitialPoseRef.current.has(pose.name)) return;
    const deadline = sessionBaselineCaptureDeadlineRef.current;
    if (!options?.force && deadline > 0 && Date.now() > deadline) return;
    sessionInitialPoseRef.current.set(pose.name, clonePose(pose));
  }, []);

  const clearVisualOffsets = useCallback((aliases: string[]) => {
    for (const alias of aliases) {
      entityVisualOffsetRef.current.delete(alias);
    }
  }, []);

  const clearHoverOutlineRefs = useCallback((manager?: SceneManagerInstance | null) => {
    const activeManager = manager ?? sceneManagerRef.current;
    const scene = (activeManager as any)?.scene;
    if (!scene || hoverOutlineRefs.current.size === 0) return;

    for (const objects of hoverOutlineRefs.current.values()) {
      for (const obj of objects) {
        try {
          if (scene.removeOutlineRoot) {
            scene.removeOutlineRoot(obj);
          } else if (scene.updateOutlineLayerMembership) {
            scene.updateOutlineLayerMembership(obj, false);
          }
        } catch {
          // Ignore cleanup failures during teardown/reconnect.
        }
      }
    }
    hoverOutlineRefs.current.clear();
  }, []);

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

  const applyMoveHistoryAction = useCallback((
    entityKey: string,
    delta: PoseVector,
    baseline: GazeboPose,
    historyAction: MoveHistoryAction,
  ) => {
    if (historyAction === 'none') return;

    if (historyAction === 'append') {
      const existing = moveHistoryRef.current.get(entityKey);
      if (!existing) {
        moveHistoryRef.current.set(entityKey, {
          baseline: clonePose(baseline),
          deltas: [{ ...delta }],
        });
        return;
      }
      existing.deltas.push({ ...delta });
      return;
    }

    if (historyAction === 'undo') {
      const history = moveHistoryRef.current.get(entityKey);
      if (!history || history.deltas.length === 0) return;
      history.deltas.pop();
      if (history.deltas.length === 0) {
        moveHistoryRef.current.delete(entityKey);
      }
      return;
    }

    if (historyAction === 'reset') {
      moveHistoryRef.current.delete(entityKey);
    }
  }, []);

  const dispatchNudgeEntity = useCallback(
    (
      entity: EntityCardData,
      delta: PoseVector,
      requestIdFromMessage?: string,
      options?: {
        historyAction?: MoveHistoryAction;
      },
    ): boolean => {
      const manager = sceneManagerRef.current;
      const transport = manager?.transport;
      const mappedEntityName = getGazeboEntityName(entity);
      const requestId =
        requestIdFromMessage ??
        `nudge-${Date.now().toString(36)}-${(++moveRequestCounterRef.current).toString(36)}`;
      if (!manager || !transport) {
        emitNudgeStatus('error', 'Move ignored: scene not ready', { requestId, entity: mappedEntityName });
        return false;
      }
      if (!transport.requestService) {
        emitNudgeStatus('error', 'Move failed: transport service calls are unavailable', {
          requestId,
          entity: mappedEntityName,
        });
        return false;
      }

      const world = transport.getWorld?.();
      if (!world) {
        emitNudgeStatus('error', 'Move ignored: world is unavailable', { requestId, entity: mappedEntityName });
        return false;
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
        const hint =
          entityPoseRef.current.size === 0
            ? 'Move ignored: waiting for dynamic poses'
            : `Move ignored: no dynamic pose for ${entityNameCandidates.join(' / ')}`;
        emitNudgeStatus('error', hint, { requestId, entity: mappedEntityName });
        console.warn('[GzWebPanel] No pose entry found for entity candidates', entityNameCandidates, [
          ...entityPoseRef.current.keys(),
        ]);
        return false;
      }
      const msgTypeCandidates = getOrderedPoseMessageTypes(transport, world);
      const msgType = msgTypeCandidates[0];
      const poseServiceName = `/world/${world}/set_pose`;
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

      if (!msgType) {
        moveTrace('move.dispatch.failed_no_compatible_type', {
          requestId,
          world,
          mappedEntityName,
          msgTypeCandidates,
        });
        emitNudgeStatus('error', 'Move failed: no compatible Pose protobuf message type in current gzweb root', {
          requestId,
          entity: mappedEntityName,
          attempt: 1,
          maxAttempts: 1,
        });
        return false;
      }

      const moveEntity = resolvedEntityName || mappedEntityName;
      moveTrace('move.dispatch.prepare', {
        requestId,
        world,
        entityName: entity.name,
        entityType: entity.type,
        mappedEntityName,
        resolvedEntityName,
        resolvedPoseName: poseName,
        entityNameCandidates,
        delta: toTracePosition(delta),
        currentPose: {
          name: currentPose.name,
          id: currentPose.id,
          position: toTracePosition(currentPose.position),
          orientation: toTraceOrientation(currentPose.orientation),
        },
        requestedPose: {
          name: canonicalNextPose.name,
          id: canonicalNextPose.id,
          position: toTracePosition(canonicalNextPose.position),
          orientation: toTraceOrientation(canonicalNextPose.orientation),
        },
        poseServiceName,
        msgTypeCandidates,
        selectedMsgType: msgType,
      });

      try {
        moveTrace('move.dispatch.set_pose', {
          requestId,
          attempt: 1,
          world,
          serviceName: poseServiceName,
          msgType,
          entity: moveEntity,
          pose: {
            name: canonicalNextPose.name,
            position: toTracePosition(canonicalNextPose.position),
            orientation: toTraceOrientation(canonicalNextPose.orientation),
          },
        });
        transport.requestService(poseServiceName, msgType, canonicalNextPose);
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        if (!message.includes('no such type')) {
          console.warn(`set_pose request failed for ${msgType}`, error);
        }
        moveTrace('move.dispatch.set_pose.error', {
          requestId,
          attempt: 1,
          world,
          serviceName: poseServiceName,
          msgType,
          poseName,
          error: message,
        });
        emitNudgeStatus('error', `Move failed: set_pose request error (${message})`, {
          requestId,
          entity: mappedEntityName,
          attempt: 1,
          maxAttempts: 1,
        });
        return false;
      }

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
      const aliases = buildPoseNameAliases(world, unique([
        poseName,
        worldScopedPoseName,
        unscopedPoseName,
      ]));
      recentMoveRef.current = {
        entity: moveEntity,
        aliases,
        atMs: Date.now(),
      };
      applyMoveHistoryAction(
        mappedEntityName,
        delta,
        currentPose,
        options?.historyAction ?? 'append',
      );
      moveTrace('move.dispatch.accepted', {
        requestId,
        world,
        serviceName: poseServiceName,
        msgType,
        entity: moveEntity,
        aliases,
        note: 'set_pose dispatched (fire-and-forget mode)',
      });
      emitNudgeStatus(
        'success',
        `Move set_pose sent for ${moveEntity}`,
        { requestId, entity: moveEntity, attempt: 1, maxAttempts: 1 },
      );
      return true;
    },
    [applyMoveHistoryAction, emitNudgeStatus],
  );

  const flushThrottledNudge = useCallback((entityKey: string, reason: string): boolean => {
    const pending = pendingThrottledNudgesRef.current.get(entityKey);
    if (!pending) return false;
    if (pending.timerId) {
      clearTimeout(pending.timerId);
    }
    pendingThrottledNudgesRef.current.delete(entityKey);
    moveTrace('move.throttle.flush', {
      entity: entityKey,
      reason,
      queuedForMs: Date.now() - pending.queuedAtMs,
      delta: toTracePosition(pending.delta),
    });
    const accepted = dispatchNudgeEntity(
      pending.entity,
      pending.delta,
      pending.requestId,
      { historyAction: 'append' },
    );
    if (accepted) {
      lastNudgeDispatchAtRef.current.set(entityKey, Date.now());
    }
    return accepted;
  }, [dispatchNudgeEntity]);

  const clearAllThrottledNudges = useCallback((reason: string) => {
    const pendingByEntity = pendingThrottledNudgesRef.current;
    if (pendingByEntity.size === 0) return;
    for (const pending of pendingByEntity.values()) {
      if (pending.timerId) {
        clearTimeout(pending.timerId);
      }
    }
    moveTrace('move.throttle.cleared', {
      reason,
      count: pendingByEntity.size,
      entities: [...pendingByEntity.keys()],
    });
    pendingByEntity.clear();
  }, []);

  const nudgeEntity = useCallback(
    (
      entity: EntityCardData,
      delta: PoseVector,
      requestIdFromMessage?: string,
      options?: {
        historyAction?: MoveHistoryAction;
      },
    ): boolean => {
      const historyAction = options?.historyAction ?? 'append';
      const entityKey = getGazeboEntityName(entity);
      if (!entityKey) {
        return dispatchNudgeEntity(entity, delta, requestIdFromMessage, { historyAction });
      }

      if (historyAction !== 'append') {
        flushThrottledNudge(entityKey, 'history_action');
        return dispatchNudgeEntity(entity, delta, requestIdFromMessage, { historyAction });
      }

      const now = Date.now();
      const lastDispatchAt = lastNudgeDispatchAtRef.current.get(entityKey) ?? 0;
      const elapsedMs = now - lastDispatchAt;
      if (elapsedMs >= NUDGE_THROTTLE_MS) {
        const accepted = dispatchNudgeEntity(entity, delta, requestIdFromMessage, { historyAction: 'append' });
        if (accepted) {
          lastNudgeDispatchAtRef.current.set(entityKey, Date.now());
        }
        return accepted;
      }

      const remainingMs = Math.max(0, NUDGE_THROTTLE_MS - elapsedMs);
      const existing = pendingThrottledNudgesRef.current.get(entityKey);
      const pending: PendingThrottledNudge = existing
        ? {
          ...existing,
          entity,
          delta: addPoseVector(existing.delta, delta),
          requestId: requestIdFromMessage ?? existing.requestId,
        }
        : {
          entity,
          delta: { ...delta },
          requestId: requestIdFromMessage,
          queuedAtMs: now,
        };

      if (!pending.timerId) {
        pending.timerId = setTimeout(() => {
          flushThrottledNudge(entityKey, 'timer');
        }, remainingMs);
      }

      pendingThrottledNudgesRef.current.set(entityKey, pending);
      moveTrace('move.throttle.queued', {
        entity: entityKey,
        remainingMs: Math.round(remainingMs),
        queuedDelta: toTracePosition(pending.delta),
      });
      return true;
    },
    [dispatchNudgeEntity, flushThrottledNudge],
  );

  const undoLastMove = useCallback((entity: EntityCardData): boolean => {
    const entityKey = getGazeboEntityName(entity);
    const history = moveHistoryRef.current.get(entityKey);
    if (!history || history.deltas.length === 0) {
      emitNudgeStatus('error', `Undo ignored: no move history for ${entityKey}`, {
        entity: entityKey,
      });
      return false;
    }

    const lastDelta = history.deltas[history.deltas.length - 1];
    const accepted = nudgeEntity(entity, invertPoseDelta(lastDelta), undefined, { historyAction: 'undo' });
    if (!accepted) {
      return false;
    }
    return true;
  }, [emitNudgeStatus, nudgeEntity]);

  const applyAbsolutePose = useCallback((
    pose: GazeboPose,
    poseNames: string[],
  ): boolean => {
    const manager = sceneManagerRef.current;
    const transport = manager?.transport;
    if (!manager || !transport || !transport.requestService) return false;
    const world = transport.getWorld?.();
    if (!world) return false;

    const poseTypes = getOrderedPoseMessageTypes(transport, world);
    const poseVectorTypes = getOrderedPoseVectorMessageTypes(transport, world);
    const poseServiceName = `/world/${world}/set_pose`;
    const poseVectorServiceName = `/world/${world}/set_pose_vector`;
    const requestNames = unique([
      pose.name,
      ...poseNames,
      ...poseNames.map((name) => (name.includes('::') ? undefined : `${world}::${name}`)),
    ]);
    moveTrace('reset.apply.prepare', {
      world,
      poseName: pose.name,
      poseNames,
      requestNames,
      requestedPose: {
        name: pose.name,
        id: pose.id,
        position: toTracePosition(pose.position),
        orientation: toTraceOrientation(pose.orientation),
      },
      poseServiceName,
      poseVectorServiceName,
      poseTypes,
      poseVectorTypes,
    });

    let totalRequests = 0;
    for (const requestName of requestNames) {
      const nextPose: GazeboPose = {
        name: requestName,
        position: { ...pose.position },
        orientation: { ...pose.orientation },
        id: pose.id,
      };
      for (const msgType of poseTypes) {
        try {
          moveTrace('reset.apply.set_pose', {
            world,
            serviceName: poseServiceName,
            msgType,
            pose: {
              name: nextPose.name,
              id: nextPose.id,
              position: toTracePosition(nextPose.position),
              orientation: toTraceOrientation(nextPose.orientation),
            },
          });
          transport.requestService(poseServiceName, msgType, nextPose);
          totalRequests += 1;
        } catch {
          // Ignore incompatible protobuf message variants.
        }
      }
      for (const msgType of poseVectorTypes) {
        try {
          moveTrace('reset.apply.set_pose_vector', {
            world,
            serviceName: poseVectorServiceName,
            msgType,
            pose: {
              name: nextPose.name,
              id: nextPose.id,
              position: toTracePosition(nextPose.position),
              orientation: toTraceOrientation(nextPose.orientation),
            },
          });
          transport.requestService(poseVectorServiceName, msgType, { pose: [nextPose] });
          totalRequests += 1;
        } catch {
          // Ignore incompatible protobuf message variants.
        }
      }
    }

    const aliases = buildPoseNameAliases(world, requestNames);
    clearVisualOffsets(aliases);
    applyPoseToScene(manager, world, aliases, pose);
    entityPoseRef.current.set(pose.name, clonePose(pose));
    moveTrace('reset.apply.sent', {
      world,
      poseName: pose.name,
      totalRequests,
      aliases,
    });
    return totalRequests > 0;
  }, [clearVisualOffsets]);

  const resetEntityPose = useCallback((entity: EntityCardData): boolean => {
    const entityKey = getGazeboEntityName(entity);
    const candidates = getEntityNameCandidates(entity);
    moveTrace('reset.entity.requested', {
      entityKey,
      candidates,
    });
    let initialPoseEntry: { poseName: string; pose: GazeboPose } | null = null;
    for (const candidate of candidates) {
      initialPoseEntry = resolvePoseEntry(sessionInitialPoseRef.current, candidate);
      if (initialPoseEntry) break;
    }

    if (!initialPoseEntry) {
      emitNudgeStatus('error', `Reset ignored: initial pose not captured for ${entityKey}`, {
        entity: entityKey,
      });
      return false;
    }

    const restoredPose = clonePose(initialPoseEntry.pose);
    const applied = applyAbsolutePose(restoredPose, unique([initialPoseEntry.poseName, ...candidates]));
    if (!applied) {
      emitNudgeStatus('error', `Reset failed: no compatible pose service for ${entityKey}`, {
        entity: entityKey,
      });
      return false;
    }
    moveTrace('reset.entity.applied', {
      entityKey,
      baselinePoseName: initialPoseEntry.poseName,
      restoredPose: {
        name: restoredPose.name,
        id: restoredPose.id,
        position: toTracePosition(restoredPose.position),
        orientation: toTraceOrientation(restoredPose.orientation),
      },
    });

    moveHistoryRef.current.delete(entityKey);
    emitNudgeStatus('success', `Reset ${entityKey} to session start pose`, {
      entity: entityKey,
    });
    return true;
  }, [applyAbsolutePose, emitNudgeStatus]);

  const resetAllPoses = useCallback((entities?: EntityCardData[]): boolean => {
    const providedEntities = Array.isArray(entities) ? entities : [];
    const targetPoses = new Map<string, GazeboPose>();
    const missingRequested: string[] = [];
    const failed: string[] = [];
    moveTrace('reset.all.requested', {
      requestedEntities: providedEntities.map((entity) => ({
        name: entity.name,
        target: entity.target,
        gazeboEntity: getGazeboEntityName(entity),
      })),
      movedHistoryKeys: [...moveHistoryRef.current.keys()],
      baselinePoseCount: sessionInitialPoseRef.current.size,
    });

    const addTargetPose = (pose: GazeboPose, fallbackName?: string) => {
      const poseName = pose.name || fallbackName;
      if (!poseName || targetPoses.has(poseName)) return;
      targetPoses.set(poseName, clonePose({ ...pose, name: poseName }));
    };

    if (providedEntities.length > 0) {
      const deduped: EntityCardData[] = [];
      const seen = new Set<string>();
      for (const entity of providedEntities) {
        const key = getGazeboEntityName(entity) || entity.name;
        if (!key || seen.has(key)) continue;
        seen.add(key);
        deduped.push(entity);
      }

      for (const entity of deduped) {
        const entityKey = getGazeboEntityName(entity) || entity.name;
        const candidates = getEntityNameCandidates(entity);
        let initialPoseEntry: { poseName: string; pose: GazeboPose } | null = null;
        for (const candidate of candidates) {
          initialPoseEntry = resolvePoseEntry(sessionInitialPoseRef.current, candidate);
          if (initialPoseEntry) break;
        }
        if (!initialPoseEntry) {
          missingRequested.push(entityKey);
          continue;
        }
        addTargetPose(initialPoseEntry.pose, initialPoseEntry.poseName);
      }
    }

    // Include all moved entities, including mouse-selected objects that may not
    // be part of the featured-entity list.
    for (const [entityKey] of moveHistoryRef.current.entries()) {
      const fromSession = resolvePoseEntry(sessionInitialPoseRef.current, entityKey);
      if (fromSession) {
        addTargetPose(fromSession.pose, fromSession.poseName);
      } else {
        missingRequested.push(entityKey);
      }
    }

    if (targetPoses.size === 0) {
      for (const [poseName, pose] of sessionInitialPoseRef.current.entries()) {
        addTargetPose(pose, poseName);
      }
    }

    if (targetPoses.size === 0) {
      emitNudgeStatus('error', 'Reset all failed: no session baseline poses are available yet', {
        entity: '__scene__',
      });
      return false;
    }

    let resetCount = 0;
    for (const [poseName, pose] of targetPoses.entries()) {
      const applied = applyAbsolutePose(pose, [poseName]);
      if (!applied) {
        failed.push(poseName);
        continue;
      }
      resetCount += 1;
    }

    if (resetCount === 0) {
      emitNudgeStatus('error', 'Reset all failed: no compatible pose service calls succeeded', {
        entity: '__scene__',
      });
      return false;
    }

    if (failed.length === 0) {
      moveHistoryRef.current.clear();
    }

    const details: string[] = [];
    if (missingRequested.length > 0) details.push(`${missingRequested.length} requested items missing baseline`);
    if (failed.length > 0) details.push(`${failed.length} service failures`);
    const suffix = details.length > 0 ? ` (${details.join(', ')})` : '';
    moveTrace('reset.all.applied', {
      targetPoseCount: targetPoses.size,
      resetCount,
      missingRequested,
      failed,
      details,
    });
    emitNudgeStatus('success', `Reset ${resetCount} object(s) to session start${suffix}`, {
      entity: '__scene__',
    });
    return true;
  }, [applyAbsolutePose, emitNudgeStatus]);

  const saveScenePreset = useCallback((name: string): boolean => {
    const trimmedName = name.trim();
    if (!trimmedName) {
      emitNudgeStatus('error', 'Scene preset save failed: name is required', {
        entity: '__scene__',
      });
      return false;
    }

    if (entityPoseRef.current.size === 0) {
      emitNudgeStatus('error', 'Scene preset save failed: no poses available yet', {
        entity: '__scene__',
      });
      return false;
    }

    const poses: Record<string, ScenePresetPose> = {};
    for (const [poseName, pose] of entityPoseRef.current.entries()) {
      poses[poseName] = {
        position: { ...pose.position },
        orientation: { ...pose.orientation },
        id: pose.id,
      };
    }

    const world = sceneManagerRef.current?.transport?.getWorld?.();
    const snapshot: ScenePresetRecord = {
      name: trimmedName,
      capturedAt: Date.now(),
      world,
      poses,
    };

    sessionScenePresetsRef.current = {
      ...sessionScenePresetsRef.current,
      [trimmedName]: snapshot,
    };
    emitScenePresetList();
    emitNudgeStatus('success', `Saved scene preset \"${trimmedName}\" (${Object.keys(poses).length} poses)`, {
      entity: '__scene__',
    });
    return true;
  }, [emitNudgeStatus, emitScenePresetList]);

  const loadScenePreset = useCallback((name: string): boolean => {
    const trimmedName = name.trim();
    if (!trimmedName) {
      emitNudgeStatus('error', 'Scene preset load failed: name is required', {
        entity: '__scene__',
      });
      return false;
    }

    const store = sessionScenePresetsRef.current;
    const preset = store[trimmedName];
    if (!preset) {
      emitNudgeStatus('error', `Scene preset \"${trimmedName}\" was not found`, {
        entity: '__scene__',
      });
      return false;
    }

    const manager = sceneManagerRef.current;
    const transport = manager?.transport;
    if (!manager || !transport || !transport.requestService) {
      emitNudgeStatus('error', 'Scene preset load failed: scene transport is unavailable', {
        entity: '__scene__',
      });
      return false;
    }
    const world = transport.getWorld?.();
    if (!world) {
      emitNudgeStatus('error', 'Scene preset load failed: world is unavailable', {
        entity: '__scene__',
      });
      return false;
    }

    const poseTypes = getOrderedPoseMessageTypes(transport, world);
    const poseVectorTypes = getOrderedPoseVectorMessageTypes(transport, world);
    const poseServiceName = `/world/${world}/set_pose`;
    const poseVectorServiceName = `/world/${world}/set_pose_vector`;

    let totalRequests = 0;
    for (const [poseName, pose] of Object.entries(preset.poses ?? {})) {
      const nextPose: GazeboPose = {
        name: poseName,
        position: { ...pose.position },
        orientation: { ...pose.orientation },
        id: pose.id,
      };

      for (const msgType of poseTypes) {
        try {
          transport.requestService(poseServiceName, msgType, nextPose);
          totalRequests += 1;
        } catch {
          // Ignore incompatible protobuf msg types.
        }
      }

      for (const msgType of poseVectorTypes) {
        try {
          transport.requestService(poseVectorServiceName, msgType, { pose: [nextPose] });
          totalRequests += 1;
        } catch {
          // Ignore incompatible protobuf msg types.
        }
      }

      applyPoseToScene(manager, world, [poseName], nextPose);
      entityPoseRef.current.set(poseName, clonePose(nextPose));
    }

    if (totalRequests === 0) {
      emitNudgeStatus('error', `Scene preset \"${trimmedName}\" could not be applied (no compatible pose message type)`, {
        entity: '__scene__',
      });
      return false;
    }

    moveHistoryRef.current.clear();
    emitNudgeStatus('success', `Loaded scene preset \"${trimmedName}\"`, {
      entity: '__scene__',
    });
    return true;
  }, [emitNudgeStatus]);

  useEffect(() => {
    if (!directManipulationEnabled) {
      setSceneControlsEnabled(true);
      return;
    }
    setSceneControlsEnabled(!dragModeEnabled && !isDraggingAxisRef.current);
  }, [directManipulationEnabled, dragModeEnabled, setSceneControlsEnabled]);

  useEffect(() => {
    emitScenePresetList();
  }, [emitScenePresetList]);

  useEffect(() => {
    if (!directManipulationEnabled) return;
    const sceneElement = document.getElementById(SCENE_ELEMENT_ID);
    if (!sceneElement) return;
    const sceneCanvas =
      ((sceneManagerRef.current as any)?.scene?.getDomElement?.() as HTMLCanvasElement | undefined) ??
      sceneElement;
    const getSceneControls = () =>
      ((sceneManagerRef.current as any)?.scene?.controls as { enabled?: boolean } | undefined);

    const onPointerDown = (event: PointerEvent) => {
      if (event.button !== 0) return;

      if (!dragModeEnabled) return;
      event.preventDefault();
      event.stopPropagation();

      const controls = getSceneControls();
      if (typeof controls?.enabled === 'boolean') {
        controlsEnabledBeforeDragRef.current = controls.enabled;
        controls.enabled = false;
      } else {
        controlsEnabledBeforeDragRef.current = null;
      }

      const activeEntity = selectedEntityRef.current;
      const dragTargetName = activeEntity ? getGazeboEntityName(activeEntity) : '';
      if (!activeEntity || !dragTargetName) {
        setEntityControlStatus('Drag start failed: select a featured entity card first');
        setManipStatus('Drag start failed: select a featured entity card first');
        return;
      }

      dragStartPointRef.current = { x: event.clientX, y: event.clientY };
      dragDeltaMetersRef.current = 0;
      dragEntityRef.current = activeEntity;
      isDraggingAxisRef.current = true;
      setIsDraggingAxis(true);
      dragPointerIdRef.current = event.pointerId;
      try {
        sceneCanvas.setPointerCapture(event.pointerId);
      } catch {
        // Some renderers may not allow explicit pointer capture.
      }

      setEntityControlStatus(`Dragging ${dragTargetName} on ${dragAxis.toUpperCase()} axis`);
      setManipStatus(`Dragging ${dragTargetName} on ${dragAxis.toUpperCase()} axis`);
    };

    const finalizeDrag = (clientX?: number, clientY?: number, reason: 'up' | 'cancel' | 'lost_capture' | 'no_buttons' = 'up') => {
      const dragStart = dragStartPointRef.current;
      const activeDragEntity = dragEntityRef.current;

      if (isDraggingAxisRef.current && dragStart && activeDragEntity) {
        const hasCoords = typeof clientX === 'number' && typeof clientY === 'number';
        const deltaMeters = hasCoords
          ? (() => {
              const dx = clientX - dragStart.x;
              const dy = clientY - dragStart.y;
              const axisPixels = dragAxis === 'x' ? dx : -dy;
              return axisPixels * DRAG_METERS_PER_PIXEL;
            })()
          : dragDeltaMetersRef.current;
        dragDeltaMetersRef.current = deltaMeters;
        if (Math.abs(deltaMeters) >= DRAG_MIN_DELTA_METERS && reason !== 'cancel' && reason !== 'lost_capture') {
          const delta: PoseVector =
            dragAxis === 'x'
              ? { x: deltaMeters, y: 0, z: 0 }
              : dragAxis === 'y'
                ? { x: 0, y: deltaMeters, z: 0 }
                : { x: 0, y: 0, z: deltaMeters };
          const accepted = nudgeEntity(activeDragEntity, delta);
          setManipStatus(
            accepted
              ? `Move requested for ${getGazeboEntityName(activeDragEntity)}`
              : `Move rejected for ${getGazeboEntityName(activeDragEntity)}`,
          );
        } else {
          const canceledMessage =
            reason === 'cancel' || reason === 'lost_capture'
              ? `Drag canceled: ${reason === 'cancel' ? 'pointer canceled' : 'pointer capture lost'}`
              : `Drag canceled: movement below ${DRAG_MIN_DELTA_METERS}m (axis ${dragAxis.toUpperCase()})`;
          setEntityControlStatus(canceledMessage);
          setManipStatus(canceledMessage);
        }
      }

      const controls = getSceneControls();
      if (typeof controls?.enabled === 'boolean' && controlsEnabledBeforeDragRef.current !== null) {
        controls.enabled = controlsEnabledBeforeDragRef.current;
      }
      controlsEnabledBeforeDragRef.current = null;

      dragStartPointRef.current = null;
      dragDeltaMetersRef.current = 0;
      dragEntityRef.current = null;
      isDraggingAxisRef.current = false;
      const pointerId = dragPointerIdRef.current;
      if (pointerId !== null) {
        try {
          sceneCanvas.releasePointerCapture(pointerId);
        } catch {
          // Ignore if pointer capture was never set.
        }
      }
      dragPointerIdRef.current = null;
      setIsDraggingAxis(false);
    };

    const onPointerMove = (event: PointerEvent) => {
      const start = dragStartPointRef.current;
      if (!start || !isDraggingAxisRef.current) return;

      if (event.buttons === 0) {
        finalizeDrag(event.clientX, event.clientY, 'no_buttons');
        return;
      }

      const dx = event.clientX - start.x;
      const dy = event.clientY - start.y;
      const axisPixels = dragAxis === 'x' ? dx : -dy;
      const deltaMeters = axisPixels * DRAG_METERS_PER_PIXEL;
      dragDeltaMetersRef.current = deltaMeters;

      setEntityControlStatus(`Drag ${dragAxis.toUpperCase()}: ${deltaMeters.toFixed(2)}m (release to apply)`);
      setManipStatus(`Drag ${dragAxis.toUpperCase()}: ${deltaMeters.toFixed(2)}m (release to apply)`);
      event.preventDefault();
      event.stopPropagation();
    };

    const onPointerUp = (event: PointerEvent) => {
      finalizeDrag(event.clientX, event.clientY, 'up');
    };

    const onPointerCancel = () => {
      finalizeDrag(undefined, undefined, 'cancel');
    };

    const onLostPointerCapture = () => {
      finalizeDrag(undefined, undefined, 'lost_capture');
    };

    const onMouseUp = (event: MouseEvent) => {
      if (!isDraggingAxisRef.current) return;
      finalizeDrag(event.clientX, event.clientY, 'up');
    };

    const onWindowBlur = () => {
      if (!isDraggingAxisRef.current) return;
      finalizeDrag(undefined, undefined, 'cancel');
    };

    sceneCanvas.addEventListener('pointerdown', onPointerDown, true);
    window.addEventListener('pointermove', onPointerMove, true);
    window.addEventListener('pointerup', onPointerUp, true);
    window.addEventListener('pointercancel', onPointerCancel, true);
    sceneCanvas.addEventListener('lostpointercapture', onLostPointerCapture, true);
    window.addEventListener('mouseup', onMouseUp, true);
    window.addEventListener('blur', onWindowBlur, true);

    return () => {
      sceneCanvas.removeEventListener('pointerdown', onPointerDown, true);
      window.removeEventListener('pointermove', onPointerMove, true);
      window.removeEventListener('pointerup', onPointerUp, true);
      window.removeEventListener('pointercancel', onPointerCancel, true);
      sceneCanvas.removeEventListener('lostpointercapture', onLostPointerCapture, true);
      window.removeEventListener('mouseup', onMouseUp, true);
      window.removeEventListener('blur', onWindowBlur, true);
      dragStartPointRef.current = null;
      dragDeltaMetersRef.current = 0;
      dragEntityRef.current = null;
      isDraggingAxisRef.current = false;
      controlsEnabledBeforeDragRef.current = null;
      dragPointerIdRef.current = null;
      setIsDraggingAxis(false);
    };
  }, [
    directManipulationEnabled,
    dragAxis,
    dragModeEnabled,
    nudgeEntity,
    setManipStatus,
  ]);

  const handleEntityControlMessage = useCallback(
    (event: MessageEvent) => {
      const message = event.data as { type?: string; payload?: unknown };
      if (!message?.type) return;

      if (message.type === ENTITY_CONTROL_MESSAGES.SELECT) {
        const entity = toEntityCardData(message.payload);
        if (!entity) return;
        setSelectedEntity(entity);
        setSelectedEntitySource('panel');
        const gazeboEntity = getGazeboEntityName(entity);
        sceneManagerRef.current?.select?.(gazeboEntity);
        setEntityControlStatus(`Selected ${gazeboEntity}`);
        return;
      }

      if (message.type === ENTITY_CONTROL_MESSAGES.SCENE_SETUP_TRACE_CONFIG) {
        const payload = (message.payload ?? {}) as Partial<SceneSetupTraceConfigMessage['payload']>;
        const enabled = typeof payload.enabled === 'boolean' ? payload.enabled : false;
        const filter = typeof payload.filter === 'string' ? payload.filter.trim() : '';
        try {
          window.localStorage.setItem('tf.move.trace', enabled ? '1' : '0');
          if (filter.length > 0) {
            window.localStorage.setItem('tf.move.trace.filter', filter);
          } else {
            window.localStorage.removeItem('tf.move.trace.filter');
          }
        } catch {
          // Ignore localStorage access failures.
        }
        window.TENSORFLEET_MOVE_TRACE = enabled;
        setEntityControlStatus(
          enabled
            ? `Move trace enabled${filter ? ` (filter: ${filter})` : ''}`
            : 'Move trace disabled',
        );
        moveTrace('trace.config.updated', {
          enabled,
          filter: filter || undefined,
          source: 'scene_setup',
        });
        return;
      }

      if (message.type === ENTITY_CONTROL_MESSAGES.SCENE_PRESET_SAVE) {
        const payload = (message.payload ?? {}) as { name?: unknown };
        const name = typeof payload.name === 'string' ? payload.name : '';
        saveScenePreset(name);
        return;
      }

      if (message.type === ENTITY_CONTROL_MESSAGES.SCENE_PRESET_LIST_REQUEST) {
        emitScenePresetList();
        return;
      }

      if (message.type === ENTITY_CONTROL_MESSAGES.SCENE_PRESET_LOAD) {
        const payload = (message.payload ?? {}) as { name?: unknown };
        const name = typeof payload.name === 'string' ? payload.name : '';
        clearAllThrottledNudges('scene_preset_load');
        loadScenePreset(name);
        return;
      }

      const entityFromMessage = toEntityCardData(message.payload);
      const entity = entityFromMessage ?? selectedEntity;

      if (message.type === ENTITY_CONTROL_MESSAGES.UNDO_LAST_MOVE) {
        if (!entity) {
          setEntityControlStatus('Undo ignored: no selected entity');
          return;
        }
        undoLastMove(entity);
        return;
      }

      if (message.type === ENTITY_CONTROL_MESSAGES.RESET_ENTITY_POSE) {
        if (!entity) {
          setEntityControlStatus('Reset ignored: no selected entity');
          return;
        }
        clearAllThrottledNudges('reset_entity_pose');
        resetEntityPose(entity);
        return;
      }

      if (message.type === ENTITY_CONTROL_MESSAGES.RESET_ALL_POSES) {
        const entities = toEntityCardDataList(message.payload);
        clearAllThrottledNudges('reset_all_poses');
        resetAllPoses(entities);
        return;
      }

      if (message.type !== ENTITY_CONTROL_MESSAGES.NUDGE) return;

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
    [clearAllThrottledNudges, emitScenePresetList, loadScenePreset, nudgeEntity, resetAllPoses, resetEntityPose, saveScenePreset, selectedEntity, undoLastMove],
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
    const activeContactTopic = contactTopicRef.current;
    if (activeContactTopic && transport?.topicMap) {
      const topic = transport.topicMap.get(activeContactTopic);
      if (
        topic &&
        contactWrappedCbRef.current &&
        topic.cb === contactWrappedCbRef.current &&
        contactOriginalCbRef.current
      ) {
        topic.cb = contactOriginalCbRef.current as (msg: any) => void;
      }
    }
    dynamicPoseTopicRef.current = null;
    dynamicPoseOriginalCbRef.current = null;
    dynamicPoseWrappedCbRef.current = null;
    contactTopicRef.current = null;
    contactOriginalCbRef.current = null;
    contactWrappedCbRef.current = null;
    recentMoveRef.current = null;
    clearAllThrottledNudges('destroy_scene');
    lastNudgeDispatchAtRef.current.clear();
    moveHistoryRef.current.clear();
    entityPoseRef.current.clear();
    sessionInitialPoseRef.current.clear();
    sessionBaselineCaptureDeadlineRef.current = 0;
    entityVisualOffsetRef.current.clear();
    sessionScenePresetsRef.current = {};
    emitScenePresetList();
    clearHoverOutlineRefs(sceneManagerRef.current);
    if (sceneManagerRef.current) {
      sceneManagerRef.current.destroy();
      sceneManagerRef.current = null;
    }
    setIsConnected(false);
    setStatusTone('muted');
    setStatusText('');
  }, [clearAllThrottledNudges, clearHoverOutlineRefs, emitScenePresetList]);

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

    // Track exact outlined object refs; do not re-resolve on HOVER_END.
    for (const modelName of modelNames) {
      try {
        const hoverKey = `${entityName}::${modelName}`;
        const scene = manager.scene;

        if (!scene) {
          console.warn('Scene not available for outline operations');
          continue;
        }

        if (type === CARD_MESSAGES.HOVER_START) {
          const previousObjects = hoverOutlineRefs.current.get(hoverKey);
          if (previousObjects) {
            for (const prev of previousObjects) {
              if (scene.removeOutlineRoot) {
                scene.removeOutlineRoot(prev);
              } else if (scene.updateOutlineLayerMembership) {
                scene.updateOutlineLayerMembership(prev, false);
              }
            }
            hoverOutlineRefs.current.delete(hoverKey);
          }

          const modelObject = resolveSceneModelObject(manager, modelName);
          if (!modelObject) {
            console.warn(`Model object not found for entity: ${entityName}/${modelName}`);
            continue;
          }

          if (scene.addOutlineRoot) {
            scene.addOutlineRoot(modelObject);
            console.log(`Added ${entityName} to outline layer`);
          } else if (scene.updateOutlineLayerMembership) {
            scene.updateOutlineLayerMembership(modelObject, true);
            console.log(`Enabled outline for ${entityName}`);
          }
          hoverOutlineRefs.current.set(hoverKey, new Set([modelObject]));
        } else if (type === CARD_MESSAGES.HOVER_END) {
          const outlinedObjects = hoverOutlineRefs.current.get(hoverKey);
          if (!outlinedObjects || outlinedObjects.size === 0) {
            continue;
          }
          for (const obj of outlinedObjects) {
            if (scene.removeOutlineRoot) {
              scene.removeOutlineRoot(obj);
              console.log(`Removed ${entityName} from outline layer`);
            } else if (scene.updateOutlineLayerMembership) {
              scene.updateOutlineLayerMembership(obj, false);
              console.log(`Disabled outline for ${entityName}`);
            }
          }
          hoverOutlineRefs.current.delete(hoverKey);
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
      sessionBaselineCaptureDeadlineRef.current = Date.now() + SESSION_BASELINE_CAPTURE_WINDOW_MS;
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
    const deadline = sessionBaselineCaptureDeadlineRef.current;
    if (deadline <= 0) return;

    const interval = setInterval(() => {
      if (Date.now() > deadline) {
        clearInterval(interval);
        return;
      }
      const manager = sceneManagerRef.current;
      if (!manager) return;
      const models = manager.getModels?.() ?? [];
      for (const model of models) {
        const modelName = typeof model?.name === 'string' ? model.name : '';
        if (!modelName) continue;
        const modelObj = resolveSceneModelObject(manager, modelName);
        const pose = toPoseFromSceneObject(modelObj);
        if (!pose) continue;
        const poseName = pose.name || modelName;
        recordSessionInitialPose({ ...pose, name: poseName }, { force: true });
      }
    }, 250);

    return () => clearInterval(interval);
  }, [isConnected, recordSessionInitialPose]);

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

        for (const poseEntry of msg.pose ?? []) {
          if (!poseEntry.name) continue;
          const position = toPoseVector(poseEntry.position);
          const orientation = toPoseQuaternion(poseEntry.orientation);
          if (!position || !orientation) continue;
          const id = toOptionalNumber(poseEntry.id);
          const poseSnapshot = {
            name: poseEntry.name,
            position,
            orientation,
            id,
          };
          entityPoseRef.current.set(poseEntry.name, poseSnapshot);
          recordSessionInitialPose(poseSnapshot);
        }
      };

      topic.cb = wrappedCb;
      dynamicPoseTopicRef.current = topicName;
      dynamicPoseOriginalCbRef.current = originalCb;
      dynamicPoseWrappedCbRef.current = wrappedCb;
    }, 250);

    return () => clearInterval(interval);
  }, [isConnected, recordSessionInitialPose]);

  useEffect(() => {
    if (!isConnected) return;

    const interval = setInterval(() => {
      const transport = sceneManagerRef.current?.transport;
      if (!transport) return;
      const world = transport.getWorld?.();
      if (!world) return;
      const topicName = resolveContactTopicName(transport, world);
      if (!topicName) return;

      const transportWithMap = transport as unknown as {
        topicMap?: Map<string, { cb: (msg: unknown) => void }>;
      };
      const topic = transportWithMap.topicMap?.get(topicName);
      if (!topic || typeof topic.cb !== 'function') return;
      if (contactTopicRef.current === topicName && contactWrappedCbRef.current === topic.cb) return;

      if (contactTopicRef.current && contactOriginalCbRef.current && contactWrappedCbRef.current) {
        const prevTopic = transportWithMap.topicMap?.get(contactTopicRef.current);
        if (prevTopic && prevTopic.cb === contactWrappedCbRef.current) {
          prevTopic.cb = contactOriginalCbRef.current as (msg: unknown) => void;
        }
      }

      const originalCb = topic.cb;
      const wrappedCb = (msg: unknown) => {
        originalCb(msg);

        const recent = recentMoveRef.current;
        if (!recent) return;
        const now = Date.now();
        if (now - recent.atMs > CONTACT_WARN_WINDOW_MS) return;
        if (recent.warnedAtMs && now - recent.warnedAtMs < CONTACT_WARN_COOLDOWN_MS) return;

        let serialized = '';
        try {
          serialized = JSON.stringify(msg)?.toLowerCase() ?? '';
        } catch {
          return;
        }
        if (!serialized) return;

        const matchedAliases = recent.aliases.filter((alias) => serialized.includes(alias.toLowerCase()));
        if (matchedAliases.length === 0) return;

        recent.warnedAtMs = now;
        emitNudgeStatus(
          'warning',
          `Contact detected after move for ${recent.entity}`,
          { entity: recent.entity },
        );
        moveTrace('move.contact.warning', {
          world,
          topicName,
          entity: recent.entity,
          matchedAliases: matchedAliases.slice(0, 4),
          observedAtMs: now - recent.atMs,
        });
      };

      topic.cb = wrappedCb;
      contactTopicRef.current = topicName;
      contactOriginalCbRef.current = originalCb;
      contactWrappedCbRef.current = wrappedCb;
    }, 500);

    return () => clearInterval(interval);
  }, [emitNudgeStatus, isConnected]);

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
                  {selectedEntity && (
                    <> (<code>{selectedEntitySource}</code>)</>
                  )}
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

      <div className={`gzweb-manip-overlay ${dragModeEnabled ? 'armed' : ''}`}>
        <div className="gzweb-manip-header">
          <span>Direct Manipulation</span>
          <label className="gzweb-manip-toggle">
            <input
              type="checkbox"
              checked={directManipulationEnabled}
              onChange={(event) => {
                setDirectManipulationEnabled(event.target.checked);
                if (!event.target.checked) {
                  setDragModeEnabled(false);
                  isDraggingAxisRef.current = false;
                  setIsDraggingAxis(false);
                  setManipStatus('Direct manipulation disabled');
                  dragStartPointRef.current = null;
                  dragEntityRef.current = null;
                  dragDeltaMetersRef.current = 0;
                  dragPointerIdRef.current = null;
                  const controls = ((sceneManagerRef.current as any)?.scene?.controls as { enabled?: boolean } | undefined);
                  if (typeof controls?.enabled === 'boolean' && controlsEnabledBeforeDragRef.current !== null) {
                    controls.enabled = controlsEnabledBeforeDragRef.current;
                  }
                  controlsEnabledBeforeDragRef.current = null;
                }
              }}
            />
            <span>{directManipulationEnabled ? 'On' : 'Off'}</span>
          </label>
        </div>
        <div className="gzweb-manip-controls">
          <button
            type="button"
            className={dragModeEnabled ? 'active' : ''}
            disabled={!directManipulationEnabled}
            onClick={() => {
              setDragModeEnabled((value) => {
                const next = !value;
                if (!next) {
                  isDraggingAxisRef.current = false;
                  setIsDraggingAxis(false);
                  setManipStatus('Drag mode disarmed');
                  dragStartPointRef.current = null;
                  dragEntityRef.current = null;
                  dragDeltaMetersRef.current = 0;
                  dragPointerIdRef.current = null;
                  const controls = ((sceneManagerRef.current as any)?.scene?.controls as { enabled?: boolean } | undefined);
                  if (typeof controls?.enabled === 'boolean' && controlsEnabledBeforeDragRef.current !== null) {
                    controls.enabled = controlsEnabledBeforeDragRef.current;
                  }
                  controlsEnabledBeforeDragRef.current = null;
                }
                return next;
              });
            }}
          >
            {dragModeEnabled ? (isDraggingAxis ? 'Dragging…' : 'Drag armed') : 'Click select'}
          </button>
          <div className="gzweb-manip-axis">
            {(['x', 'y', 'z'] as const).map((axis) => (
              <button
                key={axis}
                type="button"
                className={dragAxis === axis ? 'active' : ''}
                disabled={!directManipulationEnabled}
                onClick={() => setDragAxis(axis)}
              >
                {axis.toUpperCase()}
              </button>
            ))}
          </div>
        </div>
        <div className="gzweb-manip-hint">
          {dragModeEnabled
            ? 'Click object, drag mouse, release to apply axis move. Right = +X, up = +Y/+Z.'
            : 'Click an object in the viewport to select it.'}
        </div>
        <div className="gzweb-manip-status">
          Target: <code>{selectedEntity ? getGazeboEntityName(selectedEntity) : 'none'}</code>
          {selectedEntity && (
            <> (<code>{selectedEntitySource}</code>)</>
          )}
        </div>
        <div className="gzweb-manip-status">
          Drag status: <span>{manipStatusText}</span>
        </div>
        {entityControlStatus && (
          <div className="gzweb-manip-status">
            Status: <span>{entityControlStatus}</span>
          </div>
        )}
      </div>

      <div id={SCENE_ELEMENT_ID} className="gzweb-scene" />
    </div>
  );
};
