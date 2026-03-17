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
  isFinitePoseVector,
  poseVectorMagnitude,
  roundPoseVector,
  resolvePoseEntry,
  resolveVisualOffset,
  toPoseQuaternion,
  toPoseVector,
  PoseQuaternion,
  PoseVector,
  unique,
} from './moveControl';
import {
  buildManipulationPoseBinding,
  parseManipulationDispatchEvent,
  parseManipulationServiceReplyEvent,
  parseManipulationStateEvent,
  pollForPoseConfirmation,
  resolveManipulationObservation,
} from './manipulationController';
import { getGazeboEntityName, getPoseEditAccess, getRuntimePoseEntityName } from './posePolicy';
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
  clearSelection?: () => void;
  getSelectedEntityName?: () => string | null;
  setManipulationMode?: (mode: string) => void;
  setManipulationTargetNames?: (entityNames: string[]) => void;
  getManipulationMode?: () => string;
  setControlsEnabled?: (enabled: boolean) => void;
  getControlsEnabled?: () => boolean;
  getDomElement?: () => HTMLCanvasElement | null;
  intersectPointerOnHorizontalPlane?: (clientX: number, clientY: number, planeZ: number) => THREE.Vector3 | null;
  intersectPointerOnSceneSurface?: (
    clientX: number,
    clientY: number,
    options?: { ignoreNames?: string[]; ignoreNameSubstrings?: string[] },
  ) => { point: THREE.Vector3; surfaceName: string } | null;
  previewPose?: (
    world: string,
    poseNames: string[],
    pose: { position: PoseVector; orientation: PoseQuaternion },
  ) => boolean;
  onSceneEvent?: (eventName: string, listener: (...args: any[]) => void) => void;
  offSceneEvent?: (eventName: string, listener: (...args: any[]) => void) => void;
  getModels?: () => Array<{ name?: string }>;
  transport?: SceneManagerTransport;
  scene?: any; // The Scene instance
  getModelByName?: (name: string) => any; // Method to get model by name
};

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
    TENSORFLEET_MOVE_TRACE_LINES?: string[];
    TENSORFLEET_MOVE_TRACE_DUMP?: () => string;
    TENSORFLEET_MOVE_TRACE_CLEAR?: () => void;
  }
}

const SCENE_ELEMENT_ID = 'gz-scene';
const SESSION_BASELINE_CAPTURE_WINDOW_MS = 8000;
const CONTACT_WARN_WINDOW_MS = 3000;
const CONTACT_WARN_COOLDOWN_MS = 1500;
const DEFAULT_NUDGE_THROTTLE_MS = 120;
const MIN_NUDGE_THROTTLE_MS = 16;
const MAX_NUDGE_THROTTLE_MS = 1000;
const DEFAULT_MAX_NUDGE_DELTA_METERS = 3;
const MIN_MOVE_EPSILON_METERS = 0.0001;
const DRAG_CONFIRM_TIMEOUT_MS = 3200;
const DRAG_CONFIRM_TOLERANCE_METERS = 0.02;
const MOVE_TRACE_BUFFER_MAX_LINES = 2000;
const SCENE_EVENT_MANIPULATION_DISPATCH = 'manipulation_dispatch';
const SCENE_EVENT_MANIPULATION_SERVICE_REPLY = 'manipulation_service_reply';
const SCENE_EVENT_MANIPULATION_STATE = 'manipulation_state';

type PendingThrottledNudge = {
  entity: EntityCardData;
  delta: PoseVector;
  requestId?: string;
  timerId?: ReturnType<typeof setTimeout>;
  queuedAtMs: number;
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

type SelectionSource = 'panel' | 'viewport' | 'none';

const toFiniteNumber = (value: unknown): number | null => {
  if (typeof value === 'number' && Number.isFinite(value)) return value;
  if (typeof value !== 'string') return null;
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
};

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

const getConfiguredNudgeThrottleMs = (): number => {
  if (typeof window === 'undefined') return DEFAULT_NUDGE_THROTTLE_MS;
  let configured: number | null = null;
  try {
    const params = new URLSearchParams(window.location.search);
    configured = toFiniteNumber(
      params.get('moveThrottleMs') ??
      params.get('move_throttle_ms'),
    );
  } catch {
    // Ignore malformed query string parsing.
  }
  if (configured === null) {
    try {
      configured = toFiniteNumber(window.localStorage.getItem('tf.move.throttle.ms'));
    } catch {
      // Ignore localStorage access failures.
    }
  }
  if (configured === null) return DEFAULT_NUDGE_THROTTLE_MS;
  const rounded = Math.round(configured);
  return Math.min(MAX_NUDGE_THROTTLE_MS, Math.max(MIN_NUDGE_THROTTLE_MS, rounded));
};

const getConfiguredMaxNudgeDeltaMeters = (): number => {
  if (typeof window === 'undefined') return DEFAULT_MAX_NUDGE_DELTA_METERS;
  let configured: number | null = null;
  try {
    const params = new URLSearchParams(window.location.search);
    configured = toFiniteNumber(
      params.get('maxNudgeDeltaMeters') ??
      params.get('max_nudge_delta_meters'),
    );
  } catch {
    // Ignore malformed query string parsing.
  }
  if (configured === null) {
    try {
      configured = toFiniteNumber(window.localStorage.getItem('tf.move.max_delta_m'));
    } catch {
      // Ignore localStorage access failures.
    }
  }
  if (configured === null || configured <= 0) return DEFAULT_MAX_NUDGE_DELTA_METERS;
  return configured;
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

const toTraceValueText = (value: unknown): string => {
  if (value === undefined) return 'undefined';
  if (value === null) return 'null';
  if (typeof value === 'number' || typeof value === 'boolean') return String(value);
  if (typeof value === 'string') {
    return value.includes(' ') ? JSON.stringify(value) : value;
  }
  try {
    const serialized = JSON.stringify(value);
    if (!serialized) return String(value);
    return serialized.length > 220 ? `${serialized.slice(0, 217)}...` : serialized;
  } catch {
    return String(value);
  }
};

const toMoveTraceLine = (phase: string, ts: string, payload: Record<string, unknown>): string => {
  const preferredKeys = [
    'requestId',
    'entity',
    'world',
    'serviceName',
    'msgType',
    'attempt',
    'maxAttempts',
    'state',
    'message',
    'note',
  ];
  const payloadKeys = Object.keys(payload);
  const orderedKeys = [
    ...preferredKeys.filter((key) => payloadKeys.includes(key)),
    ...payloadKeys.filter((key) => !preferredKeys.includes(key)).sort(),
  ];
  const kv = orderedKeys.map((key) => `${key}=${toTraceValueText(payload[key])}`).join(' ');
  return kv
    ? `[MoveTrace] ts=${ts} phase=${phase} ${kv}`
    : `[MoveTrace] ts=${ts} phase=${phase}`;
};

const pushMoveTraceLine = (line: string) => {
  if (typeof window === 'undefined') return;
  const lines = window.TENSORFLEET_MOVE_TRACE_LINES ?? [];
  lines.push(line);
  if (lines.length > MOVE_TRACE_BUFFER_MAX_LINES) {
    lines.splice(0, lines.length - MOVE_TRACE_BUFFER_MAX_LINES);
  }
  window.TENSORFLEET_MOVE_TRACE_LINES = lines;
  if (!window.TENSORFLEET_MOVE_TRACE_DUMP) {
    window.TENSORFLEET_MOVE_TRACE_DUMP = () => (window.TENSORFLEET_MOVE_TRACE_LINES ?? []).join('\n');
  }
  if (!window.TENSORFLEET_MOVE_TRACE_CLEAR) {
    window.TENSORFLEET_MOVE_TRACE_CLEAR = () => {
      window.TENSORFLEET_MOVE_TRACE_LINES = [];
    };
  }
};

const moveTrace = (phase: string, payload: Record<string, unknown>) => {
  if (!isMoveTraceEnabled()) return;
  const filter = getMoveTraceFilter();
  if (filter && !matchesMoveTraceFilter(phase, payload, filter)) return;
  const ts = new Date().toISOString();
  const line = toMoveTraceLine(phase, ts, {
    filter: filter || undefined,
    ...payload,
  });
  console.info(line);
  pushMoveTraceLine(line);
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
  observedAtMs: pose.observedAtMs,
});

const captureSceneObjectPose = (
  object: any,
  poseName: string,
): GazeboPose | null => {
  if (!object || !poseName || typeof object.updateMatrixWorld !== 'function') {
    return null;
  }
  const position = new THREE.Vector3();
  const orientation = new THREE.Quaternion();
  object.updateMatrixWorld(true);
  if (typeof object.getWorldPosition !== 'function' || typeof object.getWorldQuaternion !== 'function') {
    return null;
  }
  object.getWorldPosition(position);
  object.getWorldQuaternion(orientation);
  return {
    name: poseName,
    position: {
      x: position.x,
      y: position.y,
      z: position.z,
    },
    orientation: {
      x: orientation.x,
      y: orientation.y,
      z: orientation.z,
      w: orientation.w,
    },
    observedAtMs: 0,
  };
};

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
  const [entityControlStatus, setEntityControlStatus] = useState('');
  const [directManipulationEnabled, setDirectManipulationEnabled] = useState(true);
  const [dragModeEnabled, setDragModeEnabled] = useState(false);
  const [isDraggingDirect, setIsDraggingDirect] = useState(false);
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

  const selectedEntityPoseAccess = useMemo(
    () => getPoseEditAccess(selectedEntity),
    [selectedEntity],
  );
  const selectedEntityAllowsDirectManipulation = selectedEntityPoseAccess.enabled;
  const selectedManipulationTargetNames = useMemo(() => {
    if (!selectedEntity || !selectedEntityAllowsDirectManipulation) {
      return [] as string[];
    }
    return unique([
      getRuntimePoseEntityName(selectedEntity),
      getGazeboEntityName(selectedEntity),
    ]).filter((name) => name.length > 0);
  }, [selectedEntity, selectedEntityAllowsDirectManipulation]);

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
      throttleMs: getConfiguredNudgeThrottleMs(),
      maxNudgeDeltaMeters: getConfiguredMaxNudgeDeltaMeters(),
    });
  }, []);
  const hoverOutlineRefs = useRef<Map<string, Set<any>>>(new Map());
  const isDraggingDirectRef = useRef(false);

  const setSceneControlsEnabled = useCallback((enabled: boolean) => {
    sceneManagerRef.current?.setControlsEnabled?.(enabled);
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

  const previewPoseInScene = useCallback((
    manager: SceneManagerInstance,
    world: string,
    poseNames: string[],
    pose: GazeboPose,
  ): boolean => {
    return manager.previewPose?.(world, poseNames, pose) ?? false;
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

  const seedPoseCacheFromScene = useCallback((manager?: SceneManagerInstance | null) => {
    const activeManager = manager ?? sceneManagerRef.current;
    if (!activeManager?.getModels) return;

    for (const model of activeManager.getModels() ?? []) {
      const poseName = typeof model?.name === 'string' ? model.name : '';
      if (!poseName || entityPoseRef.current.has(poseName)) continue;
      const object = resolveSceneModelObject(activeManager, poseName);
      const pose = captureSceneObjectPose(object, poseName);
      if (!pose) continue;
      entityPoseRef.current.set(poseName, pose);
      recordSessionInitialPose(pose, { force: true });
    }
  }, [recordSessionInitialPose]);

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

  const requestSetPose = useCallback((
    transport: SceneManagerTransport,
    world: string,
    msgType: string,
    pose: GazeboPose,
    traceMeta: {
      requestId?: string;
      entity?: string;
      phasePrefix: string;
    },
  ): { ok: true; serviceName: string } | { ok: false; serviceName: string; error: string } => {
    const serviceName = `/world/${world}/set_pose`;
    try {
      moveTrace(`${traceMeta.phasePrefix}.set_pose`, {
        requestId: traceMeta.requestId,
        attempt: 1,
        world,
        serviceName,
        msgType,
        entity: traceMeta.entity,
        pose: {
          name: pose.name,
          id: pose.id,
          position: toTracePosition(pose.position),
          orientation: toTraceOrientation(pose.orientation),
        },
      });
      transport.requestService!(serviceName, msgType, pose);
      return { ok: true, serviceName };
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      if (!message.includes('no such type')) {
        console.warn(`set_pose request failed for ${msgType}`, error);
      }
      moveTrace(`${traceMeta.phasePrefix}.set_pose.error`, {
        requestId: traceMeta.requestId,
        attempt: 1,
        world,
        serviceName,
        msgType,
        poseName: pose.name,
        error: message,
      });
      return { ok: false, serviceName, error: message };
    }
  }, []);

  const dispatchNudgeEntity = useCallback(
    (
      entity: EntityCardData,
      delta: PoseVector,
      requestIdFromMessage?: string,
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
      if (!isFinitePoseVector(delta)) {
        emitNudgeStatus('error', 'Move ignored: delta contains non-finite values', {
          requestId,
          entity: mappedEntityName,
        });
        return false;
      }
      const isSimpleRobot = entity.type.toLowerCase() === 'simple_robot';
      if (isSimpleRobot && Math.abs(delta.z) > MIN_MOVE_EPSILON_METERS) {
        emitNudgeStatus('warning', 'Move ignored: Simple Bot supports XY moves only in Scene Setup', {
          requestId,
          entity: mappedEntityName,
        });
        moveTrace('move.dispatch.rejected_z_for_simple_robot', {
          requestId,
          entity: mappedEntityName,
          delta: toTracePosition(delta),
        });
        return false;
      }
      const deltaMagnitude = poseVectorMagnitude(delta);
      if (deltaMagnitude <= MIN_MOVE_EPSILON_METERS) {
        moveTrace('move.dispatch.ignored_zero_delta', {
          requestId,
          entity: mappedEntityName,
          delta: toTracePosition(delta),
          epsilonMeters: MIN_MOVE_EPSILON_METERS,
        });
        return true;
      }
      const maxNudgeDeltaMeters = getConfiguredMaxNudgeDeltaMeters();
      if (deltaMagnitude > maxNudgeDeltaMeters) {
        emitNudgeStatus(
          'error',
          `Move ignored: delta ${deltaMagnitude.toFixed(3)}m exceeds max ${maxNudgeDeltaMeters.toFixed(3)}m`,
          { requestId, entity: mappedEntityName },
        );
        moveTrace('move.dispatch.rejected_delta_too_large', {
          requestId,
          entity: mappedEntityName,
          deltaMagnitudeMeters: Number(deltaMagnitude.toFixed(4)),
          maxNudgeDeltaMeters,
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
      const { poseName, pose: currentPose } = resolvedPoseEntry;
      const canonicalNextPose: GazeboPose = {
        name: poseName,
        position: roundPoseVector(addPoseVector(currentPose.position, delta)),
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
        poseServiceName: `/world/${world}/set_pose`,
        msgTypeCandidates,
        selectedMsgType: msgType,
      });

      const response = requestSetPose(transport, world, msgType, canonicalNextPose, {
        requestId,
        entity: moveEntity,
        phasePrefix: 'move.dispatch',
      });
      if (!response.ok) {
        emitNudgeStatus('error', `Move failed: set_pose request error (${response.error})`, {
          requestId,
          entity: mappedEntityName,
          attempt: 1,
          maxAttempts: 1,
        });
        return false;
      }

      previewPoseInScene(
        manager,
        world,
        unique([
          poseName,
          worldScopedPoseName,
          mappedEntityName,
          resolvedEntityName,
        ]),
        canonicalNextPose,
      );
      if (Math.abs(delta.z) <= MIN_MOVE_EPSILON_METERS) {
        const optimisticPoseNames = unique([
          poseName,
          worldScopedPoseName,
          unscopedPoseName,
          mappedEntityName,
          resolvedEntityName,
        ]);
        for (const optimisticPoseName of optimisticPoseNames) {
          entityPoseRef.current.set(optimisticPoseName, {
            ...clonePose(canonicalNextPose),
            name: optimisticPoseName,
          });
        }
      }
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
      moveTrace('move.dispatch.accepted', {
        requestId,
        world,
        serviceName: response.serviceName,
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
      const expectedPosition = { ...canonicalNextPose.position };
      const observationNames = unique([
        poseName,
        worldScopedPoseName,
        unscopedPoseName,
        mappedEntityName,
        resolvedEntityName,
        moveEntity,
      ]);
      setTimeout(() => {
        let observedName: string | undefined;
        let observedPose: GazeboPose | undefined;
        for (const candidateName of observationNames) {
          const candidatePose = entityPoseRef.current.get(candidateName);
          if (!candidatePose) continue;
          observedName = candidateName;
          observedPose = candidatePose;
          break;
        }
        if (!observedPose) {
          for (const [entryName, entryPose] of entityPoseRef.current.entries()) {
            if (!observationNames.some((name) => entryName.includes(name))) continue;
            observedName = entryName;
            observedPose = entryPose;
            break;
          }
        }
        if (!observedPose) {
          moveTrace('move.observe.after_dispatch', {
            requestId,
            world,
            entity: moveEntity,
            expectedPosition: toTracePosition(expectedPosition),
            note: 'No dynamic pose snapshot found for observation aliases',
            observationNames,
          });
          return;
        }
        const dx = observedPose.position.x - expectedPosition.x;
        const dy = observedPose.position.y - expectedPosition.y;
        const dz = observedPose.position.z - expectedPosition.z;
        moveTrace('move.observe.after_dispatch', {
          requestId,
          world,
          entity: moveEntity,
          expectedPosition: toTracePosition(expectedPosition),
          observedName,
          observedPosition: toTracePosition(observedPose.position),
          observedPoseId: observedPose.id,
          distanceToExpectedMeters: Number(Math.sqrt(dx * dx + dy * dy + dz * dz).toFixed(4)),
        });
      }, 250);
      return true;
    },
    [emitNudgeStatus, previewPoseInScene, requestSetPose],
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
    const pendingMagnitude = poseVectorMagnitude(pending.delta);
    if (pendingMagnitude <= MIN_MOVE_EPSILON_METERS) {
      moveTrace('move.throttle.drop_zero_delta', {
        entity: entityKey,
        reason,
        deltaMagnitudeMeters: Number(pendingMagnitude.toFixed(6)),
      });
      return true;
    }
    const accepted = dispatchNudgeEntity(
      pending.entity,
      pending.delta,
      pending.requestId,
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
    ): boolean => {
      const entityKey = getGazeboEntityName(entity);
      if (!isFinitePoseVector(delta)) {
        return dispatchNudgeEntity(entity, delta, requestIdFromMessage);
      }
      if (!entityKey) {
        return dispatchNudgeEntity(entity, delta, requestIdFromMessage);
      }
      if (poseVectorMagnitude(delta) <= MIN_MOVE_EPSILON_METERS) {
        moveTrace('move.throttle.ignored_zero_delta', {
          entity: entityKey,
          delta: toTracePosition(delta),
          epsilonMeters: MIN_MOVE_EPSILON_METERS,
        });
        return true;
      }

      const now = Date.now();
      const throttleMs = getConfiguredNudgeThrottleMs();
      const lastDispatchAt = lastNudgeDispatchAtRef.current.get(entityKey) ?? 0;
      const elapsedMs = now - lastDispatchAt;
      if (elapsedMs >= throttleMs) {
        const accepted = dispatchNudgeEntity(entity, delta, requestIdFromMessage);
        if (accepted) {
          lastNudgeDispatchAtRef.current.set(entityKey, Date.now());
        }
        return accepted;
      }

      const remainingMs = Math.max(0, throttleMs - elapsedMs);
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
        throttleMs,
        remainingMs: Math.round(remainingMs),
        queuedDelta: toTracePosition(pending.delta),
      });
      return true;
    },
    [dispatchNudgeEntity, flushThrottledNudge],
  );

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
    const msgType = poseTypes[0];
    const poseServiceName = `/world/${world}/set_pose`;
    const requestNames = unique([
      pose.name,
      ...poseNames,
      ...poseNames.map((name) => (name.includes('::') ? undefined : `${world}::${name}`)),
    ]);
    if (!msgType) return false;
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
      poseTypes,
      selectedMsgType: msgType,
    });

    const nextPose: GazeboPose = {
      name: pose.name,
      position: roundPoseVector(pose.position),
      orientation: { ...pose.orientation },
      id: pose.id,
    };
    const response = requestSetPose(transport, world, msgType, nextPose, {
      phasePrefix: 'reset.apply',
    });
    const totalRequests = response.ok ? 1 : 0;

    const aliases = buildPoseNameAliases(world, requestNames);
    clearVisualOffsets(aliases);
    previewPoseInScene(manager, world, aliases, nextPose);
    for (const alias of aliases) {
      entityPoseRef.current.set(alias, {
        ...clonePose(nextPose),
        name: alias,
      });
    }
    moveTrace('reset.apply.sent', {
      world,
      poseName: pose.name,
      totalRequests,
      aliases,
    });
    return totalRequests > 0;
  }, [clearVisualOffsets, previewPoseInScene, requestSetPose]);

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

    if (providedEntities.length === 0 && targetPoses.size === 0) {
      for (const [poseName, pose] of sessionInitialPoseRef.current.entries()) {
        addTargetPose(pose, poseName);
      }
    }

    if (targetPoses.size === 0) {
      const missingSummary = missingRequested.length > 0
        ? ` (missing baseline: ${missingRequested.slice(0, 5).join(', ')}${missingRequested.length > 5 ? ', ...' : ''})`
        : '';
      const message = providedEntities.length > 0
        ? `Reset all failed: no baseline pose found for requested entities${missingSummary}`
        : 'Reset all failed: no session baseline poses are available yet';
      emitNudgeStatus('error', message, {
        entity: '__scene__',
      });
      moveTrace('reset.all.failed_no_targets', {
        requestedCount: providedEntities.length,
        missingRequested,
        baselinePoseCount: sessionInitialPoseRef.current.size,
        note: `No reset targets resolved${missingSummary}`,
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
    const msgType = poseTypes[0];
    if (!msgType) {
      emitNudgeStatus('error', `Scene preset \"${trimmedName}\" could not be applied (no compatible pose message type)`, {
        entity: '__scene__',
      });
      return false;
    }

    let totalRequests = 0;
    for (const [poseName, pose] of Object.entries(preset.poses ?? {})) {
      const nextPose: GazeboPose = {
        name: poseName,
        position: roundPoseVector(pose.position),
        orientation: { ...pose.orientation },
        id: pose.id,
      };

      const response = requestSetPose(transport, world, msgType, nextPose, {
        phasePrefix: 'scene_preset.load',
      });
      if (response.ok) {
        totalRequests += 1;
      }

      previewPoseInScene(manager, world, [poseName], nextPose);
      entityPoseRef.current.set(poseName, clonePose(nextPose));
    }

    if (totalRequests === 0) {
      emitNudgeStatus('error', `Scene preset \"${trimmedName}\" could not be applied (no compatible pose message type)`, {
        entity: '__scene__',
      });
      return false;
    }

    emitNudgeStatus('success', `Loaded scene preset \"${trimmedName}\"`, {
      entity: '__scene__',
    });
    return true;
  }, [emitNudgeStatus, previewPoseInScene, requestSetPose]);

  useEffect(() => {
    emitScenePresetList();
  }, [emitScenePresetList]);

  useEffect(() => {
    if (!directManipulationEnabled) {
      setSceneControlsEnabled(true);
      return;
    }
    setSceneControlsEnabled(!isDraggingDirectRef.current);
  }, [directManipulationEnabled, dragModeEnabled, setSceneControlsEnabled]);

  useEffect(() => {
    const manager = sceneManagerRef.current;
    if (!manager?.setManipulationMode) return;
    manager.setManipulationTargetNames?.(selectedManipulationTargetNames);
    const nextMode =
      directManipulationEnabled && dragModeEnabled && selectedEntityAllowsDirectManipulation
        ? 'translate'
        : 'view';
    manager.setManipulationMode(nextMode);
  }, [
    directManipulationEnabled,
    dragModeEnabled,
    selectedEntityAllowsDirectManipulation,
    selectedManipulationTargetNames,
  ]);

  useEffect(() => {
    if (!isConnected || !directManipulationEnabled) return;
    const manager = sceneManagerRef.current;
    if (!manager?.onSceneEvent || !manager?.offSceneEvent) return;
    let disposed = false;
    type PendingDragConfirmation = {
      requestId: string;
      entity: string;
      cancel: () => void;
    };

    const pendingConfirmCancels = new Set<() => void>();
    const pendingConfirmByEntity = new Map<string, PendingDragConfirmation>();
    const pendingConfirmByRequestId = new Map<string, PendingDragConfirmation>();
    const clearPendingConfirmation = (pending: PendingDragConfirmation | undefined) => {
      if (!pending) return;
      pending.cancel();
      pendingConfirmCancels.delete(pending.cancel);
      if (pendingConfirmByEntity.get(pending.entity)?.requestId === pending.requestId) {
        pendingConfirmByEntity.delete(pending.entity);
      }
      if (pendingConfirmByRequestId.get(pending.requestId)?.entity === pending.entity) {
        pendingConfirmByRequestId.delete(pending.requestId);
      }
    };

    const handleManipulationState = (payload: unknown) => {
      const event = parseManipulationStateEvent(payload);
      const dragging = event.dragging;
      isDraggingDirectRef.current = dragging;
      setIsDraggingDirect(dragging);
      const entityName = event.entity || 'selected entity';
      if (dragging) {
        setManipStatus(`Dragging ${entityName}`);
        setEntityControlStatus(`Dragging ${entityName} (release to apply exact pose)`);
      } else {
        setManipStatus(dragModeEnabled ? 'Gizmo ready' : 'Ready');
      }
    };

    const handleManipulationDispatch = (payload: unknown) => {
      if (!dragModeEnabled) return;
      const dispatchEvent = parseManipulationDispatchEvent(payload);
      if (!dispatchEvent) return;
      const {
        name: requestedName,
        position,
        orientation,
        world,
        serviceName,
        msgType,
        ok,
        error,
        requestId: dispatchRequestId,
      } = dispatchEvent;

      const binding = buildManipulationPoseBinding({
        poses: entityPoseRef.current,
        world,
        requestedName,
      });
      const targetPose: GazeboPose = {
        name: binding.poseName,
        id: binding.poseId,
        position: roundPoseVector(position),
        orientation,
      };
      const requestId =
        dispatchRequestId ??
        `drag-${Date.now().toString(36)}-${(++moveRequestCounterRef.current).toString(36)}`;
      const existingPending = pendingConfirmByEntity.get(requestedName);
      if (existingPending) {
        clearPendingConfirmation(existingPending);
        moveTrace('drag.confirm.superseded', {
          requestId,
          world,
          entity: requestedName,
          note: 'Cancelled older confirmation poll for superseded drag request',
        });
      }
      emitNudgeStatus('pending', `Applying drag pose for ${requestedName}...`, {
        requestId,
        entity: requestedName,
        attempt: 1,
        maxAttempts: 1,
      });
      if (!ok) {
        moveTrace('drag.dispatch.error', {
          requestId,
          world,
          serviceName,
          msgType,
          entity: requestedName,
          error,
          pose: {
            name: targetPose.name,
            id: targetPose.id,
            position: toTracePosition(targetPose.position),
            orientation: toTraceOrientation(targetPose.orientation),
          },
        });
        emitNudgeStatus('error', `Drag move failed: ${error ?? 'set_pose request error'}`, {
          requestId,
          entity: requestedName,
          attempt: 1,
          maxAttempts: 1,
        });
        setManipStatus(`Drag move failed for ${requestedName}`);
        return;
      }

      moveTrace('drag.dispatch.set_pose', {
        requestId,
        attempt: 1,
        world,
        serviceName,
        msgType,
        entity: requestedName,
        pose: {
          name: targetPose.name,
          id: targetPose.id,
          position: toTracePosition(targetPose.position),
          orientation: toTraceOrientation(targetPose.orientation),
        },
      });
      recentMoveRef.current = {
        entity: requestedName,
        aliases: binding.aliases,
        atMs: Date.now(),
      };
      moveTrace('drag.dispatch.accepted', {
        requestId,
        world,
        serviceName,
        msgType,
        entity: requestedName,
        aliases: binding.aliases,
        note: 'set_pose dispatched from gzweb scene manager',
      });
      let cancelConfirmation = () => {};
      const pendingConfirmation: PendingDragConfirmation = {
        requestId,
        entity: requestedName,
        cancel: () => cancelConfirmation(),
      };
      const confirmationStartedAtMs = Date.now();
      cancelConfirmation = pollForPoseConfirmation({
        isDisposed: () => disposed,
        getObservation: () =>
          resolveManipulationObservation({
            poses: entityPoseRef.current,
            binding,
            targetPose,
            toleranceMeters: DRAG_CONFIRM_TOLERANCE_METERS,
            minObservedAtMs: confirmationStartedAtMs,
          }),
        timeoutMs: DRAG_CONFIRM_TIMEOUT_MS,
        onConfirmed: ({ name: observedName, pose: observedPose, distance }) => {
          clearPendingConfirmation(pendingConfirmation);
          emitNudgeStatus('success', `Drag move confirmed for ${requestedName}`, {
            requestId,
            entity: requestedName,
            attempt: 1,
            maxAttempts: 1,
          });
          setManipStatus(`Drag move confirmed for ${requestedName}`);
          moveTrace('drag.confirmed', {
            requestId,
            world,
            entity: requestedName,
            observedName,
            observedPosition: toTracePosition(observedPose.position),
            targetPosition: toTracePosition(targetPose.position),
            distanceToExpectedMeters: Number(distance.toFixed(4)),
          });
        },
        onTimeout: (observedPose) => {
          clearPendingConfirmation(pendingConfirmation);
          emitNudgeStatus(
            'warning',
            `Drag move sent but not confirmed within ${DRAG_CONFIRM_TIMEOUT_MS}ms`,
            {
              requestId,
              entity: requestedName,
              attempt: 1,
              maxAttempts: 1,
            },
          );
          setManipStatus(`Drag move timeout for ${requestedName}`);
          moveTrace('drag.confirm.timeout', {
            requestId,
            world,
            entity: requestedName,
            targetPosition: toTracePosition(targetPose.position),
            lastObservedPosition: observedPose ? toTracePosition(observedPose.position) : undefined,
          });
        },
      });
      pendingConfirmCancels.add(cancelConfirmation);
      pendingConfirmByEntity.set(requestedName, pendingConfirmation);
      pendingConfirmByRequestId.set(requestId, pendingConfirmation);
    };

    const handleManipulationServiceReply = (payload: unknown) => {
      const replyEvent = parseManipulationServiceReplyEvent(payload);
      if (!replyEvent) return;
      const { requestId, name, world, serviceName, requestType, responseType, ok, detail } = replyEvent;
      moveTrace('drag.dispatch.reply', {
        requestId,
        entity: name,
        world,
        serviceName,
        requestType,
        responseType,
        ok: ok === null ? 'unknown' : ok,
        detail,
      });

      if (ok !== false) {
        return;
      }

      clearPendingConfirmation(pendingConfirmByRequestId.get(requestId));

      emitNudgeStatus('error', `Drag move rejected: ${detail ?? 'service reported failure'}`, {
        requestId,
        entity: name,
        attempt: 1,
        maxAttempts: 1,
      });
      setManipStatus(`Drag move rejected for ${name}`);
    };

    manager.onSceneEvent(SCENE_EVENT_MANIPULATION_STATE, handleManipulationState);
    manager.onSceneEvent(SCENE_EVENT_MANIPULATION_DISPATCH, handleManipulationDispatch);
    manager.onSceneEvent(SCENE_EVENT_MANIPULATION_SERVICE_REPLY, handleManipulationServiceReply);

    return () => {
      disposed = true;
      for (const cancel of pendingConfirmCancels) {
        cancel();
      }
      pendingConfirmCancels.clear();
      pendingConfirmByEntity.clear();
      pendingConfirmByRequestId.clear();
      manager.offSceneEvent?.(SCENE_EVENT_MANIPULATION_STATE, handleManipulationState);
      manager.offSceneEvent?.(SCENE_EVENT_MANIPULATION_DISPATCH, handleManipulationDispatch);
      manager.offSceneEvent?.(SCENE_EVENT_MANIPULATION_SERVICE_REPLY, handleManipulationServiceReply);
    };
  }, [
    directManipulationEnabled,
    dragModeEnabled,
    emitNudgeStatus,
    isConnected,
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
        const runtimePoseEntity = getRuntimePoseEntityName(entity);
        sceneManagerRef.current?.select?.(runtimePoseEntity);
        setEntityControlStatus(`Selected ${runtimePoseEntity}`);
        return;
      }

      if (message.type === ENTITY_CONTROL_MESSAGES.SCENE_SETUP_TRACE_CONFIG) {
        const payload = (message.payload ?? {}) as Partial<SceneSetupTraceConfigMessage['payload']>;
        const enabled = typeof payload.enabled === 'boolean' ? payload.enabled : false;
        const filter = typeof payload.filter === 'string' ? payload.filter.trim() : '';
        const moveThrottleMs = toFiniteNumber(payload.moveThrottleMs);
        const maxNudgeDeltaMeters = toFiniteNumber(payload.maxNudgeDeltaMeters);
        try {
          window.localStorage.setItem('tf.move.trace', enabled ? '1' : '0');
          if (filter.length > 0) {
            window.localStorage.setItem('tf.move.trace.filter', filter);
          } else {
            window.localStorage.removeItem('tf.move.trace.filter');
          }
          if (moveThrottleMs !== null && moveThrottleMs > 0) {
            window.localStorage.setItem('tf.move.throttle.ms', String(moveThrottleMs));
          }
          if (maxNudgeDeltaMeters !== null && maxNudgeDeltaMeters > 0) {
            window.localStorage.setItem('tf.move.max_delta_m', String(maxNudgeDeltaMeters));
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
          moveThrottleMs: moveThrottleMs ?? undefined,
          maxNudgeDeltaMeters: maxNudgeDeltaMeters ?? undefined,
          source: 'scene_setup',
        });
        return;
      }

      if (message.type === ENTITY_CONTROL_MESSAGES.SCENE_PRESET_SAVE) {
        const payload = (message.payload ?? {}) as { name?: unknown };
        const name = typeof payload.name === 'string' ? payload.name : '';
        clearAllThrottledNudges('scene_preset_save');
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
    [clearAllThrottledNudges, emitScenePresetList, loadScenePreset, nudgeEntity, resetAllPoses, resetEntityPose, saveScenePreset, selectedEntity],
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
    isDraggingDirectRef.current = false;
    setIsDraggingDirect(false);
    clearAllThrottledNudges('destroy_scene');
    lastNudgeDispatchAtRef.current.clear();
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
      seedPoseCacheFromScene(manager);
      manager.setManipulationTargetNames?.(selectedManipulationTargetNames);
      manager.setManipulationMode?.(
        directManipulationEnabled && dragModeEnabled && selectedEntityAllowsDirectManipulation
          ? 'translate'
          : 'view',
      );
      if (selectedEntity) {
        manager.select?.(getRuntimePoseEntityName(selectedEntity));
      }
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
  }, [
    bindResize,
    destroyScene,
    directManipulationEnabled,
    dragModeEnabled,
    fetchVmIdFromManager,
    hasVsCodeBridge,
    requestHostVmInfo,
    resetWebSocketToNative,
    seedPoseCacheFromScene,
    selectedEntity,
    selectedEntityAllowsDirectManipulation,
    selectedManipulationTargetNames,
    token,
  ]);

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
          observedAtMs: Date.now(),
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
                  isDraggingDirectRef.current = false;
                  setIsDraggingDirect(false);
                  setManipStatus('Direct manipulation disabled');
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
                  isDraggingDirectRef.current = false;
                  setIsDraggingDirect(false);
                  setManipStatus('Gizmo disarmed');
                }
                return next;
              });
            }}
          >
            {dragModeEnabled ? (isDraggingDirect ? 'Dragging...' : 'Gizmo active') : 'Click select'}
          </button>
        </div>
        <div className="gzweb-manip-hint">
          {dragModeEnabled
            ? 'Click an entity to attach the gizmo. Drag an axis handle to move it. Release to commit the new pose.'
            : 'Click an object in the viewport to select it.'}
        </div>
        <div className="gzweb-manip-status">
          Target: <code>{selectedEntity ? getRuntimePoseEntityName(selectedEntity) : 'none'}</code>
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
