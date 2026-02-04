// foxglove-networking.ts
import {
  ChannelId,
  FoxgloveClient,
  SubscriptionId,
  ServerCapability,
  ServiceCallPayload,
  ServiceCallResponse,
  Parameter,
} from "@foxglove/ws-protocol";
import { parseChannel } from "@lichtblick/mcap-support";
import { MessageWriter as Ros2MessageWriter } from "@lichtblick/rosmsg2-serialization";
import rosDatatypesToMessageDefinition from "@lichtblick/suite-base/util/rosDatatypesToMessageDefinition";
import CommonRosTypes from "@lichtblick/rosmsg-msgs-common";
import type { MessageDefinition } from "@lichtblick/message-definition";
import {
  ProxyWebSocketClient,
  getProxyWebSocketUrl,
  ROS_PORTS,
} from "./ws-proxy-client";

const textEncoder = new TextEncoder();
const textDecoder = new TextDecoder();

type FoxgloveChannel = {
  id: ChannelId;
  topic: string;
  encoding: string;            // "cdr" expected here
  schemaName: string;
  schema: string;
  schemaEncoding?: string;     // "ros2msg" | "ros2idl" | "omgidl"
};

type ParsedChannel = ReturnType<typeof parseChannel>;

interface ResolvedChannel {
  channel: FoxgloveChannel;
  parsedChannel: ParsedChannel;
}

export interface FoxgloveDecodedMessage {
  topic: string;
  schemaName: string;
  encoding: string; // "cdr"
  payload: any;     // fully decoded object
}

type SetupCommand = {
  topic: string;
  schemaName: string;
  msg: any;
};

/** Minimal shape we use from advertiseServices */
type AdvertisedService = {
  id: number;
  name: string;                // e.g. "/mavros/cmd/arming"
  type: string;                // e.g. "mavros_msgs/srv/CommandBool"
  request?: {                  // new fields
    encoding?: string;         // "cdr" expected for ROS2
    schemaName?: string;       // e.g. "mavros_msgs/srv/CommandBool_Request"
    schema?: string;           // ROS2 .msg text
    schemaEncoding?: string;   // "ros2msg" | "ros2idl"
  };
  response?: {
    encoding?: string;
    schemaName?: string;
    schema?: string;
    schemaEncoding?: string;
  };
  // Deprecated foxglove bridge fields we still tolerate:
  requestSchema?: string;
  responseSchema?: string;
};

export interface ServiceCallRequest {
  serviceName: string;
  request: any;
}

type ResolvedService = {
  service: AdvertisedService;
  requestWriter: Ros2MessageWriter;
  parsedResponse: ParsedChannel;
  requestEncoding: "cdr";
  responseEncoding: "cdr";
};

/**
 * Connection configuration for FoxgloveWsClient
 * Currently routed through the vm-manager WebSocket proxy only.
 */
export interface FoxgloveConnectionConfig {
  /** 
   * Direct connection mode: WebSocket URL to connect to (e.g., ws://172.16.0.10:8765)
   * Used when connecting directly to VM without proxy
   */
  url?: string;

  /**
   * Proxy connection mode: Connect through vm-manager WebSocket proxy
   * Requires proxyUrl (or vmManagerUrl), token, nodeId, and optionally targetPort
   */
  useProxy?: boolean;

  /** Fully qualified WebSocket proxy URL (e.g., wss://eu.vm.tensorfleet.net/ws) */
  proxyUrl?: string;

  /** VM Manager base URL for proxy mode (e.g., https://eu.vm.tensorfleet.net) */
  vmManagerUrl?: string;

  /** JWT authentication token for proxy mode */
  token?: string;

  /** VM/Node ID to connect to through proxy */
  nodeId?: string;

  /** Target port in VM (default: 8765 for Foxglove Bridge) */
  targetPort?: number;
}


export class FoxgloveWsClient {
  private client: FoxgloveClient;
  private requestCounter: number = 0;

  // Topics
  private channelsById = new Map<ChannelId, ResolvedChannel>();
  private channelsByTopic = new Map<string, ResolvedChannel>();
  private subscriptionsByTopic = new Map<string, SubscriptionId>();
  private subscriptionsById = new Map<SubscriptionId, ResolvedChannel>();
  private pendingSubscriptions = new Set<string>();
  private isOpenFlag = false;

  // Pubs
  private publicationsByTopic = new Map<
    string,
    { id: ChannelId; schemaName: string; writer?: Ros2MessageWriter }
  >();

  // Services
  private servicesByName = new Map<string, ResolvedService>();
  private nextServiceCallId = 1;
  private serviceCallbacks = new Map<number, (resp: ServiceCallResponse) => void>();
  private serviceEncoding: "cdr" | undefined;
  private servicesById = new Map<number, ResolvedService>();
  private serviceWaiters = new Map<string, Set<() => void>>();

  // Capabilities
  private supportedEncodings: string[] | undefined;
  private serverCapabilities: string[] = [];
  private rosProfile: "ros2" | undefined;

  // Datatypes cache for building writers (topics)
  private datatypesFromChannels: Map<string, MessageDefinition> = new Map();

  // Setup (latched) pubs
  private setupCommands: SetupCommand[] = [];

  // Setup (latched) service calls to be executed on connect/reconnect
  private setupServiceCalls: Array<ServiceCallRequest> = [];

  // Parameters
  private parameters = new Map<string, unknown>();
  // private parameterTypes = new Map<string, Parameter["type"]>();
  private setupParameterSets: Array<{ name: string; value: any }> = [];

  // ---- Parameter read plumbing ----
  private pendingParamReads = new Map<
    string,
    { resolve: (vals: Map<string, unknown>) => void; reject: (err: Error) => void; namesKey: string }
  >();

  private parameterListeners = new Set<(changed: { name: string; value: unknown }[]) => void>();

  // External hooks
  public onOpen?: () => void;
  public onClose?: (ev: CloseEvent | Event) => void;
  public onError?: (ev: Event) => void;
  public onNewTopic?: (topic: string, type: string) => void;
  public onMessage?: (msg: FoxgloveDecodedMessage) => void;
  public onServerInfo?: () => void;

  // Proxy client reference (if using proxy mode)
  private proxyClient: ProxyWebSocketClient | null = null;
  private connectionConfig: FoxgloveConnectionConfig;

  /**
   * Create a new FoxgloveWsClient
   * 
   * @param config - Connection configuration supporting both direct and proxy modes
   * 
   * Direct mode (legacy): { url: "ws://172.16.0.10:8765" }
   * Proxy mode: { useProxy: true, vmManagerUrl: "https://eu.vm.tensorfleet.net", token: "...", nodeId: "..." }
   */
  constructor(config: FoxgloveConnectionConfig | { url: string }) {
    // Normalize config to FoxgloveConnectionConfig
    this.connectionConfig = 'url' in config && !('useProxy' in config)
      ? { url: config.url }
      : config as FoxgloveConnectionConfig;

    const ws = this.createWebSocket();
    this.client = new FoxgloveClient({ ws });

    // Connection lifecycle
    this.client.on("open", () => {
      this.isOpenFlag = true;
      this.onOpen?.();
      this.processPendingSubscriptions();

      // Re-publish queued setup messages
      for (const cmd of this.setupCommands) {
        try {
          this.publish(cmd.topic, cmd.schemaName, cmd.msg);
        } catch (err) {
          console.error("[FoxgloveWsClient] Re-publish setup error:", err);
        }
      }

      // Execute any queued setup service calls (will be retried after services advertise)
      if (this.areStartupServicesReady()) {
        this.processSetupServiceCalls();
      }
    });

    this.client.on("close", (ev) => {
      this.isOpenFlag = false;
      this.onClose?.(ev as unknown as CloseEvent);
    });

    this.client.on("error", (ev) => {
      this.onError?.(ev as unknown as Event);
    });

    // Server info (encodings, capabilities, ROS profile)
    this.client.on("serverInfo", (event) => {
      this.supportedEncodings = event.supportedEncodings;

      this.serverCapabilities = Array.isArray(event.capabilities) ? event.capabilities : [];

      if (Array.isArray(event.capabilities)) {
        console.log("[FoxgloveWsClient] Got server capabilities ", event.capabilities);
      }

      const maybeRosDistro = event.metadata?.["ROS_DISTRO"];
      if (maybeRosDistro) {
        this.rosProfile = "ros2";
      }

      // Prefer "cdr" for ROS2 services
      this.serviceEncoding = (event.supportedEncodings ?? []).includes("cdr") ? "cdr" : undefined;

      // Parameters capability: fetch all params initially so we know types
      if (this.serverCapabilities.includes(ServerCapability.parameters)) {
        try {
          console.log("[FoxgloveWsClient] requesting parameterValues");
          // Empty names list => request "all parameters" (Foxglove Bridge behavior)
          this.client.getParameters([], `${++this.requestCounter}`);
        } catch (err) {
          console.warn("[FoxgloveWsClient] Failed to request initial parameters:", err);
        }
      }

      // Workaround. for now do this every time this happens.
      this.processSetupParameterSets();

      this.onServerInfo?.();
    });

    // Channel advertisement (topics)
    this.client.on("advertise", (channels: FoxgloveChannel[]) => {
      for (const channel of channels) {
        if (channel.encoding !== "cdr") {
          console.warn("[FoxgloveWsClient] Skipping non-CDR channel", channel.topic, "encoding:", channel.encoding);
          continue;
        }

        let schemaEncoding: string;
        if (
          channel.schemaEncoding == undefined ||
          ["ros2idl", "ros2msg", "omgidl"].includes(channel.schemaEncoding)
        ) {
          schemaEncoding = channel.schemaEncoding ?? "ros2msg";
        } else {
          console.warn("[FoxgloveWsClient] Unsupported schemaEncoding for CDR channel", channel.topic, channel.schemaEncoding);
          continue;
        }

        const schemaData = textEncoder.encode(channel.schema);

        let parsedChannel: ParsedChannel;
        try {
          parsedChannel = parseChannel({
            messageEncoding: channel.encoding,
            schema: { name: channel.schemaName, encoding: schemaEncoding, data: schemaData },
          });
        } catch (err) {
          console.error("[FoxgloveWsClient] Failed to parse channel schema for", channel.topic, err);
          continue;
        }

        const resolved: ResolvedChannel = { channel, parsedChannel };
        this.channelsById.set(channel.id, resolved);
        this.channelsByTopic.set(channel.topic, resolved);

        // cache datatypes for writers
        for (const [name, def] of parsedChannel.datatypes) {
          this.datatypesFromChannels.set(name, def);
        }

        this.onNewTopic?.(channel.topic, channel.schemaName);
      }

      this.processPendingSubscriptions();
    });

    this.client.on("unadvertise", (removedIds: ChannelId[]) => {
      for (const id of removedIds) {
        const chanInfo = this.channelsById.get(id);
        if (!chanInfo) continue;
        this.channelsById.delete(id);
        this.channelsByTopic.delete(chanInfo.channel.topic);
      }
    });

    // Topic messages
    this.client.on("message", ({ subscriptionId, data }) => {
      const chanInfo = this.subscriptionsById.get(subscriptionId);
      if (!chanInfo) return;

      try {
        const decoded = chanInfo.parsedChannel.deserialize(data);
        const { topic, schemaName, encoding } = chanInfo.channel;
        this.onMessage?.({ topic, schemaName, encoding, payload: decoded });
      } catch (err) {
        console.error("[FoxgloveWsClient] Failed to decode message on topic", chanInfo.channel.topic, err);
      }
    });

    // --- Services ---

    this.client.on("advertiseServices", (services: AdvertisedService[]) => {
      if (!Array.isArray(services)) return;
      const needsStartup = !this.areStartupServicesReady();

      const parseOpts = { allowEmptySchema: true }; // <-- IMPORTANT

      for (const service of services) {
        try {
          const reqEncoding = (service.request?.encoding ?? this.serviceEncoding) as "cdr";
          const resEncoding = (service.response?.encoding ?? this.serviceEncoding) as "cdr";
          if (reqEncoding !== "cdr" || resEncoding !== "cdr") {
            console.warn("[FoxgloveWsClient] Skipping service (non-CDR):", service.name);
            continue;
          }

          const reqSchemaName = service.request?.schemaName ?? `${service.type}_Request`;
          const resSchemaName = service.response?.schemaName ?? `${service.type}_Response`;

          const reqSchema = service.request?.schema ?? service.requestSchema ?? "";
          const resSchema = service.response?.schema ?? service.responseSchema ?? "";

          const parsedReq = parseChannel({
            messageEncoding: "cdr",
            schema: {
              name: reqSchemaName,
              encoding: service.request?.schemaEncoding ?? "ros2msg",
              data: textEncoder.encode(reqSchema),
            },
          }, parseOpts);

          const parsedRes = parseChannel({
            messageEncoding: "cdr",
            schema: {
              name: resSchemaName,
              encoding: service.response?.schemaEncoding ?? "ros2msg",
              data: textEncoder.encode(resSchema),
            },
          }, parseOpts);

          for (const [n, d] of parsedReq.datatypes) this.datatypesFromChannels.set(n, d);
          for (const [n, d] of parsedRes.datatypes) this.datatypesFromChannels.set(n, d);

          const msgdefReq = rosDatatypesToMessageDefinition(parsedReq.datatypes, reqSchemaName);
          const requestWriter = new Ros2MessageWriter(msgdefReq);

          const resolved: ResolvedService = {
            service,
            requestWriter,
            parsedResponse: parsedRes,
            requestEncoding: "cdr",
            responseEncoding: "cdr",
          };

          this.servicesByName.set(service.name, resolved);
          this.servicesById.set(service.id, resolved);

          // after: this.servicesByName.set(service.name, resolved);
          const waiters = this.serviceWaiters.get(service.name);
          if (waiters) {
            for (const fn of waiters) fn();
            this.serviceWaiters.delete(service.name);
          }

          console.log(`[FoxgloveWsClient] added service ${service.name}:${service.type}`);
        } catch (err) {
          console.error("[FoxgloveWsClient] Failed to parse service", service.name, err);
        }
      }

      // Now that services are available, try executing any queued setup service calls
      if (needsStartup && this.areStartupServicesReady()) {
        this.processSetupServiceCalls();
      }
    });

    this.client.on("serviceCallResponse", (resp: ServiceCallResponse) => {
      const cb = this.serviceCallbacks.get(resp.callId);
      if (!cb) {
        console.warn("[FoxgloveWsClient] Unhandled serviceCallResponse", resp.callId);
        return;
      }

      const svc = this.servicesById.get(resp.serviceId);
      if (svc) {
        try {
          const bytes = new Uint8Array(
            resp.data.buffer,
            resp.data.byteOffset,
            resp.data.byteLength,
          );
          const decoded = svc.parsedResponse.deserialize(bytes);
          console.log(
            `[FoxgloveWsClient] service call ${resp.callId} (${svc.service.name}) decoded response`,
            decoded,
          );
        } catch (err) {
          console.error(
            `[FoxgloveWsClient] Failed to decode response for service call ${resp.callId} (${svc?.service.name})`,
            err,
          );
        }
      } else {
        console.log(
          `[FoxgloveWsClient] service call ${resp.callId} (serviceId=${resp.serviceId}) got response (no decoder)`,
          resp,
        );
      }

      this.serviceCallbacks.delete(resp.callId);
      cb(resp);
    });

    // --- Parameters ---

    this.client.on("parameterValues", (event: any) => {
      // Foxglove ws-protocol typically includes { id, parameters }
      const parameters = event?.parameters;
      const requestId: string | undefined = event?.id;

      if (!Array.isArray(parameters)) return;

      const changed: { name: string; value: unknown }[] = [];

      for (const p of parameters as Array<{ name: string; value: unknown; type?: Parameter["type"] }>) {
        let val: unknown = (p as any).value;

        // byte_array comes base64-encoded string in Foxglove protocol
        if ((p as any).type === "byte_array" && typeof val === "string") {
          const s = atob(val);
          const out = new Uint8Array(s.length);
          for (let i = 0; i < s.length; i++) out[i] = s.charCodeAt(i);
          val = out;
        }

        const prev = this.parameters.get(p.name);
        this.parameters.set(p.name, val);

        // track changes (shallow)
        if (prev !== val) {
          changed.push({ name: p.name, value: val });
        }
      }

      if (changed.length > 0) {
        for (const fn of this.parameterListeners) {
          try {
            fn(changed);
          } catch (e) {
            console.warn("[FoxgloveWsClient] parameter listener error:", e);
          }
        }
      }

      // Resolve pending read for this requestId if present
      if (requestId && this.pendingParamReads.has(requestId)) {
        const pending = this.pendingParamReads.get(requestId)!;
        this.pendingParamReads.delete(requestId);

        // Build result map for the requested names (or all if request was empty)
        const out = new Map<string, unknown>();

        // If they asked for "all" (empty list), return everything we currently have
        if (pending.namesKey === "") {
          for (const [k, v] of this.parameters.entries()) out.set(k, v);
        } else {
          // namesKey is sorted join("|"), so reconstruct list
          const requested = pending.namesKey.split("|").filter(Boolean);
          for (const name of requested) out.set(name, this.parameters.get(name));
        }

        pending.resolve(out);
      } else if (!requestId) {
        // If the server doesn't include ids, best-effort resolve ALL pending reads
        // by letting cache update be the "response".
        if (this.pendingParamReads.size > 0) {
          for (const [rid, pending] of Array.from(this.pendingParamReads.entries())) {
            this.pendingParamReads.delete(rid);

            const out = new Map<string, unknown>();
            if (pending.namesKey === "") {
              for (const [k, v] of this.parameters.entries()) out.set(k, v);
            } else {
              const requested = pending.namesKey.split("|").filter(Boolean);
              for (const name of requested) out.set(name, this.parameters.get(name));
            }
            pending.resolve(out);
          }
        }
      }
    });
  }

  /**
   * Create the appropriate WebSocket connection based on config
   * Returns a WebSocket for direct mode, or a proxy-backed WebSocket for proxy mode
   */
  private createWebSocket(): WebSocket {
    const config = this.connectionConfig;
    const useProxy = config.useProxy ?? true;

    // Proxy mode: connect through vm-manager
    if (useProxy) {
      const proxyUrl = config.proxyUrl
        ? getProxyWebSocketUrl(config.proxyUrl)
        : config.vmManagerUrl
          ? getProxyWebSocketUrl(config.vmManagerUrl)
          : undefined;

      if (!proxyUrl) {
        throw new Error("[FoxgloveWsClient] Proxy mode requires proxyUrl or vmManagerUrl");
      }
      if (!config.token) {
        throw new Error("[FoxgloveWsClient] Proxy mode requires token");
      }
      if (!config.nodeId) {
        throw new Error("[FoxgloveWsClient] Proxy mode requires nodeId");
      }

      const targetPort = config.targetPort ?? ROS_PORTS.FOXGLOVE_BRIDGE;

      console.log(`[FoxgloveWsClient] Creating proxy connection to ${proxyUrl} for node ${config.nodeId}:${targetPort}`);

      // Create the proxy client
      this.proxyClient = new ProxyWebSocketClient(
        {
          proxyUrl,
          token: config.token,
          nodeId: config.nodeId,
          targetPort,
          subprotocols: [FoxgloveClient.SUPPORTED_SUBPROTOCOL],
        },
        {
          onStateChange: (state) => {
            console.log(`[FoxgloveWsClient] Proxy state: ${state}`);
          },
          onLoginFailed: (response) => {
            console.error(`[FoxgloveWsClient] Proxy login failed: ${response.message}`);
          },
        }
      );

      // Return a WebSocket-like shim that manages the proxy handshake before opening
      return this.createProxyWebSocketShim(proxyUrl);
    }

    throw new Error("[FoxgloveWsClient] Proxy mode only is supported.");
  }

  /**
   * Create a WebSocket-like shim for proxy mode
   * This allows FoxgloveClient to attach handlers before the proxy connects
   */
  private createProxyWebSocketShim(proxyUrl: string): WebSocket {
    const proxyClient = this.proxyClient!;
    const requestedProtocol = FoxgloveClient.SUPPORTED_SUBPROTOCOL;

    // Capture any existing handlers so we can wrap them
    const baseHandlers = proxyClient.getEventHandlers();

    // Create event handler storage
    let onopen: ((ev: Event) => void) | null = null;
    let onclose: ((ev: CloseEvent) => void) | null = null;
    let onerror: ((ev: Event) => void) | null = null;
    let onmessage: ((ev: MessageEvent) => void) | null = null;

    // Track ready state
    let readyState: number = WebSocket.CONNECTING;
    let negotiatedProtocol: string = requestedProtocol;

    // Message queue for messages sent before connected (normalized to string or ArrayBuffer)
    const messageQueue: Array<string | ArrayBuffer> = [];

    // Create the shim object
    const shim = {
      binaryType: "arraybuffer" as BinaryType,

      get readyState() {
        return readyState;
      },

      get onopen() { return onopen; },
      set onopen(handler: ((ev: Event) => void) | null) {
        onopen = handler;
      },

      get onclose() { return onclose; },
      set onclose(handler: ((ev: CloseEvent) => void) | null) {
        onclose = handler;
      },

      get onerror() { return onerror; },
      set onerror(handler: ((ev: Event) => void) | null) {
        onerror = handler;
      },

      get onmessage() { return onmessage; },
      set onmessage(handler: ((ev: MessageEvent) => void) | null) {
        onmessage = handler;
      },

      send(data: string | ArrayBuffer | ArrayBufferView) {
        const normalized: string | ArrayBuffer =
          typeof data === "string"
            ? data
            : data instanceof ArrayBuffer
              ? data
              : new Uint8Array(data.buffer, data.byteOffset, data.byteLength).slice().buffer;

        if (readyState !== WebSocket.OPEN) {
          // Queue messages until connected
          messageQueue.push(normalized);
          return;
        }

        proxyClient.send(normalized);
      },

      close(code?: number, reason?: string) {
        readyState = WebSocket.CLOSING;
        proxyClient.close();
        readyState = WebSocket.CLOSED;
      },

      // Constants
      CONNECTING: WebSocket.CONNECTING,
      OPEN: WebSocket.OPEN,
      CLOSING: WebSocket.CLOSING,
      CLOSED: WebSocket.CLOSED,

      // Additional required properties
      bufferedAmount: 0,
      extensions: "",
      get protocol() {
        return negotiatedProtocol;
      },
      url: proxyUrl,

      // Event listener methods (not used by FoxgloveClient but required by interface)
      addEventListener: () => { },
      removeEventListener: () => { },
      dispatchEvent: () => true,
    } as WebSocket;

    // Wire up proxy events to the shim
    // Use a small delay to let FoxgloveClient attach its handlers
    setTimeout(() => {
      proxyClient.setEventHandlers({
        ...baseHandlers,
        onOpen: () => {
          readyState = WebSocket.OPEN;
          negotiatedProtocol = proxyClient.getRawWebSocket()?.protocol || negotiatedProtocol;
          baseHandlers.onOpen?.();

          // Flush queued messages
          while (messageQueue.length > 0) {
            const msg = messageQueue.shift()!;
            proxyClient.send(msg);
          }

          // Trigger onopen handler
          onopen?.({ type: "open", target: shim } as unknown as Event);
        },
        onClose: (ev: CloseEvent) => {
          readyState = WebSocket.CLOSED;
          baseHandlers.onClose?.(ev);
          onclose?.(ev);
        },
        onError: (ev: Event | Error) => {
          baseHandlers.onError?.(ev as Event);
          onerror?.(ev instanceof Error ? { type: "error", error: ev, target: shim } as unknown as Event : (ev as Event));
        },
        onMessage: (data: ArrayBuffer | string) => {
          baseHandlers.onMessage?.(data);
          onmessage?.({ data, type: "message", target: shim } as unknown as MessageEvent);
        },
      });

      // Start the connection
      proxyClient.connect();
    }, 0);

    return shim;
  }


  // ---------- Subscriptions ----------
  private processPendingSubscriptions() {
    if (!this.isOpenFlag) return;

    for (const topic of Array.from(this.pendingSubscriptions)) {
      console.log("[FoxgloveWsClient] Processing pending subscription to ", topic)
      const chanInfo = this.channelsByTopic.get(topic);
      if (!chanInfo) continue;

      const subId = this.client.subscribe(chanInfo.channel.id);
      this.subscriptionsByTopic.set(topic, subId);
      this.subscriptionsById.set(subId, chanInfo);
      this.pendingSubscriptions.delete(topic);
    }
  }

  public subscribe(topic: string) {
    this.pendingSubscriptions.add(topic);
    this.processPendingSubscriptions();
  }

  public unsubscribe(topic: string) {
    const subId = this.subscriptionsByTopic.get(topic);
    if (subId != undefined) {
      this.client.unsubscribe(subId);
      this.subscriptionsByTopic.delete(topic);
      this.subscriptionsById.delete(subId);
    }
    this.pendingSubscriptions.delete(topic);
  }

  public close() {
    this.client.close();
    this.isOpenFlag = false;
  }

  public isConnected(): boolean {
    return this.isOpenFlag;
  }

  // ---------- Queued setup publishing ----------
  /**
   * Queue a setup publish that will be sent immediately if connected and
   * automatically re-published on each reconnect.
   */
  public publishSetup(topic: string, schemaName: string, msg: any) {
    this.setupCommands.push({ topic, schemaName, msg });
    if (this.isOpenFlag) {
      try {
        this.publish(topic, schemaName, msg);
      } catch (err) {
        console.error(
          `[FoxgloveWsClient] Failed to publish setup message for '${topic}' (${schemaName})`,
          err,
        );
      }
    }
  }

  // ---------- Queued setup service calls ----------
  /**
   * Queue a setup service call that will be attempted on connect and on each reconnect.
   * Calls are re-attempted after services are advertised.
   */
  public registerSetupServiceCall(request: ServiceCallRequest) {
    console.log("[FoxgloveWsClient] Registarting startup service call ", request);
    this.setupServiceCalls.push(request);
    // If already open, try immediately (will throw if service isn't up yet; swallow)
    if (this.isOpenFlag) {
      if (this.servicesByName.has(request.serviceName)) {
        this.callService(request);
      }
    } else {
      console.log(`[FoxgloveWsClient] queueing startup service call :`, request)
    }
  }


  private areStartupServicesReady(): boolean {
    for (const request of this.setupServiceCalls) {
      if (!this.servicesByName.has(request.serviceName)) {
        return false;
      }
    }

    return true;
  }

  private async processSetupServiceCalls() {
    console.log("[FoxgloveWsClient] Processing startup service calls")

    if (!this.isOpenFlag || this.setupServiceCalls.length === 0) return;
    if (!this.areStartupServicesReady()) {
      console.log(`[FoxgloveWsClient] setup waiting for all startup services to be available.`);
      return
    }
    for (const request of this.setupServiceCalls) {
      try {
        await this.callService(request);
      } catch (err) {
        // Service may not be advertised yet; ignore and rely on next attempt after advertiseServices
        console.warn(`[FoxgloveWsClient] setup service call deferred: ${request.serviceName}`, err instanceof Error ? err.message : err);
      }
    }
  }

  // ---------- PUBLISH (CDR) ----------
  private buildRos2WriterFor(schemaName: string): Ros2MessageWriter | undefined {
    // Try datatypes from already parsed channels (best match)
    for (const { parsedChannel } of this.channelsById.values()) {
      try {
        const msgdef = rosDatatypesToMessageDefinition(parsedChannel.datatypes, schemaName);
        return new Ros2MessageWriter(msgdef);
      } catch {
        // not found here; continue
      }
    }

    // Fallback to shipped ROS2 types (Humble preferred, else Galactic)
    const ros2 = (CommonRosTypes as any).ros2humble ?? (CommonRosTypes as any).ros2galactic;
    if (ros2) {
      const datatypes = new Map<string, MessageDefinition>();
      for (const name in ros2) {
        datatypes.set(name, (ros2 as Record<string, MessageDefinition>)[name]!);
      }
      try {
        const msgdef = rosDatatypesToMessageDefinition(datatypes, schemaName);
        return new Ros2MessageWriter(msgdef);
      } catch {
        // fall through
      }
    }
    return undefined;
  }

  private ensureAdvertisedCDR(topic: string, schemaName: string): {
    id: ChannelId;
    writer?: Ros2MessageWriter;
  } {
    const existing = this.publicationsByTopic.get(topic);
    if (existing) return { id: existing.id, writer: existing.writer };

    // Verify (or at least warn) that 'cdr' is supported for client publish
    if (this.supportedEncodings && !this.supportedEncodings.includes("cdr")) {
      // proceed anyway; some servers accept regardless
    }

    // Some servers require clientPublish capability; many accept regardless.
    const channelId = this.client.advertise({
      topic,
      encoding: "cdr",
      schemaName,
    });

    const writer = this.buildRos2WriterFor(schemaName);
    this.publicationsByTopic.set(topic, { id: channelId, schemaName, writer });
    return { id: channelId, writer };
  }

  public publish(topic: string, schemaName: string, msg: any) {
    const { id, writer } = this.ensureAdvertisedCDR(topic, schemaName);

    let w = writer;
    if (!w) {
      const built = this.buildRos2WriterFor(schemaName);
      if (!built) {
        console.error(
          `[FoxgloveWsClient] Cannot publish on '${topic}' (${schemaName}): no ROS2 message definition found`,
        );
        return;
      }
      this.publicationsByTopic.set(topic, { id, schemaName, writer: built });
      w = built;
    }

    try {
      const bytes = w.writeMessage(msg);
      this.client.sendMessage(id, bytes);
    } catch (err) {
      console.error(
        `[FoxgloveWsClient] Failed to serialize message for '${topic}' (${schemaName})`,
        err,
      );
    }
  }

  // ---------- SERVICES (CDR, ROS 2) ----------
  public async waitForService(serviceName: string, timeoutMs = 10_000): Promise<void> {
    if (this.servicesByName.has(serviceName)) return;

    await new Promise<void>((resolve, reject) => {
      const t = setTimeout(() => {
        // remove waiter
        const set = this.serviceWaiters.get(serviceName);
        if (set) {
          set.delete(resolve);
          if (set.size === 0) this.serviceWaiters.delete(serviceName);
        }
        reject(new Error(`Timeout waiting for service: ${serviceName}`));
      }, timeoutMs);

      const wrappedResolve = () => {
        clearTimeout(t);
        resolve();
      };

      let set = this.serviceWaiters.get(serviceName);
      if (!set) {
        set = new Set();
        this.serviceWaiters.set(serviceName, set);
      }
      set.add(wrappedResolve);
    });
  }

  public async callService<T = any>(call: ServiceCallRequest): Promise<T> {
    const { serviceName, request } = call;
    const svc = this.servicesByName.get(serviceName);
    if (!svc) {
      throw new Error(`Service '${serviceName}' not advertised (yet).`);
    }
    if (svc.requestEncoding !== "cdr" || svc.responseEncoding !== "cdr") {
      throw new Error(`Service '${serviceName}' uses unsupported encoding (only 'cdr' supported).`);
    }

    console.log("[FoxgloveWsClient] Sending service call to ", serviceName);

    const callId = this.nextServiceCallId++;
    const payload: ServiceCallPayload = {
      serviceId: svc.service.id,
      callId,
      encoding: "cdr",
      data: new DataView(new Uint8Array().buffer),
    };

    try {
      const bytes = svc.requestWriter.writeMessage(request);

      payload.data = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    } catch (err) {
      throw new Error(`Failed to serialize service request for '${serviceName}': ${String(err)}`);
    }

    const response = await new Promise<ServiceCallResponse>((resolve, reject) => {
      const timeout = setTimeout(() => {
        this.serviceCallbacks.delete(callId);
        reject(new Error(`Service call timeout: ${serviceName}`));
      }, 30000);

      this.serviceCallbacks.set(callId, (resp) => {
        clearTimeout(timeout);
        resolve(resp);
      });

      this.client.sendServiceCallRequest(payload);
    });

    try {
      const bytes = new Uint8Array(
        response.data.buffer,
        response.data.byteOffset,
        response.data.byteLength,
      );
      const out = svc.parsedResponse.deserialize(bytes);
      return out as T;
    } catch (err) {
      throw new Error(`Failed to deserialize service response for '${serviceName}': ${String(err)}`);
    }
  }

  // ---------- PARAMETERS (ROS 2 via Foxglove) ----------

  /**
   * Queue a startup parameter set. It will only be sent once the server has advertised
   * its parameter list and the specific parameter names are known (types available).
   * Queued sets will be re-applied on reconnect once parameters are fetched again.
   */
  public registerSetupParameterSet(name: string, value: any) {
    console.log("[FoxgloveWsClient] Registering startup parameter set ", { name, value });
    this.setupParameterSets.push({ name, value });
    // If we are already connected and types are known for this param, try processing now.
    if (this.isOpenFlag && this.areStartupParametersReady()) {
      this.setParameter(name, value);
    }
  }

  /**
   * Immediately set a parameter on the server. Requires parameter type to be known.
   */
  public setParameter(name: string, value: any) {
    console.log(`[FoxgloveWsClient] setting ROS param ${name} to `, value);

    if (!this.serverCapabilities.includes(ServerCapability.parameters)) {
      throw new Error("Server does not support parameters capability");
    }

    const param: Parameter = {
      name,
      value: value as Parameter["value"],
    };

    // Special case: binary data -> byte_array
    if (value instanceof Uint8Array) {
      let s = "";
      for (let i = 0; i < value.length; i++) {
        s += String.fromCharCode(value[i]);
      }
      param.value = btoa(s);
      param.type = "byte_array";
    }

    // You *could* add heuristics here, e.g. if Array<number> -> float64_array,
    // but it's not required. Leaving type undefined is fine for normal JSON
    // scalars / arrays / objects.

    try {
      this.client.setParameters(
        [param],
        `${++this.requestCounter}`, // request id
      );
      // Optimistic local cache update
      this.parameters.set(name, value);
      console.log(`[FoxgloveWsClient] setParameter sent: ${name}`);
    } catch (err) {
      console.error(`[FoxgloveWsClient] setParameter failed: ${name}`, err);
      throw err;
    }
  }

  private areStartupParametersReady(): boolean {


    if (!this.serverCapabilities.includes(ServerCapability.parameters)) return false;
    return true;
    // Workaround. for now don't check.
    // for (const { name } of this.setupParameterSets) {
    //   if (!this.parameterTypes.has(name)) return false;
    // }
    // return true;
  }

  processSetupParameterSets() {
    if (!this.isOpenFlag || this.setupParameterSets.length === 0) return;
    if (!this.areStartupParametersReady()) {
      console.log(`[FoxgloveWsClient] setup parameters waiting for server parameter list/types.`);
      return;
    }

    console.log("[FoxgloveWsClient] Processing startup parameter sets ", this.setupParameterSets);
    for (const { name, value } of this.setupParameterSets) {
      try {
        this.setParameter(name, value);
      } catch (err) {
        console.warn(`[FoxgloveWsClient] setup parameter set deferred: ${name}`, err instanceof Error ? err.message : err);
      }
    }
  }

  /** Subscribe to local parameter-cache updates */
  public onParameterChanged(cb: (changed: { name: string; value: unknown }[]) => void): () => void {
    this.parameterListeners.add(cb);
    return () => this.parameterListeners.delete(cb);
  }

  /** Local cache read (no network) */
  public getCachedParameter(name: string): unknown | undefined {
    return this.parameters.get(name);
  }

  /** List locally-known parameter names (no network) */
  public listCachedParameters(): string[] {
    return Array.from(this.parameters.keys()).sort();
  }

  /** Fetch one parameter value (uses cache; falls back to getParameters([name])) */
  public async getParameter(name: string, opts?: { force?: boolean; timeoutMs?: number }): Promise<unknown> {
    const force = opts?.force ?? false;
    const timeoutMs = opts?.timeoutMs ?? 10_000;

    if (!force && this.parameters.has(name)) {
      return this.parameters.get(name);
    }

    const res = await this.getParameters([name], { timeoutMs });
    return res.get(name);
  }

  /** Fetch many parameter values */
  public async getParameters(
    names: string[],
    opts?: { timeoutMs?: number },
  ): Promise<Map<string, unknown>> {
    if (!this.serverCapabilities.includes(ServerCapability.parameters)) {
      throw new Error("Server does not support parameters capability");
    }
    if (!this.isOpenFlag) {
      throw new Error("Not connected");
    }

    const timeoutMs = opts?.timeoutMs ?? 10_000;
    const requestId = `${++this.requestCounter}`;

    // Normalize names: empty array => request all (Foxglove bridge convention)
    const normalized = Array.isArray(names) ? names : [];
    const namesKey = normalized.slice().sort().join("|");

    return await new Promise<Map<string, unknown>>((resolve, reject) => {
      const t = window.setTimeout(() => {
        this.pendingParamReads.delete(requestId);
        reject(new Error(`getParameters timeout (id=${requestId}, names=${JSON.stringify(normalized)})`));
      }, timeoutMs);

      this.pendingParamReads.set(requestId, {
        namesKey,
        resolve: (vals) => {
          clearTimeout(t);
          resolve(vals);
        },
        reject: (err) => {
          clearTimeout(t);
          reject(err);
        },
      });

      try {
        this.client.getParameters(normalized, requestId);
      } catch (err) {
        clearTimeout(t);
        this.pendingParamReads.delete(requestId);
        reject(err instanceof Error ? err : new Error(String(err)));
      }
    });
  }

  /** Explicit "fetch all" helper */
  public async getAllParameters(opts?: { timeoutMs?: number }): Promise<Map<string, unknown>> {
    return await this.getParameters([], opts);
  }

  getAvailableTopics(): { topic: string, type: string }[] {
    return Array.from(this.channelsByTopic.entries()).map(([topic, type]) => ({ topic, type: type.channel.schemaName }));
  }

  getTopicType(topic: string): string | undefined {
    return this.channelsByTopic.get(topic)?.channel.schemaName;
  }
}
