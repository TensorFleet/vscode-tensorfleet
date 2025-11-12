// foxglove-networking.ts
import {
  ChannelId,
  FoxgloveClient,
  SubscriptionId,
  ServerCapability,
  ServiceCallPayload,
  ServiceCallResponse,
} from "@foxglove/ws-protocol";
import { parseChannel } from "./lichtblick/mcap-support";
import { MessageWriter as Ros2MessageWriter } from "@lichtblick/rosmsg2-serialization";
import rosDatatypesToMessageDefinition from "./lichtblick/suite-base/util/rosDatatypesToMessageDefinition";
import CommonRosTypes from "@lichtblick/rosmsg-msgs-common";
import type { MessageDefinition } from "@lichtblick/message-definition";

const textEncoder = new TextEncoder();

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

type ResolvedService = {
  service: AdvertisedService;
  requestWriter: Ros2MessageWriter;
  parsedResponse: ParsedChannel;
  requestEncoding: "cdr";
  responseEncoding: "cdr";
};

export class FoxgloveWsClient {
  private client: FoxgloveClient;

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

  // Capabilities
  private supportedEncodings: string[] | undefined;
  private serverCapabilities: string[] = [];
  private rosProfile: "ros2" | undefined;

  // Datatypes cache for building writers (topics)
  private datatypesFromChannels: Map<string, MessageDefinition> = new Map();

  // Setup (latched) pubs
  private setupCommands: SetupCommand[] = [];

  // External hooks
  public onOpen?: () => void;
  public onClose?: (ev: CloseEvent | Event) => void;
  public onError?: (ev: Event) => void;
  public onNewTopic?: (topic: string, type: string) => void;
  public onMessage?: (msg: FoxgloveDecodedMessage) => void;

  constructor({ url }: { url: string }) {
    this.client = new FoxgloveClient({
      ws: new WebSocket(url, [FoxgloveClient.SUPPORTED_SUBPROTOCOL]),
    });

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

      const maybeRosDistro = event.metadata?.["ROS_DISTRO"];
      if (maybeRosDistro) {
        this.rosProfile = "ros2";
      }

      // Prefer "cdr" for ROS2 services
      this.serviceEncoding = (event.supportedEncodings ?? []).includes("cdr") ? "cdr" : undefined;
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

          this.servicesByName.set(service.name, {
            service,
            requestWriter,
            parsedResponse: parsedRes,
            requestEncoding: "cdr",
            responseEncoding: "cdr",
          });
        } catch (err) {
          console.error("[FoxgloveWsClient] Failed to parse service", service.name, err);
        }
      }
    });
  }

  // ---------- Subscriptions ----------
  private processPendingSubscriptions() {
    if (!this.isOpenFlag) return;

    for (const topic of Array.from(this.pendingSubscriptions)) {
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
  public async callService<T = any>(serviceName: string, request: any): Promise<T> {
    const svc = this.servicesByName.get(serviceName);
    if (!svc) {
      throw new Error(`Service '${serviceName}' not advertised (yet).`);
    }
    if (svc.requestEncoding !== "cdr" || svc.responseEncoding !== "cdr") {
      throw new Error(`Service '${serviceName}' uses unsupported encoding (only 'cdr' supported).`);
    }

    const callId = this.nextServiceCallId++;
    const payload: ServiceCallPayload = {
      serviceId: svc.service.id,
      callId,
      encoding: "cdr",
      data: new DataView(new Uint8Array().buffer),
    };

    try {
      const bytes = svc.requestWriter.writeMessage(request);
      payload.data = new DataView(bytes.buffer);
    } catch (err) {
      throw new Error(`Failed to serialize service request for '${serviceName}': ${String(err)}`);
    }

    const response = await new Promise<ServiceCallResponse>((resolve, reject) => {
      const timeout = setTimeout(() => {
        this.serviceCallbacks.delete(callId);
        reject(new Error(`Service call timeout: ${serviceName}`));
      }, 10000);

      this.serviceCallbacks.set(callId, (resp) => {
        clearTimeout(timeout);
        resolve(resp);
      });

      this.client.sendServiceCallRequest(payload);
    });

    try {
      const out = svc.parsedResponse.deserialize(response.data);
      return out as T;
    } catch (err) {
      throw new Error(`Failed to deserialize service response for '${serviceName}': ${String(err)}`);
    }
  }
}
