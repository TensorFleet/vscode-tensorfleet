// FoxgloveNetworking.ts
import {
  ChannelId,
  FoxgloveClient,
  SubscriptionId,
} from "@foxglove/ws-protocol";
import { parseChannel } from "./lichtblick/mcap-support";

const textEncoder = new TextEncoder();

type FoxgloveChannel = {
  id: ChannelId;
  topic: string;
  encoding: string;
  schemaName: string;
  schema: string;
  schemaEncoding?: string;
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
  payload: any;     // fully decoded JS object
}

export class FoxgloveWsClient {
  private client: FoxgloveClient;
  private channelsById = new Map<ChannelId, ResolvedChannel>();
  private channelsByTopic = new Map<string, ResolvedChannel>();
  private subscriptionsByTopic = new Map<string, SubscriptionId>();
  private subscriptionsById = new Map<SubscriptionId, ResolvedChannel>();
  private pendingSubscriptions = new Set<string>();
  private isOpenFlag = false;

  // Callbacks wired by ROS2Bridge
  public onOpen?: () => void;
  public onClose?: (ev: CloseEvent | Event) => void;
  public onError?: (ev: Event) => void;
  public onNewTopic?: (topic: string, type: string) => void;
  public onMessage?: (msg: FoxgloveDecodedMessage) => void;

  constructor({ url }: { url: string }) {
    this.client = new FoxgloveClient({
      ws: new WebSocket(url, [FoxgloveClient.SUPPORTED_SUBPROTOCOL]),
    });

    this.client.on("open", () => {
      this.isOpenFlag = true;
      this.onOpen?.();
      // Re-subscribe to pending topics when connection opens
      this.processPendingSubscriptions();
    });

    this.client.on("close", (ev) => {
      this.isOpenFlag = false;
      this.onClose?.(ev as unknown as CloseEvent);
      // we don't clear state here; reconnect logic lives in ROS2Bridge
    });

    this.client.on("error", (ev) => {
      this.onError?.(ev as unknown as Event);
    });

    // Channels + schemas
    this.client.on("advertise", (channels: FoxgloveChannel[]) => {
      for (const channel of channels) {
        // Only support CDR
        if (channel.encoding !== "cdr") {
          console.warn(
            "[FoxgloveWsClient] Skipping non-CDR channel",
            channel.topic,
            "encoding:",
            channel.encoding,
          );
          continue;
        }

        let schemaEncoding: string;
        if (
          channel.schemaEncoding == undefined ||
          ["ros2idl", "ros2msg", "omgidl"].includes(channel.schemaEncoding)
        ) {
          schemaEncoding = channel.schemaEncoding ?? "ros2msg";
        } else {
          console.warn(
            "[FoxgloveWsClient] Unsupported schemaEncoding for CDR channel",
            channel.topic,
            channel.schemaEncoding,
          );
          continue;
        }

        const schemaData = textEncoder.encode(channel.schema);

        let parsedChannel: ParsedChannel;
        try {
          parsedChannel = parseChannel({
            messageEncoding: channel.encoding, // "cdr"
            schema: {
              name: channel.schemaName,
              encoding: schemaEncoding,
              data: schemaData,
            },
          });
        } catch (err) {
          console.error(
            "[FoxgloveWsClient] Failed to parse channel schema for",
            channel.topic,
            err,
          );
          continue;
        }

        const resolved: ResolvedChannel = { channel, parsedChannel };
        this.channelsById.set(channel.id, resolved);
        this.channelsByTopic.set(channel.topic, resolved);

        this.onNewTopic?.(channel.topic, channel.schemaName);
      }

      // Any pending subscriptions might now be resolvable
      this.processPendingSubscriptions();
    });

    this.client.on("unadvertise", (removedIds: ChannelId[]) => {
      for (const id of removedIds) {
        const chanInfo = this.channelsById.get(id);
        if (!chanInfo) {
          continue;
        }
        this.channelsById.delete(id);
        this.channelsByTopic.delete(chanInfo.channel.topic);
      }
    });

    // Actual message data (decode using parsedChannel.deserialize)
    this.client.on("message", ({ subscriptionId, data }) => {
      const chanInfo = this.subscriptionsById.get(subscriptionId);
      if (!chanInfo) {
        console.warn(
          "[FoxgloveWsClient] Message on unknown subscription",
          subscriptionId,
        );
        return;
      }

      try {
        const decoded = chanInfo.parsedChannel.deserialize(data);
        const { topic, schemaName, encoding } = chanInfo.channel;

        this.onMessage?.({
          topic,
          schemaName,
          encoding,
          payload: decoded, // <── fully decoded JS object
        });
      } catch (err) {
        console.error(
          "[FoxgloveWsClient] Failed to decode message on topic",
          chanInfo.channel.topic,
          err,
        );
      }
    });
  }

  private processPendingSubscriptions() {
    if (!this.isOpenFlag) {
      return;
    }
    for (const topic of Array.from(this.pendingSubscriptions)) {
      const chanInfo = this.channelsByTopic.get(topic);
      if (!chanInfo) {
        continue;
      }
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

  // Optional publish stub — you can flesh this out later if you want CDR *encoding* as well.
  public publish(_topic: string, _schemaName: string, _msg: any) {
    console.warn(
      "[FoxgloveWsClient] publish() not implemented (encoding side not wired)",
    );
  }
}
