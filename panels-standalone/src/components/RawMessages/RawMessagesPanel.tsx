// Ported from: lichtblick/packages/suite-base/src/panels/RawMessages/index.tsx
// Date: 2025-11-07
// Modifications:
// - Removed PanelAPI/framework integrations and replaced with a standalone ROS2 bridge subscription flow.
// - Added topic/type inputs, connection status indicator, and lightweight JSON rendering via react-json-tree.
// - Dropped diff/toolbar/setting support to keep the first iteration focused on showing raw payloads.

import React, { useCallback, useEffect, useRef, useState } from "react";
import type { Theme } from "react-base16-styling";
import { JSONTree } from "react-json-tree";
import { Subscription, ros2Bridge } from "../../ros2-bridge";
import "./RawMessagesPanel.css";

type RawPayloadMessage = {
  topic: string;
  type: string;
  msg?: unknown;
  payload?: unknown;
};

type ReceivedMessage = {
  id: number;
  receivedAt: string;
  payload: RawPayloadMessage;
};

const jsonTreeTheme: Theme = {
  scheme: "tensorfleet-dark",
  author: "Tensorfleet",
  base00: "#050608",
  base01: "#111318",
  base02: "#1c1f26",
  base03: "#2f333b",
  base04: "#6b7280",
  base05: "#cbd5ef",
  base06: "#f1f5f9",
  base07: "#ffffff",
  base08: "#f38ba0",
  base09: "#f9d000",
  base0A: "#f7c948",
  base0B: "#5fb9a2",
  base0C: "#5f9df9",
  base0D: "#75bdfc",
  base0E: "#c678dd",
  base0F: "#7f8493",
};

export function RawMessagesPanel() {
  const [topicInput, setTopicInput] = useState("/tf");
  const [typeInput, setTypeInput] = useState("std_msgs/msg/String");
  const [messages, setMessages] = useState<ReceivedMessage[]>([]);
  const [activeTopic, setActiveTopic] = useState<string>("");
  const [isSubscribed, setIsSubscribed] = useState(false);
  const [isConnected, setIsConnected] = useState(() => ros2Bridge.isConnected());
  const cleanupRef = useRef<(() => void) | null>(null);
  const nextId = useRef(1);

  const handleMessage = useCallback((payload: RawPayloadMessage) => {
    const entry: ReceivedMessage = {
      id: nextId.current++,
      receivedAt: new Date().toISOString(),
      payload,
    };
    setMessages((prev) => [entry, ...prev].slice(0, 25));
  }, []);

  const handleSubscribe = useCallback(() => {
    const cleanedTopic = topicInput.trim();
    if (cleanedTopic.length === 0) {
      return;
    }

    cleanupRef.current?.();
    setMessages([]);

    const subscription: Subscription = {
      topic: cleanedTopic,
      type: typeInput.trim() || "unknown",
    };

    const cleanup = ros2Bridge.subscribe(subscription, (payload) => {
      handleMessage(payload);
    });

    cleanupRef.current = cleanup;
    setActiveTopic(cleanedTopic);
    setIsSubscribed(true);
  }, [topicInput, typeInput, handleMessage]);

  const handleUnsubscribe = useCallback(() => {
    cleanupRef.current?.();
    cleanupRef.current = null;
    setIsSubscribed(false);
    setActiveTopic("");
  }, []);

  useEffect(() => {
    const id = window.setInterval(() => {
      setIsConnected(ros2Bridge.isConnected());
    }, 1000);
    return () => {
      clearInterval(id);
      cleanupRef.current?.();
    };
  }, []);

  const connectionClass = isConnected ? "connection-status connected" : "connection-status";

  return (
    <div className="raw-messages-root">
      <div className="raw-messages-header">
        <div>
          <div className="raw-messages-title">Raw Messages</div>
          <div className={connectionClass}>{isConnected ? "Connected" : "Disconnected"}</div>
        </div>
        <div className="connection-status">
          {activeTopic ? `Subscribed to ${activeTopic}` : "Subscribe to a topic to start receiving payloads"}
        </div>
      </div>

      <div className="raw-messages-controls">
        <label>
          <div className="form-inline-label">Topic</div>
          <input
            type="text"
            placeholder="/tf"
            value={topicInput}
            onChange={(event) => setTopicInput(event.target.value)}
          />
        </label>
        <label>
          <div className="form-inline-label">Message type</div>
          <input
            type="text"
            placeholder="std_msgs/msg/String"
            value={typeInput}
            onChange={(event) => setTypeInput(event.target.value)}
          />
        </label>
        <button type="button" onClick={isSubscribed ? handleUnsubscribe : handleSubscribe}>
          {isSubscribed ? "Unsubscribe" : "Subscribe"}
        </button>
      </div>

      <div className="raw-messages-list" data-testid="raw-messages-list">
        {messages.length === 0 ? (
          <div className="raw-messages-empty">No messages captured yet</div>
        ) : (
          messages.map((message) => {
            const body = message.payload.msg ?? message.payload.payload ?? {};
            const hasObjectPayload = body && typeof body === "object";
            return (
              <div className="raw-message-card" key={message.id}>
                <div className="raw-message-meta">
                  <span>{message.payload.topic}</span>
                  <span>{message.payload.type}</span>
                  <span>{new Date(message.receivedAt).toLocaleTimeString()}</span>
                </div>
                {hasObjectPayload ? (
                  <JSONTree data={body} theme={jsonTreeTheme} invertTheme={false} hideRoot />
                ) : (
                  <pre className="raw-message-pre">{String(body)}</pre>
                )}
              </div>
            );
          })
        )}
      </div>
    </div>
  );
}

export default RawMessagesPanel;
