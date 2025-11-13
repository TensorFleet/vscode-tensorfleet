// Ported from: lichtblick/packages/suite-base/src/panels/RawMessages/index.tsx
// Date: 2025-11-07
// Refactored: 2025-01-XX
// - Simplified to show only latest message, no caching or pause

import React, { type ChangeEvent, useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { Theme } from "react-base16-styling";
import { JSONTree } from "react-json-tree";
import { Subscription, ros2Bridge, type ImageMessage } from "../../ros2-bridge";
import { getTopicSuggestions, type TopicSuggestion } from "../../utils/topicSuggestions";
import "./RawMessagesPanel.css";

type RawPayloadMessage = {
  topic: string;
  type: string;
  msg?: unknown;
  payload?: unknown;
};

type ReceivedMessage = {
  id: number;
  receivedAt: number; // timestamp in ms
  payload: RawPayloadMessage;
};

const MESSAGE_RATE_WINDOW_MS = 1000; // Calculate rate over 1 second

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


function formatPreview(value: unknown): string {
  if (value == undefined) {
    return "Payload unavailable";
  }
  if (typeof value === "object") {
    try {
      const serialized = JSON.stringify(value, null, 2);
      return serialized.length > 200 ? `${serialized.slice(0, 197)}…` : serialized;
    } catch {
      return "[object]";
    }
  }
  const asText = String(value);
  return asText.length > 200 ? `${asText.slice(0, 197)}…` : asText;
}

function formatRelativeTime(timestamp: number): string {
  const now = Date.now();
  const diff = now - timestamp;
  
  if (diff < 1000) {
    return "just now";
  }
  if (diff < 60000) {
    const seconds = Math.floor(diff / 1000);
    return `${seconds}s ago`;
  }
  if (diff < 3600000) {
    const minutes = Math.floor(diff / 60000);
    return `${minutes}m ago`;
  }
  const date = new Date(timestamp);
  return date.toLocaleTimeString();
}

function formatAbsoluteTime(timestamp: number): string {
  const date = new Date(timestamp);
  return date.toLocaleTimeString();
}

function getTransformCount(payload: unknown): number {
  if (!payload || typeof payload !== "object") {
    return 0;
  }
  const asRecord = payload as Record<string, unknown>;
  if (Array.isArray(asRecord.transforms)) {
    return asRecord.transforms.length;
  }
  if (Array.isArray(asRecord.transform)) {
    return asRecord.transform.length;
  }
  return 0;
}

function isImageMessage(body: unknown): boolean {
  if (!body || typeof body !== "object") {
    return false;
  }
  const asRecord = body as Record<string, unknown>;
  // Check if it has image-like properties
  return (
    typeof asRecord.width === "number" &&
    typeof asRecord.height === "number" &&
    typeof asRecord.encoding === "string" &&
    (typeof asRecord.data === "string" || Array.isArray(asRecord.data))
  );
}

function getImageDataUri(body: unknown): string | null {
  if (!body || typeof body !== "object") {
    return null;
  }
  const asRecord = body as Record<string, unknown>;
  const data = asRecord.data;
  
  // If it's already a data URI (starts with "data:")
  if (typeof data === "string" && data.startsWith("data:")) {
    return data;
  }
  
  // If it's base64 encoded, construct data URI
  if (typeof data === "string") {
    // Determine encoding from the message
    const encoding = asRecord.encoding as string || "rgb8";
    let mimeType = "image/jpeg"; // default
    
    // Try to detect from data URI if present
    if (data.includes("data:image/")) {
      return data;
    }
    
    // For now, assume JPEG if it's a long base64 string
    return `data:image/jpeg;base64,${data}`;
  }
  
  return null;
}

export function RawMessagesPanel() {
  const [selectedTopic, setSelectedTopic] = useState("/clock");
  const [currentMessage, setCurrentMessage] = useState<ReceivedMessage | null>(null);
  const [activeTopic, setActiveTopic] = useState("");
  const [isSubscribed, setIsSubscribed] = useState(false);
  const [isConnected, setIsConnected] = useState(() => ros2Bridge.isConnected());
  const [isHeaderExpanded, setIsHeaderExpanded] = useState(true);
  const [messageRate, setMessageRate] = useState(0);
  const [lastMessageTime, setLastMessageTime] = useState<number>(0);

  const cleanupRef = useRef<(() => void) | null>(null);
  const rateIntervalRef = useRef<number | null>(null);
  const messageCountRef = useRef(0);
  const messageTimesRef = useRef<number[]>([]); // Track message timestamps for rate calculation

  const topicSuggestions = useMemo(() => getTopicSuggestions(), []);
  const selectedSuggestion = useMemo(() => {
    return topicSuggestions.find((s) => s.topic === selectedTopic);
  }, [selectedTopic, topicSuggestions]);
  
  // Calculate message rate over the last second
  useEffect(() => {
    if (!isSubscribed) {
      setMessageRate(0);
      messageTimesRef.current = [];
      if (rateIntervalRef.current) {
        clearInterval(rateIntervalRef.current);
        rateIntervalRef.current = null;
      }
      return;
    }
    
    // Calculate rate every 500ms based on messages in the last second
    const updateRate = () => {
      const now = Date.now();
      const windowStart = now - MESSAGE_RATE_WINDOW_MS;
      
      // Remove old timestamps outside the window
      messageTimesRef.current = messageTimesRef.current.filter(time => time > windowStart);
      
      // Count messages in the last second
      setMessageRate(messageTimesRef.current.length);
    };
    
    updateRate();
    
    if (!rateIntervalRef.current) {
      rateIntervalRef.current = window.setInterval(updateRate, 500);
    }
    
    return () => {
      if (rateIntervalRef.current) {
        clearInterval(rateIntervalRef.current);
        rateIntervalRef.current = null;
      }
    };
  }, [isSubscribed]);

  const handleMessage = useCallback((message: any) => {
    // Handle both RawPayloadMessage format and ImageMessage format
    let payload: RawPayloadMessage;
    
    // Check if it's an ImageMessage (from image topics)
    if (message.topic && message.data && message.encoding !== undefined) {
      const imageMsg = message as ImageMessage;
      console.log('[RawMessages] Received ImageMessage:', {
        topic: imageMsg.topic,
        encoding: imageMsg.encoding,
        width: imageMsg.width,
        height: imageMsg.height,
      });
      // Convert ImageMessage to RawPayloadMessage format
      payload = {
        topic: imageMsg.topic,
        type: "sensor_msgs/msg/Image",
        msg: {
          header: {
            stamp: {
              sec: imageMsg.timestampNanos ? Math.floor(imageMsg.timestampNanos / 1_000_000_000) : 0,
              nanosec: imageMsg.timestampNanos ? imageMsg.timestampNanos % 1_000_000_000 : 0,
            },
            frame_id: imageMsg.frameId,
          },
          height: imageMsg.height,
          width: imageMsg.width,
          encoding: imageMsg.encoding,
          is_bigendian: 0,
          step: imageMsg.width * (imageMsg.encoding.includes('16') ? 2 : 1),
          data: imageMsg.data, // This is the data URI (base64)
        },
      };
    } else {
      // Regular message format
      console.log('[RawMessages] Received regular message:', {
        topic: message.topic || message.payload?.topic,
        type: message.type || message.payload?.type,
        hasMsg: !!message.msg,
        hasPayload: !!message.payload,
      });
      payload = {
        topic: message.topic || message.payload?.topic || "",
        type: message.type || message.payload?.type || "unknown",
        msg: message.msg ?? message.payload?.msg ?? message.payload ?? message,
        payload: message.payload ?? message,
      };
    }
    
    const entry: ReceivedMessage = {
      id: messageCountRef.current++,
      receivedAt: Date.now(),
      payload,
    };
    
    const now = Date.now();
    setCurrentMessage(entry);
    setLastMessageTime(now);
    // Track message time for rate calculation
    messageTimesRef.current.push(now);
    // Keep only last 100 timestamps to prevent memory issues
    if (messageTimesRef.current.length > 100) {
      messageTimesRef.current.shift();
    }
  }, []);

  const handleSubscribe = useCallback(() => {
    const cleanedTopic = selectedTopic.trim();
    if (cleanedTopic.length === 0) {
      return;
    }
    
    cleanupRef.current?.();
    setCurrentMessage(null);
    setMessageRate(0);
    setLastMessageTime(0);
    messageCountRef.current = 0;
    messageTimesRef.current = [];
    
    const suggestion = getTopicSuggestions().find((s) => s.topic === cleanedTopic);
    const subscription: Subscription = {
      topic: cleanedTopic,
      type: suggestion?.type || "unknown",
    };
    
    console.log('[RawMessages] Subscribing to topic:', cleanedTopic, 'type:', subscription.type);
    
    cleanupRef.current = ros2Bridge.subscribe(subscription, (message) => {
      console.log('[RawMessages] Message received for topic:', cleanedTopic, 'message keys:', Object.keys(message));
      handleMessage(message);
    });
    
    setActiveTopic(cleanedTopic);
    setIsSubscribed(true);
  }, [selectedTopic, handleMessage]);

  const handleUnsubscribe = useCallback(() => {
    cleanupRef.current?.();
    cleanupRef.current = null;
    setIsSubscribed(false);
    setActiveTopic("");
    setCurrentMessage(null);
    setMessageRate(0);
    setLastMessageTime(0);
    messageCountRef.current = 0;
    messageTimesRef.current = [];
  }, []);

  const handleCopyMessage = useCallback(async (message: ReceivedMessage) => {
    try {
      const body = message.payload.msg ?? message.payload.payload ?? {};
      const jsonString = JSON.stringify(body, null, 2);
      await navigator.clipboard.writeText(jsonString);
      
      // Show temporary feedback
      const button = document.querySelector(`[data-message-id="${message.id}"]`) as HTMLElement;
      if (button) {
        const originalText = button.textContent;
        button.textContent = "✓ Copied!";
        button.style.color = "#4ade80";
        setTimeout(() => {
          button.textContent = originalText;
          button.style.color = "";
        }, 2000);
      }
    } catch (error) {
      console.error("Failed to copy message:", error);
      // Fallback for older browsers
      const textArea = document.createElement("textarea");
      const body = message.payload.msg ?? message.payload.payload ?? {};
      textArea.value = JSON.stringify(body, null, 2);
      textArea.style.position = "fixed";
      textArea.style.opacity = "0";
      document.body.appendChild(textArea);
      textArea.select();
      try {
        document.execCommand("copy");
        const button = document.querySelector(`[data-message-id="${message.id}"]`) as HTMLElement;
        if (button) {
          const originalText = button.textContent;
          button.textContent = "✓ Copied!";
          button.style.color = "#4ade80";
          setTimeout(() => {
            button.textContent = originalText;
            button.style.color = "";
          }, 2000);
        }
      } catch (err) {
        console.error("Fallback copy failed:", err);
      }
      document.body.removeChild(textArea);
    }
  }, []);


  const handleTopicDropdownChange = useCallback((event: ChangeEvent<HTMLSelectElement>) => {
    const topic = event.target.value;
    if (topic) {
      setSelectedTopic(topic);
    }
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

  return (
    <div className="raw-messages-root">
      <header
        className={`raw-messages-header ${isHeaderExpanded ? "raw-messages-header--expanded" : "raw-messages-header--collapsed"}`}
      >
        <div className="raw-messages-header__top">
          <div className="raw-messages-header__title-section">
            <div className="raw-messages-title">Raw Messages</div>
            <div className="raw-messages-header__status-row">
              <span className={`connection-dot ${isConnected ? "connected" : "disconnected"}`} />
              <span className="raw-messages-header__status-text">
                {isConnected ? "Connected" : "Disconnected"}
              </span>
              {isSubscribed && (
                <>
                  <span className="raw-messages-header__separator">•</span>
                  <span className="raw-messages-header__live-indicator">
                    <span className="raw-messages-header__live-dot" />
                    Live
                  </span>
                  <span className="raw-messages-header__rate">{messageRate} msg/s</span>
                </>
              )}
            </div>
          </div>
          <div className="raw-messages-header__actions">
            {isSubscribed && (
              <button
                className="raw-messages-header__control-button raw-messages-header__control-button--danger"
                type="button"
                onClick={handleUnsubscribe}
              >
                Unsubscribe
              </button>
            )}
            <button
              className="raw-messages-header__toggle"
              type="button"
              onClick={() => setIsHeaderExpanded(!isHeaderExpanded)}
              aria-expanded={isHeaderExpanded}
            >
              {isHeaderExpanded ? "Hide controls ▴" : "Show controls ▾"}
            </button>
          </div>
        </div>
        
        {isHeaderExpanded && (
          <div className="raw-messages-header__controls">
            <div className="raw-messages-header__info">
              <div className="raw-messages-header__info-item">
                <span className="raw-messages-header__info-label">Topic</span>
                <strong className="raw-messages-header__info-value">
                  {isSubscribed ? activeTopic : selectedTopic || "Not selected"}
                </strong>
              </div>
              {selectedSuggestion && (
                <div className="raw-messages-header__info-item">
                  <span className="raw-messages-header__info-label">Type</span>
                  <span className="raw-messages-header__info-value">{selectedSuggestion.type}</span>
                </div>
              )}
            </div>
            
            <div className="raw-messages-header__form">
              <label className="raw-messages-header__form-group">
                <span className="raw-messages-header__form-label">Select Topic</span>
                <select
                  className="raw-messages-header__select"
                  value={selectedTopic}
                  onChange={handleTopicDropdownChange}
                  disabled={isSubscribed}
                >
                  <option value="">Choose a topic...</option>
                  {topicSuggestions.map((suggestion) => (
                    <option key={suggestion.topic} value={suggestion.topic}>
                      {suggestion.topic} — {suggestion.type}
                    </option>
                  ))}
                </select>
                {selectedSuggestion?.description && (
                  <span className="raw-messages-header__hint">{selectedSuggestion.description}</span>
                )}
              </label>
              
              {!isSubscribed && (
                <button
                  className="raw-messages-header__subscribe-button"
                  type="button"
                  onClick={handleSubscribe}
                  disabled={!selectedTopic || !isConnected}
                >
                  <span className="raw-messages-header__subscribe-icon">📡</span>
                  Subscribe
                </button>
              )}
            </div>
          </div>
        )}
      </header>

      <section className="raw-messages-content">
        <div className="raw-messages-content__toolbar">
          <div className="raw-messages-content__latest">
            <span className="raw-messages-content__label">Topic</span>
            <strong>{activeTopic || "—"}</strong>
            {isSubscribed && currentMessage && (
              <>
                {messageRate > 0 && (
                  <>
                    <span className="raw-messages-content__separator">•</span>
                    <span className="raw-messages-content__detail">{messageRate} msg/s</span>
                  </>
                )}
              </>
            )}
          </div>
        </div>
        
        <div className="raw-messages-content__main">
          {!currentMessage ? (
            <div className="raw-messages-empty">
              {isSubscribed ? "Waiting for messages..." : "Subscribe to a topic to see messages"}
            </div>
          ) : (
            <>
              {/* Single large message card */}
              {(() => {
                const body = currentMessage.payload.msg ?? currentMessage.payload.payload ?? {};
                const hasObjectPayload = body && typeof body === "object";
                const transformCount = getTransformCount(body);
                const isImage = isImageMessage(body);
                const imageDataUri = isImage ? getImageDataUri(body) : null;
                const imageBody = body as Record<string, unknown>;
                
                return (
                  <article className="raw-message-card raw-message-card--large">
                    <header className="raw-message-card__header">
                      <div className="raw-message-card__topic-row">
                        <span className="raw-message-card__topic-icon" aria-hidden="true">
                          {isImage ? "🖼️" : "🔷"}
                        </span>
                        <div className="raw-message-card__topic-info">
                          <div className="raw-message-card__topic">{currentMessage.payload.topic}</div>
                          <div className="raw-message-card__meta">
                            <span>{currentMessage.payload.type}</span>
                            {isImage && (
                              <>
                                <span className="raw-message-card__image-info">
                                  {imageBody.width}×{imageBody.height} • {imageBody.encoding}
                                </span>
                              </>
                            )}
                            <span className="raw-message-card__timestamp">
                              {formatRelativeTime(currentMessage.receivedAt)}
                            </span>
                            <span className="raw-message-card__absolute-time">
                              {formatAbsoluteTime(currentMessage.receivedAt)}
                            </span>
                          </div>
                        </div>
                      </div>
                      <div className="raw-message-card__actions">
                        {transformCount > 0 && (
                          <span className="raw-message-card__badge">{transformCount} transforms</span>
                        )}
                        <button
                          className="raw-message-card__copy-button"
                          type="button"
                          onClick={() => handleCopyMessage(currentMessage)}
                          data-message-id={currentMessage.id}
                          title="Copy message as JSON"
                        >
                          📋 Copy
                        </button>
                      </div>
                    </header>
                    <div className="raw-message-card__details">
                      {isImage && imageDataUri ? (
                        <div className="raw-message-card__image-container">
                          <img 
                            src={imageDataUri} 
                            alt={`${currentMessage.payload.topic} image`}
                            className="raw-message-card__image"
                          />
                          <div className="raw-message-card__image-metadata">
                            <div className="raw-message-card__image-metadata-item">
                              <span className="raw-message-card__image-metadata-label">Dimensions:</span>
                              <span>{imageBody.width} × {imageBody.height} pixels</span>
                            </div>
                            <div className="raw-message-card__image-metadata-item">
                              <span className="raw-message-card__image-metadata-label">Encoding:</span>
                              <span>{imageBody.encoding}</span>
                            </div>
                            {imageBody.header && typeof imageBody.header === "object" && (
                              <div className="raw-message-card__image-metadata-item">
                                <span className="raw-message-card__image-metadata-label">Frame ID:</span>
                                <span>{(imageBody.header as Record<string, unknown>).frame_id as string || "N/A"}</span>
                              </div>
                            )}
                          </div>
                          <details className="raw-message-card__image-raw-data">
                            <summary className="raw-message-card__image-raw-data-summary">
                              Show raw message data
                            </summary>
                            <div className="raw-message-card__image-raw-data-content">
                              {hasObjectPayload ? (
                                <JSONTree 
                                  data={body as object} 
                                  theme={jsonTreeTheme} 
                                  invertTheme={false} 
                                  hideRoot 
                                  shouldExpandNode={() => false}
                                />
                              ) : (
                                <pre className="raw-message-card__plaintext">{String(body)}</pre>
                              )}
                            </div>
                          </details>
                        </div>
                      ) : hasObjectPayload ? (
                        <JSONTree data={body as object} theme={jsonTreeTheme} invertTheme={false} hideRoot />
                      ) : (
                        <pre className="raw-message-card__plaintext">{String(body)}</pre>
                      )}
                    </div>
                  </article>
                );
              })()}
            </>
          )}
        </div>
      </section>
    </div>
  );
}

export default RawMessagesPanel;
