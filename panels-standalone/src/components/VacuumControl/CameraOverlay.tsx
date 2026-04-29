import React, { useCallback, useEffect, useRef, useState } from "react";
import { ros2Bridge } from "../../ros2-bridge";
import type { ImageMessage } from "tensorfleet-util/ros/ros-types";

const PREFERRED_TOPICS = [
  "/oakd/rgb/preview/image_raw",
  "/oakd/rgb/preview/image/compressed",
  "/oakd/rgb/image_raw",
  "/camera/color/image_raw",
  "/camera/image_raw",
  "/image_raw",
];

const IMAGE_TYPES = new Set([
  "sensor_msgs/msg/Image",
  "sensor_msgs/msg/CompressedImage",
  "sensor_msgs/Image",
  "sensor_msgs/CompressedImage",
]);

type DiscoveredSub = { topic: string; type: string };

function pickBestSub(topics: DiscoveredSub[]): DiscoveredSub | null {
  const imageSubs = topics.filter((t) => IMAGE_TYPES.has(t.type));
  for (const pref of PREFERRED_TOPICS) {
    const match = imageSubs.find((t) => t.topic === pref);
    if (match) return match;
  }
  return imageSubs[0] ?? null;
}

type DragState = { mouseX: number; mouseY: number; posX: number; posY: number };

export function CameraOverlay(): React.JSX.Element {
  const [isMinimized, setIsMinimized] = useState(false);
  const [isVisible, setIsVisible] = useState(true);
  const [activeSub, setActiveSub] = useState<DiscoveredSub | null>(null);
  const [frame, setFrame] = useState<ImageMessage | null>(null);
  const [pos, setPos] = useState({ x: 16, y: 16 });
  const [dragging, setDragging] = useState(false);

  const dragStartRef = useRef<DragState | null>(null);

  // Topic discovery – runs until a topic is found, then keeps checking
  // so that topic switches (e.g. reconnect) are reflected.
  useEffect(() => {
    const discover = () => {
      const topics = ros2Bridge.getAvailableImageTopics();
      const best = pickBestSub(topics);
      setActiveSub((prev) => {
        if (!best) return prev;
        if (prev?.topic === best.topic && prev.type === best.type) return prev;
        return best;
      });
    };
    discover();
    const id = setInterval(discover, 2000);
    return () => clearInterval(id);
  }, []);

  // Image subscription – subscribe() requires { topic, type } to wire Foxglove
  useEffect(() => {
    if (!activeSub) return;

    const handler = (msg: unknown) => {
      const imageMsg = msg as ImageMessage;
      if (typeof imageMsg.data === "string" && imageMsg.data.length > 0) {
        setFrame(imageMsg);
      }
    };

    // subscribe() returns an unsubscribe function, but we also need the topic
    // string for unsubscribe() – keep both paths.
    ros2Bridge.subscribe(activeSub, handler);
    return () => {
      ros2Bridge.unsubscribe(activeSub.topic, handler);
      setFrame(null);
    };
  }, [activeSub]);

  // Drag – use pointer events to match the map stage's pointer model.
  // stopPropagation on the overlay root prevents the map's onPointerDown
  // from registering a destination placement while we drag the window.
  const handleDragStart = useCallback(
    (e: React.PointerEvent) => {
      e.preventDefault();
      e.stopPropagation();
      (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
      dragStartRef.current = { mouseX: e.clientX, mouseY: e.clientY, posX: pos.x, posY: pos.y };
      setDragging(true);
    },
    [pos],
  );

  const handleDragMove = useCallback((e: React.PointerEvent) => {
    if (!dragging) return;
    const start = dragStartRef.current;
    if (!start) return;
    setPos({ x: start.posX + e.clientX - start.mouseX, y: start.posY + e.clientY - start.mouseY });
  }, [dragging]);

  const handleDragEnd = useCallback(() => {
    setDragging(false);
  }, []);

  const topicShortName = activeSub?.topic.split("/").filter(Boolean).pop() ?? "Camera";

  // Block all pointer events on the overlay from reaching the map stage.
  const stopProp = (e: React.PointerEvent | React.MouseEvent) => { e.stopPropagation(); };

  if (!isVisible) {
    return (
      <button
        className="vacuum-camera-restore"
        type="button"
        title="Show camera feed"
        onClick={() => { setIsVisible(true); }}
        onPointerDown={stopProp}
        style={{ transform: `translate(${pos.x}px, ${pos.y}px)` }}
      >
        <svg viewBox="0 0 20 20" aria-hidden="true">
          <path d="M2 6A1.5 1.5 0 0 1 3.5 4.5h9A1.5 1.5 0 0 1 14 6v8A1.5 1.5 0 0 1 12.5 15.5h-9A1.5 1.5 0 0 1 2 14z" />
          <path d="M14 7.5 18 5.5v9l-4-2z" />
        </svg>
        <span>Camera</span>
      </button>
    );
  }

  return (
    <div
      className={[
        "vacuum-camera-overlay",
        dragging ? "vacuum-camera-overlay--dragging" : "",
        isMinimized ? "vacuum-camera-overlay--minimized" : "",
      ].filter(Boolean).join(" ")}
      style={{ transform: `translate(${pos.x}px, ${pos.y}px)` }}
      aria-label="Camera feed"
      onPointerDown={stopProp}
    >
      {/* Title bar – drag handle */}
      <div
        className="vacuum-camera-titlebar"
        onPointerDown={handleDragStart}
        onPointerMove={handleDragMove}
        onPointerUp={handleDragEnd}
        onPointerCancel={handleDragEnd}
        role="toolbar"
        aria-label="Camera window controls"
      >
        <span className="vacuum-camera-title">
          <svg viewBox="0 0 16 16" aria-hidden="true">
            <path d="M1 4.5A1 1 0 0 1 2 3.5h8a1 1 0 0 1 1 1v7a1 1 0 0 1-1 1H2a1 1 0 0 1-1-1z" />
            <path d="M11 6.5 15 4.5v7l-4-2z" />
          </svg>
          {topicShortName}
        </span>

        <div className="vacuum-camera-controls">
          <button
            className="vacuum-camera-btn"
            type="button"
            title={isMinimized ? "Expand camera" : "Minimize camera"}
            onClick={() => { setIsMinimized((v) => !v); }}
            aria-label={isMinimized ? "Expand camera" : "Minimize camera"}
            onPointerDown={(e) => { e.stopPropagation(); }}
          >
            <svg viewBox="0 0 12 12" aria-hidden="true">
              <path d={isMinimized ? "M2 8h8" : "M2 4h8"} />
            </svg>
          </button>

          <button
            className="vacuum-camera-btn vacuum-camera-btn--close"
            type="button"
            title="Hide camera"
            onClick={() => { setIsVisible(false); }}
            aria-label="Hide camera"
            onPointerDown={(e) => { e.stopPropagation(); }}
          >
            <svg viewBox="0 0 12 12" aria-hidden="true">
              <path d="M2 2l8 8M10 2 2 10" />
            </svg>
          </button>
        </div>
      </div>

      {!isMinimized && (
        <div className="vacuum-camera-frame">
          {frame?.data ? (
            <img
              className="vacuum-camera-img"
              src={frame.data}
              alt="Robot camera feed"
              draggable={false}
            />
          ) : (
            <div className="vacuum-camera-placeholder">
              <svg viewBox="0 0 40 40" aria-hidden="true">
                <rect x="3" y="8" width="25" height="18" rx="2" />
                <path d="M28 14l9-4v20l-9-4z" />
                <circle cx="15.5" cy="17" r="4.5" />
              </svg>
              <span>{activeSub ? "Waiting for camera…" : "No camera topic found"}</span>
            </div>
          )}
        </div>
      )}
    </div>
  );
}
