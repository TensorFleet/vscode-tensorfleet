// MissionControlBridge.tsx
import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { DroneController } from "@/mission-control/drone-controller";

/** Keep this in sync with the UI emitter */
type MissionAction = "arm" | "disarm" | "takeoff" | "land" | "rtl" | "goto";

interface AppRequestDetail<T = any> {
  category: "mission_control";
  action: MissionAction;
  payload?: T;
  requestId: string;
  timestamp: number;
}

const EVENT_NAME = "app:request";

// ---------- Toasts ----------

type ToastKind = "success" | "error" | "info";
type Toast = {
  id: string;
  kind: ToastKind;
  title: string;
  description?: string;
  createdAt: number;
  closing?: boolean;
};

const toastStyles: Record<string, React.CSSProperties> = {
  wrap: {
    position: "fixed",
    left: "50%",
    bottom: 24,
    transform: "translateX(-50%)",
    display: "flex",
    flexDirection: "column",
    gap: 8,
    zIndex: 9999,
    pointerEvents: "none",
  },
  itemBase: {
    minWidth: 280,
    maxWidth: 520,
    padding: "10px 12px",
    borderRadius: 10,
    boxShadow: "0 6px 20px rgba(0,0,0,0.12)",
    border: "1px solid #e5e7eb",
    background: "white",
    color: "#111827",
    fontFamily:
      "ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial",
    pointerEvents: "auto",
    transition: "opacity 300ms ease, transform 300ms ease",
    opacity: 1,
    transform: "translateY(0px)",
  },
  itemClosing: {
    opacity: 0,
    transform: "translateY(8px)",
  },
  row: { display: "flex", alignItems: "start", gap: 8 },
  dot: (kind: ToastKind): React.CSSProperties => ({
    flex: "0 0 8px",
    width: 8,
    height: 8,
    marginTop: 6,
    borderRadius: 999,
    background:
      kind === "success" ? "#10b981" : kind === "error" ? "#ef4444" : "#3b82f6",
  }),
  content: { display: "grid", gap: 2 },
  title: { fontSize: 14, fontWeight: 600, lineHeight: 1.2 },
  desc: { fontSize: 12, color: "#4b5563", whiteSpace: "pre-wrap" },
  srLive: {
    position: "absolute",
    width: 1,
    height: 1,
    padding: 0,
    margin: -1,
    overflow: "hidden",
    clip: "rect(0,0,0,0)",
    whiteSpace: "nowrap",
    border: 0,
  },
};

function ToastHost({ toasts }: { toasts: Toast[] }) {
  return (
    <div style={toastStyles.wrap} aria-live="polite" aria-atomic>
      {/* Screen-reader live region */}
      <div style={toastStyles.srLive}>
        {toasts.length ? toasts[toasts.length - 1].title : ""}
      </div>

      {toasts.map((t) => {
        const style = {
          ...toastStyles.itemBase,
          ...(t.closing ? toastStyles.itemClosing : null),
        };
        return (
          <div key={t.id} style={style} role="status">
            <div style={toastStyles.row}>
              <div style={toastStyles.dot(t.kind)} />
              <div style={toastStyles.content}>
                <div style={toastStyles.title}>{t.title}</div>
                {t.description ? (
                  <div style={toastStyles.desc}>{t.description}</div>
                ) : null}
              </div>
            </div>
          </div>
        );
      })}
    </div>
  );
}

// ---------- Bridge ----------

export interface MissionControlBridgeProps {
  /** A ready DroneController instance from "@/mission-control/drone-controller". */
  controller: DroneController;
  /** Optional: how long to show toast messages (ms). Default: 6000 */
  toastMs?: number;
}

/**
 * Listens for "app:request" (mission_control) and forwards actions to DroneController.
 * Shows bottom-center toasts for each forwarded command (or any error).
 *
 * Usage:
 *   <MissionControlBridge controller={controllerInstance} />
 */
export default function MissionControlBridge({
  controller,
  toastMs = 6000,
}: MissionControlBridgeProps) {
  const [toasts, setToasts] = useState<Toast[]>([]);
  const timers = useRef<Record<string, number[]>>({}); // id -> [closeTimer, removeTimer]

  const pushToast = useCallback(
    (partial: Omit<Toast, "createdAt" | "closing">) => {
      const t: Toast = { ...partial, createdAt: Date.now() };
      setToasts((arr) => [...arr, t]);

      // schedule fade-out ~300ms before removal
      const closeTimer = window.setTimeout(() => {
        setToasts((arr) =>
          arr.map((x) => (x.id === t.id ? { ...x, closing: true } : x))
        );
      }, Math.max(0, toastMs - 300));

      const removeTimer = window.setTimeout(() => {
        setToasts((arr) => arr.filter((x) => x.id !== t.id));
        // cleanup timer registry
        delete timers.current[t.id];
      }, toastMs);

      timers.current[t.id] = [closeTimer, removeTimer];
    },
    [toastMs]
  );

  // Cleanup timers on unmount
  useEffect(() => {
    return () => {
      Object.values(timers.current).forEach(([a, b]) => {
        window.clearTimeout(a);
        window.clearTimeout(b);
      });
      timers.current = {};
    };
  }, []);

  const forward = useCallback(
    async (detail: AppRequestDetail) => {
      const id = detail.requestId;
      const prettyPayload =
        detail.payload == null ? "" : `\nPayload: ${JSON.stringify(detail.payload)}`;

      const ok = (msg: string, extra?: string) =>
        pushToast({
          id,
          kind: "success",
          title: msg,
          description:
            `Forwarded "${detail.action}" (#${id.slice(0, 6)}).` +
            (extra ? `\n${extra}` : "") +
            prettyPayload,
        });

      const fail = (err: unknown) =>
        pushToast({
          id,
          kind: "error",
          title: `Failed to forward "${detail.action}"`,
          description:
            (err instanceof Error ? err.message : String(err)) + prettyPayload,
        });

      try {
        switch (detail.action) {
          case "arm": {
            await controller.arm();
            ok("Arming requested");
            break;
          }
          case "disarm": {
            await controller.disarm();
            ok("Disarm requested");
            break;
          }
          case "takeoff": {
            const altitude =
              typeof detail.payload?.altitude === "number"
                ? detail.payload.altitude
                : 2.0;
            // Optional convenience: if UI set auto=true, best-effort arm first.
            if (detail.payload?.auto) {
              try {
                await controller.arm();
              } catch {
                /* likely already armed; ignore */
              }
            }
            const yaw =
              typeof detail.payload?.yaw === "number" ? detail.payload.yaw : undefined;
            await controller.takeoff(altitude, yaw);
            ok("Takeoff requested", `Altitude: ${altitude} m`);
            break;
          }
          case "land": {
            await controller.land(0);
            ok("Land requested");
            break;
          }
          case "rtl": {
            await controller.rtl();
            ok("Return-to-Launch requested");
            break;
          }
          case "goto": {
            // The provided DroneController API does not expose a global LLA goto.
            // If you add a ServiceCaller.commandLong for MAV_CMD_NAV_WAYPOINT (16),
            // you could support it here. For now, we surface a clear error.
            throw new Error(
              'Goto is not supported by DroneController. Provide a ServiceCaller with a "goto" helper (e.g., MAV_CMD_NAV_WAYPOINT) or add a controller.gotoLLA(lat, lon, alt, yaw).'
            );
          }
          default:
            throw new Error(`Unsupported action "${(detail as any).action}".`);
        }
      } catch (err) {
        fail(err);
      }
    },
    [controller, pushToast]
  );

  // Event listener
  useEffect(() => {
    const handler = (ev: Event) => {
      const e = ev as CustomEvent<AppRequestDetail>;
      const d = e.detail;
      if (!d || d.category !== "mission_control") return;

      // Show a quick "received" info toast immediately
      pushToast({
        id: `recv-${d.requestId}`,
        kind: "info",
        title: `Received "${d.action}"`,
        description:
          d.payload == null ? undefined : JSON.stringify(d.payload, null, 0),
      });

      // Forward to controller
      forward(d);
    };

    window.addEventListener(EVENT_NAME, handler as EventListener);
    return () =>
      window.removeEventListener(EVENT_NAME, handler as EventListener);
  }, [forward, pushToast]);

  // Keep the component otherwise invisible; only renders the toast host.
  const visibleToasts = useMemo(() => toasts, [toasts]);
  return <ToastHost toasts={visibleToasts} />;
}
