// SimulationControlBridge.tsx
import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { SimulationController } from "@/mission-control/simulation_controller";

/** Keep this in sync with the UI emitter */
interface SimulationRequestDetail<T = any> {
  category: "simulation_control";
  action: "restart";
  payload?: T;
  requestId: string;
  timestamp: number;
}

const EVENT_NAME = "app:request";

// ---------- Toasts (same style as your MissionControlBridge) ----------

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
  itemClosing: { opacity: 0, transform: "translateY(8px)" },
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

export interface SimulationControlBridgeProps {
  controller: SimulationController;
  toastMs?: number;
}

/**
 * Listens for "app:request" (simulation_control) and forwards actions to SimulationController.
 * Shows bottom-center toasts for each forwarded command (or any error).
 */
export default function SimulationControlBridge({
  controller,
  toastMs = 6000,
}: SimulationControlBridgeProps) {
  const [toasts, setToasts] = useState<Toast[]>([]);
  const timers = useRef<Record<string, number[]>>({});

  const pushToast = useCallback(
    (partial: Omit<Toast, "createdAt" | "closing">) => {
      const t: Toast = { ...partial, createdAt: Date.now() };
      setToasts((arr) => [...arr, t]);

      const closeTimer = window.setTimeout(() => {
        setToasts((arr) =>
          arr.map((x) => (x.id === t.id ? { ...x, closing: true } : x))
        );
      }, Math.max(0, toastMs - 300));

      const removeTimer = window.setTimeout(() => {
        setToasts((arr) => arr.filter((x) => x.id !== t.id));
        delete timers.current[t.id];
      }, toastMs);

      timers.current[t.id] = [closeTimer, removeTimer];
    },
    [toastMs]
  );

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
    async (detail: SimulationRequestDetail) => {
      const id = detail.requestId;

      const ok = (msg: string, extra?: string) =>
        pushToast({
          id,
          kind: "success",
          title: msg,
          description:
            `Forwarded "${detail.action}" (#${id.slice(0, 6)}).` +
            (extra ? `\n${extra}` : ""),
        });

      const fail = (err: unknown) =>
        pushToast({
          id,
          kind: "error",
          title: `Failed to forward "${detail.action}"`,
          description: err instanceof Error ? err.message : String(err),
        });

      try {
        switch (detail.action) {
          case "restart": {
            await controller.restart();
            ok("Simulation restart requested");
            break;
          }
          default:
            throw new Error(`Unsupported simulation action "${(detail as any).action}".`);
        }
      } catch (err) {
        fail(err);
      }
    },
    [controller, pushToast]
  );

  useEffect(() => {
    const handler = (ev: Event) => {
      const e = ev as CustomEvent<SimulationRequestDetail>;
      const d = e.detail;
      if (!d || d.category !== "simulation_control") return;

      pushToast({
        id: `recv-${d.requestId}`,
        kind: "info",
        title: `Received simulation "${d.action}"`,
      });

      forward(d);
    };

    window.addEventListener(EVENT_NAME, handler as EventListener);
    return () =>
      window.removeEventListener(EVENT_NAME, handler as EventListener);
  }, [forward, pushToast]);

  const visibleToasts = useMemo(() => toasts, [toasts]);
  return <ToastHost toasts={visibleToasts} />;
}
