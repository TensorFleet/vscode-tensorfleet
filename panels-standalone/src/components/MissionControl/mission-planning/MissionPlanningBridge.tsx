import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  MissionPlanningAction,
  MissionPlanningEventDetail,
  MissionPlan,
  MissionWaypoint,
  MissionValidationError,
  validateMission,
  calculateMissionStats,
  createEmptyMission,
  generateWaypointId,
  postMissionPlanningRequest,
} from "./mission-planning-protocol";

/** Keep this in sync with the UI emitter */
type MissionPlanningEvent = CustomEvent<MissionPlanningEventDetail>;

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
  dot: {
    flex: "0 0 8px",
    width: 8,
    height: 8,
    marginTop: 6,
    borderRadius: 999,
    background: "#3b82f6", // Default color, will be overridden per toast
  },
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
              <div style={{ 
                ...toastStyles.dot, 
                background: t.kind === "success" ? "#10b981" : t.kind === "error" ? "#ef4444" : "#3b82f6" 
              }} />
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

// ---------- Mission Store ----------
interface MissionStore {
  missions: MissionPlan[];
  activeMissionId: string | null;
  isLoading: boolean;
  error: string | null;
}

class MissionStoreImpl implements MissionStore {
  private listeners: Array<(store: MissionStore) => void> = [];
  private _missions: MissionPlan[] = [];
  private _activeMissionId: string | null = null;
  private _isLoading = false;
  private _error: string | null = null;

  get missions() { return this._missions; }
  get activeMissionId() { return this._activeMissionId; }
  get isLoading() { return this._isLoading; }
  get error() { return this._error; }

  setMissions(missions: MissionPlan[]) {
    this._missions = missions;
    this.notify();
  }

  setActiveMissionId(id: string | null) {
    this._activeMissionId = id;
    this.notify();
  }

  setIsLoading(loading: boolean) {
    this._isLoading = loading;
    this.notify();
  }

  setError(error: string | null) {
    this._error = error;
    this.notify();
  }

  addMission(mission: MissionPlan) {
    this._missions = [...this._missions, mission];
    this.notify();
  }

  updateMission(mission: MissionPlan) {
    this._missions = this._missions.map(m => m.id === mission.id ? mission : m);
    this.notify();
  }

  removeMission(missionId: string) {
    this._missions = this._missions.filter(m => m.id !== missionId);
    if (this._activeMissionId === missionId) {
      this._activeMissionId = null;
    }
    this.notify();
  }

  subscribe(fn: (store: MissionStore) => void) {
    this.listeners.push(fn);
    return () => {
      this.listeners = this.listeners.filter(l => l !== fn);
    };
  }

  private notify() {
    this.listeners.forEach(fn => fn(this));
  }
}

const missionStore = new MissionStoreImpl();

// ---------- Mission Planning Bridge ----------
export interface MissionPlanningBridgeProps {
  /** Optional: how long to show toast messages (ms). Default: 6000 */
  toastMs?: number;
}

/**
 * Listens for "app:request" (mission_planning) and manages mission state.
 * Shows bottom-center toasts for each forwarded command (or any error).
 *
 * Usage:
 *   <MissionPlanningBridge />
 */
export default function MissionPlanningBridge({
  toastMs = 6000,
}: MissionPlanningBridgeProps) {
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

  // Event listener for mission planning requests
  useEffect(() => {
    const handler = (ev: Event) => {
      const e = ev as MissionPlanningEvent;
      const d = e.detail;
      if (!d || d.category !== "mission_planning") return;

      // Show a quick "received" info toast immediately
      pushToast({
        id: `recv-${d.requestId}`,
        kind: "info",
        title: `Received "${d.action}"`,
        description:
          d.payload == null ? undefined : JSON.stringify(d.payload, null, 0),
      });

      // Forward to handler
      handleMissionPlanningRequest(d);
    };

    window.addEventListener(EVENT_NAME, handler as EventListener);
    return () =>
      window.removeEventListener(EVENT_NAME, handler as EventListener);
  }, [pushToast]);

  // Keep the component otherwise invisible; only renders the toast host.
  const visibleToasts = useMemo(() => toasts, [toasts]);
  return <ToastHost toasts={visibleToasts} />;
}

// ---------- Request Handlers ----------
async function handleMissionPlanningRequest(detail: MissionPlanningEventDetail) {
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
      case "create_mission": {
        const { name, description } = detail.payload as { name: string; description?: string };
        const mission = createEmptyMission(name, description);
        missionStore.addMission(mission);
        missionStore.setActiveMissionId(mission.id);
        ok("Mission created", `Name: ${name}`);
        break;
      }
      case "save_mission": {
        const { mission } = detail.payload as { mission: MissionPlan };
        const errors = validateMission(mission);
        if (errors.length > 0) {
          throw new Error(`Validation failed: ${errors.map(e => e.message).join(", ")}`);
        }
        
        // Update stats
        const stats = calculateMissionStats(mission);
        mission.metadata = { ...mission.metadata, ...stats };
        mission.updatedAt = Date.now();
        mission.status = "saved";
        
        missionStore.updateMission(mission);
        ok("Mission saved", `Name: ${mission.name}`);
        break;
      }
      case "load_mission": {
        const { missionId } = detail.payload as { missionId: string };
        const mission = missionStore.missions.find(m => m.id === missionId);
        if (!mission) {
          throw new Error(`Mission not found: ${missionId}`);
        }
        missionStore.setActiveMissionId(missionId);
        ok("Mission loaded", `Name: ${mission.name}`);
        break;
      }
      case "delete_mission": {
        const { missionId } = detail.payload as { missionId: string };
        const mission = missionStore.missions.find(m => m.id === missionId);
        if (!mission) {
          throw new Error(`Mission not found: ${missionId}`);
        }
        missionStore.removeMission(missionId);
        ok("Mission deleted", `Name: ${mission.name}`);
        break;
      }
      case "cancel_mission": {
        const { missionId } = detail.payload as { missionId: string };
        const mission = missionStore.missions.find(m => m.id === missionId);
        if (!mission) {
          throw new Error(`Mission not found: ${missionId}`);
        }
        if (mission.status === "executing") {
          // TODO: Send cancel command to drone controller
          // For now, just update status
          mission.status = "cancelled";
          missionStore.updateMission(mission);
        }
        ok("Mission cancelled", `Name: ${mission.name}`);
        break;
      }
      case "finalize_mission": {
        const { missionId } = detail.payload as { missionId: string };
        const mission = missionStore.missions.find(m => m.id === missionId);
        if (!mission) {
          throw new Error(`Mission not found: ${missionId}`);
        }
        const errors = validateMission(mission);
        if (errors.length > 0) {
          throw new Error(`Validation failed: ${errors.map(e => e.message).join(", ")}`);
        }
        mission.status = "saved";
        mission.updatedAt = Date.now();
        missionStore.updateMission(mission);
        ok("Mission finalized", `Name: ${mission.name}`);
        break;
      }
      case "execute_mission": {
        const { missionId } = detail.payload as { missionId: string };
        const mission = missionStore.missions.find(m => m.id === missionId);
        if (!mission) {
          throw new Error(`Mission not found: ${missionId}`);
        }
        if (mission.status !== "saved") {
          throw new Error(`Mission must be saved before execution: ${mission.name}`);
        }
        
        // TODO: Send execute command to drone controller
        // For now, just update status
        mission.status = "executing";
        missionStore.updateMission(mission);
        ok("Mission execution started", `Name: ${mission.name}`);
        break;
      }
      case "list_missions": {
        // This is handled by the MissionList component directly
        ok("Mission list requested");
        break;
      }
      default:
        throw new Error(`Unsupported action "${(detail as any).action}".`);
    }
  } catch (err) {
    fail(err);
  }
}

// ---------- Toast forwarding ----------
function pushToast(partial: Omit<Toast, "createdAt" | "closing">) {
  // This will be called from the bridge component's context
  // For now, we'll use a global function that the bridge can override
  (window as any).__missionPlanningPushToast?.(partial);
}

// ---------- Mission List Component ----------
export function MissionList() {
  const [store, setStore] = useState<MissionStore>(() => ({
    missions: missionStore.missions,
    activeMissionId: missionStore.activeMissionId,
    isLoading: missionStore.isLoading,
    error: missionStore.error,
  }));

  useEffect(() => {
    const unsubscribe = missionStore.subscribe((newStore) => {
      setStore({ ...newStore });
    });
    return unsubscribe;
  }, []);

  const handleExecute = (missionId: string) => {
    postMissionPlanningRequest("execute_mission", { missionId });
  };

  const handleDelete = (missionId: string) => {
    if (confirm("Are you sure you want to delete this mission?")) {
      postMissionPlanningRequest("delete_mission", { missionId });
    }
  };

  const handleLoad = (missionId: string) => {
    postMissionPlanningRequest("load_mission", { missionId });
  };

  return (
    <div style={{ padding: 16 }}>
      <div style={{ fontSize: 18, fontWeight: 600, marginBottom: 12 }}>Saved Missions</div>
      
      {store.error && (
        <div style={{ color: "#ef4444", marginBottom: 8 }}>{store.error}</div>
      )}
      
      {store.missions.length === 0 ? (
        <div style={{ color: "#6b7280", fontStyle: "italic" }}>No saved missions</div>
      ) : (
        <div style={{ display: "grid", gap: 8 }}>
          {store.missions.map((mission) => (
            <div
              key={mission.id}
              style={{
                border: "1px solid #e5e7eb",
                borderRadius: 8,
                padding: 12,
                background: store.activeMissionId === mission.id ? "#f3f4f6" : "white",
                display: "grid",
                gridTemplateColumns: "1fr auto",
                gap: 8,
                alignItems: "center",
              }}
            >
              <div>
                <div style={{ fontWeight: 600 }}>{mission.name}</div>
                {mission.description && (
                  <div style={{ fontSize: 12, color: "#6b7280", marginTop: 2 }}>
                    {mission.description}
                  </div>
                )}
                <div style={{ fontSize: 12, color: "#6b7280", marginTop: 4 }}>
                  Status: {mission.status} • Waypoints: {mission.waypoints.length}
                  {mission.metadata?.totalDistance && (
                    <> • Distance: {(mission.metadata.totalDistance / 1000).toFixed(2)} km</>
                  )}
                </div>
              </div>
              
              <div style={{ display: "flex", gap: 8, justifyContent: "flex-end" }}>
                <button
                  style={{
                    padding: "6px 12px",
                    border: "1px solid #e5e7eb",
                    borderRadius: 6,
                    background: "white",
                    cursor: "pointer",
                    fontSize: 12,
                  }}
                  onClick={() => handleLoad(mission.id)}
                >
                  Load
                </button>
                {mission.status === "saved" && (
                  <button
                    style={{
                      padding: "6px 12px",
                      border: "1px solid #e5e7eb",
                      borderRadius: 6,
                      background: "#10b981",
                      color: "white",
                      cursor: "pointer",
                      fontSize: 12,
                    }}
                    onClick={() => handleExecute(mission.id)}
                  >
                    Execute
                  </button>
                )}
                <button
                  style={{
                    padding: "6px 12px",
                    border: "1px solid #e5e7eb",
                    borderRadius: 6,
                    background: "#ef4444",
                    color: "white",
                    cursor: "pointer",
                    fontSize: 12,
                  }}
                  onClick={() => handleDelete(mission.id)}
                >
                  Delete
                </button>
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

// ---------- Context Menu Component ----------
export function MissionContextMenu({ 
  missionId, 
  onExecute, 
  onCancel, 
  onDelete 
}: { 
  missionId: string; 
  onExecute: () => void; 
  onCancel: () => void; 
  onDelete: () => void; 
}) {
  return (
    <div
      style={{
        position: "absolute",
        top: 8,
        right: 8,
        background: "white",
        border: "1px solid #e5e7eb",
        borderRadius: 8,
        boxShadow: "0 4px 12px rgba(0,0,0,0.15)",
        padding: 8,
        display: "flex",
        flexDirection: "column",
        gap: 4,
        zIndex: 1000,
      }}
    >
      <button
        style={{
          padding: "6px 12px",
          border: "1px solid #e5e7eb",
          borderRadius: 6,
          background: "#10b981",
          color: "white",
          cursor: "pointer",
          fontSize: 12,
          textAlign: "left",
        }}
        onClick={onExecute}
      >
        Execute
      </button>
      <button
        style={{
          padding: "6px 12px",
          border: "1px solid #e5e7eb",
          borderRadius: 6,
          background: "#f59e0b",
          color: "white",
          cursor: "pointer",
          fontSize: 12,
          textAlign: "left",
        }}
        onClick={onCancel}
      >
        Cancel
      </button>
      <button
        style={{
          padding: "6px 12px",
          border: "1px solid #e5e7eb",
          borderRadius: 6,
          background: "#ef4444",
          color: "white",
          cursor: "pointer",
          fontSize: 12,
          textAlign: "left",
        }}
        onClick={onDelete}
      >
        Delete
      </button>
    </div>
  );
}