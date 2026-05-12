import React, { useCallback, useEffect, useRef, useState } from "react";
import { ros2Bridge } from "tensorfleet-ros";
import { DirectionalPad } from "../Teleop/DirectionalPad";
import { DirectionalPadAction } from "../Teleop/types";

const TELEOP_TOPIC = "/cmd_vel_raw";
const TELEOP_MSG_TYPE = "geometry_msgs/msg/Twist";
const PUBLISH_RATE_HZ = 10;
const LINEAR_SPEED = 0.3;
const ANGULAR_SPEED = 0.8;

type Vec3 = { x: number; y: number; z: number };
type TwistMsg = { linear: Vec3; angular: Vec3 };
type ActiveDir = "fwd" | "rev" | "left" | "right";

function zeroTwist(): TwistMsg {
  return { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
}

function buildTwist(dirs: Set<ActiveDir>): TwistMsg {
  const msg = zeroTwist();
  if (dirs.has("fwd")) msg.linear.x += LINEAR_SPEED;
  if (dirs.has("rev")) msg.linear.x -= LINEAR_SPEED;
  if (dirs.has("left")) msg.angular.z += ANGULAR_SPEED;
  if (dirs.has("right")) msg.angular.z -= ANGULAR_SPEED;
  return msg;
}

function padActionToDir(action: DirectionalPadAction): ActiveDir {
  switch (action) {
    case DirectionalPadAction.UP:
      return "fwd";
    case DirectionalPadAction.DOWN:
      return "rev";
    case DirectionalPadAction.LEFT:
      return "left";
    case DirectionalPadAction.RIGHT:
      return "right";
  }
}

export function TeleopCard(props: { disabled?: boolean; disabledReason?: string } = {}): React.JSX.Element {
  const [isConnected, setIsConnected] = useState(false);
  const [isExpanded, setIsExpanded] = useState(true);
  const [kbdEnabled, setKbdEnabled] = useState(false);
  const [padAction, setPadAction] = useState<DirectionalPadAction | undefined>();
  const [lastTwist, setLastTwist] = useState<TwistMsg | null>(null);

  const kbdDirsRef = useRef<Set<ActiveDir>>(new Set<ActiveDir>());
  const padDirRef = useRef<ActiveDir | null>(null);
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null);

  useEffect(() => {
    const check = () => setIsConnected(ros2Bridge.isConnected());
    check();
    const id = setInterval(check, 1000);
    return () => clearInterval(id);
  }, []);

  const getActiveDirs = useCallback((): Set<ActiveDir> => {
    const dirs = new Set<ActiveDir>(kbdDirsRef.current ?? []);
    if (padDirRef.current) dirs.add(padDirRef.current);
    return dirs;
  }, []);

  const publishNow = useCallback((): boolean => {
    if (props.disabled) {
      return false;
    }
    const dirs = getActiveDirs();
    if (dirs.size === 0 || !ros2Bridge.isConnected()) {
      return false;
    }
    const msg = buildTwist(dirs);
    ros2Bridge.publish(TELEOP_TOPIC, TELEOP_MSG_TYPE, msg);
    setLastTwist(msg);
    return true;
  }, [getActiveDirs, props.disabled]);

  const ensureInterval = useCallback(() => {
    if (intervalRef.current) return;
    intervalRef.current = setInterval(() => {
      if (!publishNow()) {
        if (intervalRef.current) {
          clearInterval(intervalRef.current);
          intervalRef.current = null;
        }
      }
    }, 1000 / PUBLISH_RATE_HZ);
  }, [publishNow]);

  const stopAndZero = useCallback(() => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    if (ros2Bridge.isConnected()) {
      ros2Bridge.publish(TELEOP_TOPIC, TELEOP_MSG_TYPE, zeroTwist());
    }
    setLastTwist(null);
  }, []);

  const handlePadAction = useCallback(
    (action?: DirectionalPadAction) => {
      if (action === undefined) {
        padDirRef.current = null;
        setPadAction(undefined);
        if (getActiveDirs().size === 0) stopAndZero();
      } else {
        padDirRef.current = padActionToDir(action);
        setPadAction(action);
        publishNow();
        ensureInterval();
      }
    },
    [getActiveDirs, stopAndZero, publishNow, ensureInterval],
  );

  useEffect(() => {
    if (!kbdEnabled || !isConnected || props.disabled) return;

    const KEY_MAP: Record<string, ActiveDir> = {
      w: "fwd",
      arrowup: "fwd",
      s: "rev",
      arrowdown: "rev",
      a: "left",
      arrowleft: "left",
      d: "right",
      arrowright: "right",
    };

    const EDITABLE = new Set(["INPUT", "TEXTAREA", "SELECT"]);
    const shouldSkip = (t: EventTarget | null) =>
      t instanceof HTMLElement && (t.isContentEditable || EDITABLE.has(t.tagName));

    const onKeyDown = (e: KeyboardEvent) => {
      if (shouldSkip(e.target)) return;
      const dir = KEY_MAP[e.key.toLowerCase()];
      if (!dir) return;
      e.preventDefault();
      if (kbdDirsRef.current?.has(dir)) return;
      kbdDirsRef.current?.add(dir);
      publishNow();
      ensureInterval();
    };

    const onKeyUp = (e: KeyboardEvent) => {
      const dir = KEY_MAP[e.key.toLowerCase()];
      if (!dir) return;
      kbdDirsRef.current?.delete(dir);
      if (getActiveDirs().size === 0) stopAndZero();
    };

    const onBlur = () => {
      kbdDirsRef.current?.clear();
      if (getActiveDirs().size === 0) stopAndZero();
    };

    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    window.addEventListener("blur", onBlur);

    return () => {
      kbdDirsRef.current?.clear();
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      window.removeEventListener("blur", onBlur);
      stopAndZero();
    };
  }, [kbdEnabled, isConnected, props.disabled, getActiveDirs, publishNow, ensureInterval, stopAndZero]);

  useEffect(() => {
    if (!isConnected || props.disabled) stopAndZero();
  }, [isConnected, props.disabled, stopAndZero]);

  const isMoving = !!(lastTwist && (lastTwist.linear.x !== 0 || lastTwist.angular.z !== 0));

  const speedLabel = (() => {
    if (!lastTwist) return null;
    const { linear, angular } = lastTwist;
    if (linear.x !== 0) return `${linear.x > 0 ? "▲" : "▼"} ${Math.abs(linear.x).toFixed(1)} m/s`;
    if (angular.z !== 0) return `${angular.z > 0 ? "↺" : "↻"} ${Math.abs(angular.z).toFixed(1)} r/s`;
    return null;
  })();

  return (
    <section className="vacuum-panel-card vacuum-panel-card--teleop">
      <button
        className="vacuum-panel-card__head vacuum-teleop-head"
        type="button"
        onClick={() => { setIsExpanded((v) => !v); }}
        aria-expanded={isExpanded}
      >
        <p className="vacuum-panel-card__eyebrow">Manual Control</p>
        <div className="vacuum-teleop-head-right">
          <span className={`vacuum-teleop-badge vacuum-teleop-badge--${isMoving ? "active" : isConnected ? "ready" : "off"}`}>
            {isMoving ? "Moving" : isConnected ? "Ready" : "Offline"}
          </span>
          <svg className="vacuum-teleop-chevron" viewBox="0 0 16 16" aria-hidden="true">
            <path d={isExpanded ? "M4 10l4-4 4 4" : "M4 6l4 4 4-4"} />
          </svg>
        </div>
      </button>

      {isExpanded && (
        <div className="vacuum-teleop-body">
          {!isConnected || props.disabled ? (
            <p className="vacuum-teleop-hint">
              {props.disabled ? props.disabledReason ?? "Manual control is disabled." : "Connect to the robot to enable manual control."}
            </p>
          ) : (
            <>
              <div className="vacuum-teleop-pad-wrap">
                <DirectionalPad
                  onAction={handlePadAction}
                  disabled={!isConnected || props.disabled}
                  activeAction={padAction}
                />
              </div>
              <div className="vacuum-teleop-footer">
                <button
                  className={`vacuum-teleop-kbd${kbdEnabled ? " vacuum-teleop-kbd--active" : ""}`}
                  type="button"
                  title={
                    kbdEnabled
                      ? "Keyboard on – click to disable WASD / arrow key control"
                      : "Keyboard off – click to enable WASD / arrow key control"
                  }
                  onClick={() => {
                    if (kbdEnabled) {
                      kbdDirsRef.current?.clear();
                      stopAndZero();
                    }
                    setKbdEnabled((v) => !v);
                  }}
                >
                  <svg viewBox="0 0 20 20" aria-hidden="true">
                    <rect x="2" y="5" width="16" height="10" rx="2" />
                    <rect x="4" y="7" width="3" height="2.5" rx="0.5" />
                    <rect x="8.5" y="7" width="3" height="2.5" rx="0.5" />
                    <rect x="13" y="7" width="3" height="2.5" rx="0.5" />
                    <rect x="6.5" y="11" width="7" height="2" rx="0.5" />
                  </svg>
                  <span>{kbdEnabled ? "WASD on" : "WASD off"}</span>
                </button>
                {speedLabel ? (
                  <span className="vacuum-teleop-readout">{speedLabel}</span>
                ) : null}
              </div>
            </>
          )}
        </div>
      )}
    </section>
  );
}
