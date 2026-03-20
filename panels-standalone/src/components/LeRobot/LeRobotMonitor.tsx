import React, { useCallback, useEffect, useRef, useState } from 'react';
import './LeRobotMonitor.css';

// ── Types ─────────────────────────────────────────────────────────────────────

type Tab = 'teleop' | 'training' | 'deploy';

interface Metric    { step: number; loss: number; gradNorm: number; }
interface LogLine   { text: string; cls: string; }
interface Checkpoint { step: number; loss: number | null; gradNorm: number | null; savedAt: string; }

type RosStatus = 'disconnected' | 'connecting' | 'connected';

// ── Rosbridge hook ────────────────────────────────────────────────────────────

interface RosbridgeHandle {
  status: RosStatus;
  subscribe: (topic: string, type: string, cb: (msg: unknown) => void) => () => void;
}

function useRosbridge(url: string): RosbridgeHandle {
  const [status, setStatus] = useState<RosStatus>('disconnected');
  const wsRef  = useRef<WebSocket | null>(null);
  const cbsRef = useRef<Map<string, Array<(msg: unknown) => void>>>(new Map());
  const urlRef = useRef(url);
  urlRef.current = url;

  const sendSubscribe = (ws: WebSocket, topic: string) => {
    ws.send(JSON.stringify({ op: 'subscribe', topic }) as string);
  };

  useEffect(() => {
    if (!url) return;
    const safeUrl = url;
    let dead = false;
    let timer: ReturnType<typeof setTimeout>;

    function connect() {
      if (dead) return;
      try {
        setStatus('connecting');
        const ws = new WebSocket(safeUrl);
        wsRef.current = ws;

        ws.onopen = () => {
          if (dead) { ws.close(); return; }
          setStatus('connected');
          cbsRef.current.forEach((_, topic) => sendSubscribe(ws, topic));
        };

        ws.onclose = () => {
          if (dead) return;
          setStatus('disconnected');
          timer = setTimeout(connect, 2000) as unknown as ReturnType<typeof setTimeout>;
        };

        ws.onerror = () => {
          ws.close();
        };

        ws.onmessage = (ev) => {
          try {
            const msg = JSON.parse(ev.data as string) as { op: string; topic: string; msg: unknown };
            if (msg.op === 'publish') {
              const cbs = cbsRef.current.get(msg.topic);
              if (cbs) cbs.forEach(cb => cb(msg.msg));
            }
          } catch { /* ignore parse errors */ }
        };
      } catch {
        if (!dead) timer = setTimeout(connect, 2000);
      }
    }

    connect();
    return () => {
      dead = true;
      clearTimeout(timer);
      if (wsRef.current) { wsRef.current.onclose = null; wsRef.current.close(); }
      setStatus('disconnected');
    };
  }, [url]);

  const subscribe = useCallback((topic: string, type: string, cb: (msg: unknown) => void) => {
    if (!cbsRef.current.has(topic)) cbsRef.current.set(topic, []);
    cbsRef.current.get(topic)!.push(cb);
    // Subscribe if already open
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ op: 'subscribe', topic, type }) as string);
    }
    return () => {
      const list = cbsRef.current.get(topic);
      if (!list) return;
      const idx = list.indexOf(cb);
      if (idx >= 0) list.splice(idx, 1);
    };
  }, []);

  return { status, subscribe };
}

// ── Canvas helpers ────────────────────────────────────────────────────────────

function cssVar(name: string, fallback = ''): string {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
}

function drawLossChart(canvas: HTMLCanvasElement, metrics: Metric[], checkpointSteps: Set<number>) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  if (!rect.width || !rect.height) return;
  canvas.width  = rect.width  * dpr;
  canvas.height = rect.height * dpr;
  const ctx = canvas.getContext('2d')!;
  ctx.scale(dpr, dpr);
  const W = rect.width, H = rect.height;
  const P = { t: 6, r: 10, b: 22, l: 44 };

  ctx.clearRect(0, 0, W, H);

  const fg     = cssVar('--vscode-foreground', '#ccc');
  const border = cssVar('--vscode-editorWidget-border', 'rgba(128,128,128,0.25)');
  const blue   = cssVar('--vscode-charts-blue',  cssVar('--vscode-terminal-ansiBlue',  '#4d9cf5'));
  const green  = cssVar('--vscode-charts-green', cssVar('--vscode-terminal-ansiGreen', '#4ec94e'));

  if (metrics.length < 2) {
    ctx.fillStyle = fg; ctx.globalAlpha = 0.35;
    ctx.font = '11px sans-serif'; ctx.textAlign = 'center';
    ctx.fillText('Waiting for training data…', W / 2, H / 2);
    return;
  }

  const minX = metrics[0].step, maxX = metrics[metrics.length - 1].step;
  const losses = metrics.map(m => m.loss);
  const minY = Math.min(...losses) * 0.95, maxY = Math.max(...losses) * 1.05;
  const xS = (s: number) => P.l + ((s - minX) / (maxX - minX || 1)) * (W - P.l - P.r);
  const yS = (l: number) => P.t + (1 - (l - minY) / (maxY - minY || 1)) * (H - P.t - P.b);

  // Grid
  ctx.strokeStyle = border; ctx.lineWidth = 0.5;
  for (let i = 0; i <= 3; i++) {
    const y = P.t + (i / 3) * (H - P.t - P.b);
    ctx.beginPath(); ctx.moveTo(P.l, y); ctx.lineTo(W - P.r, y); ctx.stroke();
    const v = maxY - (i / 3) * (maxY - minY);
    ctx.fillStyle = fg; ctx.globalAlpha = 0.4;
    ctx.font = '9px sans-serif'; ctx.textAlign = 'right';
    ctx.fillText(v.toFixed(3), P.l - 3, y + 3);
    ctx.globalAlpha = 1;
  }

  // Checkpoint lines
  ctx.setLineDash([3, 3]); ctx.strokeStyle = green; ctx.globalAlpha = 0.4; ctx.lineWidth = 1;
  checkpointSteps.forEach(step => {
    if (step < minX || step > maxX) return;
    const x = xS(step);
    ctx.beginPath(); ctx.moveTo(x, P.t); ctx.lineTo(x, H - P.b); ctx.stroke();
  });
  ctx.setLineDash([]); ctx.globalAlpha = 1;

  // Loss line
  ctx.beginPath(); ctx.strokeStyle = blue; ctx.lineWidth = 1.5; ctx.lineJoin = 'round';
  metrics.forEach((m, i) => {
    const x = xS(m.step), y = yS(m.loss);
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.stroke();

  // X axis labels
  ctx.fillStyle = fg; ctx.globalAlpha = 0.4; ctx.font = '9px sans-serif'; ctx.textAlign = 'center';
  for (let i = 0; i <= 4; i++) {
    const s = minX + (i / 4) * (maxX - minX);
    ctx.fillText(fmtStep(Math.round(s)), xS(s), H - P.b + 12);
  }
}

function drawBarChart(canvas: HTMLCanvasElement, buckets: Array<{ count: number }>) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  if (!rect.width || !rect.height) return;
  canvas.width  = rect.width  * dpr;
  canvas.height = rect.height * dpr;
  const ctx = canvas.getContext('2d')!;
  ctx.scale(dpr, dpr);
  const W = rect.width, H = rect.height;
  const P = { t: 4, r: 6, b: 14, l: 28 };
  const fg     = cssVar('--vscode-foreground', '#ccc');
  const border = cssVar('--vscode-editorWidget-border', 'rgba(128,128,128,0.25)');
  const blue   = cssVar('--vscode-charts-blue', cssVar('--vscode-terminal-ansiBlue', '#4d9cf5'));

  ctx.clearRect(0, 0, W, H);
  if (!buckets.length) {
    ctx.fillStyle = fg; ctx.globalAlpha = 0.3;
    ctx.font = '11px sans-serif'; ctx.textAlign = 'center';
    ctx.fillText('No inference data', W / 2, H / 2);
    return;
  }
  const maxCount = Math.max(...buckets.map(b => b.count), 1);
  const barW = (W - P.l - P.r) / buckets.length;

  ctx.strokeStyle = border; ctx.lineWidth = 0.5;
  ctx.beginPath(); ctx.moveTo(P.l, P.t); ctx.lineTo(P.l, H - P.b); ctx.stroke();

  ctx.fillStyle = fg; ctx.globalAlpha = 0.35; ctx.font = '9px sans-serif'; ctx.textAlign = 'right';
  ctx.fillText(String(maxCount), P.l - 2, P.t + 7);
  ctx.globalAlpha = 1;

  buckets.forEach((b, i) => {
    const x = P.l + i * barW + 1;
    const h = (b.count / maxCount) * (H - P.t - P.b);
    const y = H - P.b - h;
    ctx.fillStyle = blue; ctx.globalAlpha = 0.7;
    ctx.fillRect(x, y, Math.max(barW - 2, 1), h);
    ctx.globalAlpha = 1;
  });
}

// ── Helpers ───────────────────────────────────────────────────────────────────

function fmtStep(n: number): string {
  if (n >= 1000) return (n / 1000).toFixed(1).replace(/\.0$/, '') + 'K';
  return String(n);
}

function classifyTrain(line: string): string {
  if (/step:\d/i.test(line))                                        return 'step';
  if (/checkpoint|training complete/i.test(line))                   return 'checkpoint';
  if (/warn|only \d+ episode|push_to_hub failed/i.test(line))       return 'warn';
  return 'default';
}

function classifyDeploy(line: string): string {
  if (/\[DEPLOY\].*Inferences:/i.test(line))                        return 'step';
  if (/policy loaded|observations received/i.test(line))            return 'checkpoint';
  if (/interrupt|error/i.test(line))                                return 'warn';
  return 'default';
}

function addLogLine(prev: LogLine[], text: string, cls: string, max = 200): LogLine[] {
  const next = [...prev, { text, cls }];
  return next.length > max ? next.slice(next.length - max) : next;
}

function dotClass(lastMs: number | null, now: number): string {
  if (lastMs === null) return '';
  const d = now - lastMs;
  if (d < 500)  return 'lr-dot--ok';
  if (d < 2000) return 'lr-dot--warn';
  return 'lr-dot--err';
}

function ageText(lastMs: number | null, now: number): string {
  if (lastMs === null) return '—';
  const d = now - lastMs;
  if (d < 1000) return `${d}ms`;
  return `${(d / 1000).toFixed(1)}s`;
}

const JOINT_NAMES  = ['Shoulder Pan', 'Shoulder Lift', 'Elbow', 'Wrist Flex', 'Wrist Roll', 'Gripper'];
const JOINT_LIMITS: [number, number][] = [[-1.92, 1.92], [-1.75, 1.75], [-1.75, 1.57], [-1.66, 1.66], [-2.79, 2.79], [-0.17, 1.75]];
const CAM_TOPICS   = ['/so_arm101/wrist_camera/image_raw', '/so_arm101/agent_camera/image_raw', '/so_arm101/side_camera/image_raw'];
const CAM_LABELS   = ['/wrist_camera', '/agent_camera', '/side_camera'];
const DEPLOY_TOPICS = [
  { name: '/joint_states',                              dir: 'sub' },
  { name: '/so_arm101/wrist_camera/image_raw',          dir: 'sub' },
  { name: '/so_arm101/agent_camera/image_raw',          dir: 'sub' },
  { name: '/so_arm101/side_camera/image_raw',           dir: 'sub' },
  { name: '/arm_controller/joint_trajectory',           dir: 'pub' },
  { name: '/gripper_controller/joint_trajectory',       dir: 'pub' },
];

// ── Component ─────────────────────────────────────────────────────────────────

export function LeRobotMonitor(): React.JSX.Element {
  const [tab, setTab]                     = useState<Tab>('teleop');
  const [workspacePath, setWorkspacePath] = useState('');
  const [trainWatching, setTrainWatching] = useState(false);

  // Training
  const [metrics, setMetrics]             = useState<Metric[]>([]);
  const [checkpointSteps]                 = useState(() => new Set<number>());
  const [checkpoints, setCheckpoints]     = useState<Checkpoint[]>([]);
  const [totalSteps, setTotalSteps]       = useState(0);
  const [step, setStep]                   = useState(0);
  const [loss, setLoss]                   = useState<number | null>(null);
  const [lossHist, setLossHist]           = useState<Array<{ step: number; loss: number }>>([]);
  const [gradNorm, setGradNorm]           = useState<number | null>(null);
  const [epoch, setEpoch]                 = useState<number | null>(null);
  const [datasetFrames, setDatasetFrames] = useState<number | null>(null);
  const [trainLog, setTrainLog]           = useState<LogLine[]>([]);
  const [configText, setConfigText]       = useState<string | null>(null);
  const [configOpen, setConfigOpen]       = useState(false);

  // Teleop
  const [joints, setJoints]               = useState<Array<number | null>>(Array(6).fill(null));
  const [jointVel, setJointVel]           = useState<number[]>(Array(6).fill(0));
  const [camLastMs, setCamLastMs]         = useState<Array<number | null>>(Array(3).fill(null));
  const [recording, setRecording]         = useState<'idle' | 'recording' | 'waiting'>('idle');
  const [datasetInfo, setDatasetInfo]     = useState<string | null>(null);

  // Deploy
  const [inferences, setInferences]       = useState(0);
  const [actions, setActions]             = useState(0);
  const [policyPath, setPolicyPath]       = useState<string | null>(null);
  const [deployLog, setDeployLog]         = useState<LogLine[]>([]);
  const [topicLastMs, setTopicLastMs]     = useState<Array<number | null>>(Array(6).fill(null));
  const [buckets, setBuckets]             = useState<Array<{ count: number }>>([]);
  const [sessions, setSessions]           = useState<Array<{ time: string; inferences: number; policy: string }>>([]);
  const bucketRef                         = useRef<{ start: number; count: number }>({ start: 0, count: 0 });
  const prevSessionInfer                  = useRef(0);

  // ROS connection — URL comes from window global injected by extension host
  const rosbridgeUrl = (window as unknown as Record<string, string>)['LEROBOT_ROSBRIDGE_URL'] || '';
  const ros = useRosbridge(rosbridgeUrl);

  // Tick for age displays
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    const t = setInterval(() => setNow(Date.now()), 250);
    return () => clearInterval(t);
  }, []);

  // Charts
  const lossCanvasRef      = useRef<HTMLCanvasElement>(null);
  const throughputCanvasRef = useRef<HTMLCanvasElement>(null);
  const metricsRef         = useRef(metrics);
  metricsRef.current       = metrics;
  const checkpointStepsRef = useRef(checkpointSteps);
  checkpointStepsRef.current = checkpointSteps;
  const bucketsRef         = useRef(buckets);
  bucketsRef.current       = buckets;

  // Decimation counter for joint states
  const jointDecRef = useRef(0);

  // ── ROS subscriptions ──────────────────────────────────────────────────────

  useEffect(() => {
    const unsubs: Array<() => void> = [];

    // Joint states
    unsubs.push(ros.subscribe('/joint_states', 'sensor_msgs/msg/JointState', (raw) => {
      jointDecRef.current++;
      if (jointDecRef.current % 10 !== 0) return; // decimate to ~10 Hz
      const msg = raw as { position?: number[]; velocity?: number[] };
      setJoints((msg.position || []).slice(0, 6).map((v, i) => (i < 6 ? v : null)));
      setJointVel((msg.velocity || []).slice(0, 6).map(v => Math.abs(v)));
      // Update deploy topic freshness
      setTopicLastMs(prev => { const next = [...prev]; next[0] = Date.now(); return next; });
    }));

    // Camera topics
    CAM_TOPICS.forEach((topic, i) => {
      unsubs.push(ros.subscribe(topic, 'sensor_msgs/msg/Image', () => {
        setCamLastMs(prev => { const next = [...prev]; next[i] = Date.now(); return next; });
        setTopicLastMs(prev => { const next = [...prev]; next[i + 1] = Date.now(); return next; });
      }));
    });

    return () => unsubs.forEach(u => u());
  }, [ros]);

  // ── Extension host messages ────────────────────────────────────────────────

  useEffect(() => {
    function onMessage(ev: MessageEvent) {
      const msg = ev.data as Record<string, unknown>;
      if (!msg?.type) return;

      switch (msg.type) {
        case 'init': {
          if (msg.workspaceFolder) setWorkspacePath(msg.workspaceFolder as string);
          if (msg.checkpoints) {
            const cps = (msg.checkpoints as Array<{ step: number; savedAt: string }>);
            cps.forEach(c => checkpointSteps.add(c.step));
            setCheckpoints(cps.map(c => ({ step: c.step, loss: null, gradNorm: null, savedAt: c.savedAt })));
          }
          break;
        }

        case 'watching':
          setTrainWatching(Boolean(msg.active));
          break;

        case 'metrics': {
          const { step: s, loss: l, gradNorm: g, epoch: e } = msg as Record<string, number>;
          setStep(s);
          setLoss(l);
          setGradNorm(g);
          if (e != null) setEpoch(e);
          setLossHist(prev => {
            const next = [{ step: s, loss: l }, ...prev];
            return next.length > 500 ? next.slice(0, 500) : next;
          });
          setMetrics(prev => {
            const next = [...prev, { step: s, loss: l, gradNorm: g }];
            return next.length > 2000 ? next.slice(next.length - 2000) : next;
          });
          setTrainWatching(true);
          break;
        }

        case 'config': {
          if (msg.totalSteps)    setTotalSteps(msg.totalSteps as number);
          if (msg.datasetFrames) setDatasetFrames(msg.datasetFrames as number);
          if (msg.raw)           setConfigText(msg.raw as string);
          break;
        }

        case 'checkpoint': {
          const cs = msg.step as number;
          if (!checkpointSteps.has(cs)) {
            checkpointSteps.add(cs);
            setCheckpoints(prev => {
              const entry: Checkpoint = { step: cs, loss: loss, gradNorm: gradNorm, savedAt: new Date().toLocaleTimeString() };
              return [...prev, entry].sort((a, b) => a.step - b.step);
            });
          }
          break;
        }

        case 'checkpoints_scan': {
          const cps = (msg.checkpoints as Array<{ step: number; savedAt: string }>) || [];
          cps.forEach(c => checkpointSteps.add(c.step));
          setCheckpoints(prev => {
            const existing = new Set(prev.map(c => c.step));
            const news = cps.filter(c => !existing.has(c.step))
              .map(c => ({ step: c.step, loss: null, gradNorm: null, savedAt: c.savedAt }));
            return [...prev, ...news].sort((a, b) => a.step - b.step);
          });
          break;
        }

        case 'stdout': {
          const line = msg.line as string;
          const mode = msg.mode as string;

          if (mode === 'train') {
            setTrainLog(prev => addLogLine(prev, line, classifyTrain(line)));
            if (/recording episode/i.test(line)) setRecording('recording');
            if (/waiting for/i.test(line))       setRecording('waiting');
            if (/saved|finalized/i.test(line))   setRecording('idle');
          } else if (mode === 'deploy') {
            setDeployLog(prev => addLogLine(prev, line, classifyDeploy(line)));

            // Parse inference count
            const im = line.match(/\[DEPLOY\]\s+Inferences:\s*(\d+),\s*Actions:\s*(\d+)/i);
            if (im) {
              const inf = parseInt(im[1], 10);
              const act = parseInt(im[2], 10);
              setInferences(inf);
              setActions(act);

              // Throughput bucket
              const t10 = Math.floor(Date.now() / 10000);
              if (bucketRef.current.start !== t10) {
                if (bucketRef.current.start) {
                  setBuckets(prev => {
                    const next = [...prev, { count: bucketRef.current.count }];
                    return next.length > 60 ? next.slice(next.length - 60) : next;
                  });
                }
                bucketRef.current = { start: t10, count: 25 };
              } else {
                bucketRef.current.count += 25;
              }
            }

            // Policy path
            const pm = line.match(/\[DEPLOY\].*?policy.*?path[:\s]+(\S+)/i);
            if (pm) setPolicyPath(pm[1]);

            // Session close detection
            if (/deploy.*stopped|session.*ended/i.test(line) && inferences > prevSessionInfer.current) {
              setSessions(prev => [{
                time: new Date().toLocaleTimeString(),
                inferences,
                policy: policyPath || '—'
              }, ...prev].slice(0, 20));
              prevSessionInfer.current = inferences;
            }
          }
          break;
        }

        case 'dataset_info': {
          const { totalEpisodes, totalFrames, savedPath } = msg as Record<string, unknown>;
          const parts: string[] = [];
          if (totalEpisodes != null) parts.push(`${totalEpisodes} episodes`);
          if (totalFrames   != null) parts.push(`${fmtStep(totalFrames as number)} frames`);
          if (savedPath)             parts.push(savedPath as string);
          setDatasetInfo(parts.join(' · ') || null);
          break;
        }
      }
    }

    window.addEventListener('message', onMessage);
    // Signal ready so extension host sends init
    try { (window as unknown as { acquireVsCodeApi?: () => { postMessage: (m: unknown) => void } }).acquireVsCodeApi?.()?.postMessage({ command: 'lerobot.ready' }); } catch { /* not in VS Code */ }

    return () => window.removeEventListener('message', onMessage);
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // ── Chart effects ──────────────────────────────────────────────────────────

  useEffect(() => {
    if (tab !== 'training' || !lossCanvasRef.current) return;
    const id = requestAnimationFrame(() => {
      if (lossCanvasRef.current) drawLossChart(lossCanvasRef.current, metricsRef.current, checkpointStepsRef.current);
    });
    return () => cancelAnimationFrame(id);
  }, [tab, metrics, checkpointSteps.size]);

  useEffect(() => {
    if (tab !== 'deploy' || !throughputCanvasRef.current) return;
    const id = requestAnimationFrame(() => {
      if (throughputCanvasRef.current) drawBarChart(throughputCanvasRef.current, bucketsRef.current);
    });
    return () => cancelAnimationFrame(id);
  }, [tab, buckets]);

  useEffect(() => {
    if (!lossCanvasRef.current) return;
    const obs = new ResizeObserver(() => {
      if (lossCanvasRef.current) drawLossChart(lossCanvasRef.current, metricsRef.current, checkpointStepsRef.current);
    });
    obs.observe(lossCanvasRef.current);
    return () => obs.disconnect();
  }, []);

  // ── Loss delta ─────────────────────────────────────────────────────────────

  const deltaEl = () => {
    if (loss === null || lossHist.length === 0) return null;
    const ref = lossHist.find(h => h.step <= step - 200);
    if (!ref) return null;
    const d = loss - ref.loss;
    if (Math.abs(d) < 1e-5) return <span className="lr-delta lr-delta--flat">→</span>;
    if (d < 0) return <span className="lr-delta lr-delta--down">↓{Math.abs(d).toFixed(4)}</span>;
    return <span className="lr-delta lr-delta--up">↑{d.toFixed(4)}</span>;
  };

  // ── Render ─────────────────────────────────────────────────────────────────

  return (
    <div className="lr">
      {/* ── Header ── */}
      <header className="lr-header">
        <span className="lr-header__title">LeRobot Monitor</span>
        {workspacePath && <span className="lr-header__path">{workspacePath}</span>}
        <div className="lr-conns">
          <span className="lr-conn">
            <span className={`lr-dot ${trainWatching ? 'lr-dot--ok lr-dot--pulse' : ''}`} />
            Training
          </span>
          <span className="lr-conn">
            <span className={`lr-dot ${ros.status === 'connected' ? 'lr-dot--ok' : ros.status === 'connecting' ? 'lr-dot--warn' : ''}`} />
            ROS
          </span>
        </div>
      </header>

      {/* ── Tab bar ── */}
      <div className="lr-tabs">
        {(['teleop', 'training', 'deploy'] as Tab[]).map(t => (
          <button
            key={t}
            className={`lr-tab ${tab === t ? 'lr-tab--active' : ''}`}
            onClick={() => setTab(t)}
          >
            {t.charAt(0).toUpperCase() + t.slice(1)}
          </button>
        ))}
      </div>

      {/* ── Content ── */}
      <div className="lr-content">

        {/* ════════ TELEOP ════════ */}
        <div className={`lr-pane ${tab === 'teleop' ? 'lr-pane--active' : ''}`}>

          {/* Joint states */}
          <div className="lr-card">
            <div className="lr-card__label">Joint States</div>
            <div className="lr-joints">
              {JOINT_NAMES.map((name, i) => {
                const pos = joints[i] ?? 0;
                const vel = jointVel[i] ?? 0;
                const [lo, hi] = JOINT_LIMITS[i];
                const norm = (pos - lo) / (hi - lo);
                const pct  = Math.max(0, Math.min(100, norm * 100));
                const centerPct = 50;
                const fillLeft  = Math.min(pct, centerPct);
                const fillWidth = Math.abs(pct - centerPct);
                return (
                  <div className="lr-joint" key={i}>
                    <span className="lr-joint__name">J{i + 1} {name}</span>
                    <div className="lr-joint__track">
                      <div
                        className={`lr-joint__fill ${vel > 0.01 ? 'lr-joint__fill--active' : ''}`}
                        style={{ left: `${fillLeft}%`, width: `${fillWidth}%` }}
                      />
                    </div>
                    <span className="lr-joint__val">{joints[i] !== null ? pos.toFixed(3) : '—'}</span>
                  </div>
                );
              })}
            </div>
          </div>

          <div className="lr-two">
            {/* Cameras */}
            <div className="lr-card">
              <div className="lr-card__label">Camera Topics</div>
              <div className="lr-cams">
                {CAM_LABELS.map((label, i) => (
                  <div className="lr-cam" key={i}>
                    <span className={`lr-dot ${dotClass(camLastMs[i], now)}`} />
                    <span className="lr-cam__name">{label}</span>
                    <span className="lr-cam__age">{ageText(camLastMs[i], now)}</span>
                  </div>
                ))}
              </div>
            </div>

            {/* Recording + dataset */}
            <div className="lr-card">
              <div className="lr-card__label">Recording</div>
              <span className={`lr-badge ${recording === 'recording' ? 'lr-badge--recording' : recording === 'waiting' ? 'lr-badge--waiting' : ''}`}>
                {recording === 'recording' ? '● Recording' : recording === 'waiting' ? '◐ Waiting' : 'Idle'}
              </span>
              {datasetInfo && (
                <div style={{ marginTop: 8, fontSize: '0.75rem', opacity: 0.7, lineHeight: 1.5 }}>
                  {datasetInfo}
                </div>
              )}
            </div>
          </div>
        </div>

        {/* ════════ TRAINING ════════ */}
        <div className={`lr-pane ${tab === 'training' ? 'lr-pane--active' : ''}`}>

          {/* Metric cards */}
          <div className="lr-metrics">
            <div className="lr-metric">
              <div className="lr-metric__label">Step</div>
              <div className="lr-metric__val">{step > 0 ? fmtStep(step) : '—'}</div>
              {totalSteps > 0 && <div className="lr-metric__sub">of {fmtStep(totalSteps)}</div>}
            </div>
            <div className="lr-metric">
              <div className="lr-metric__label">Loss</div>
              <div className="lr-metric__val">
                {loss !== null ? loss.toFixed(4) : '—'}
                {deltaEl()}
              </div>
            </div>
            <div className="lr-metric">
              <div className="lr-metric__label">Grad Norm</div>
              <div className="lr-metric__val">{gradNorm !== null ? gradNorm.toFixed(4) : '—'}</div>
            </div>
            <div className="lr-metric">
              <div className="lr-metric__label">Epoch</div>
              <div className="lr-metric__val">{epoch !== null ? epoch.toFixed(2) : '—'}</div>
              {datasetFrames && <div className="lr-metric__sub">{fmtStep(datasetFrames)} frames</div>}
            </div>
          </div>

          {/* Progress bar */}
          {totalSteps > 0 && step > 0 && (() => {
            const pct = Math.min(100, (step / totalSteps) * 100);
            return (
              <div className="lr-progress">
                <div className="lr-progress__labels">
                  <span>Step {fmtStep(step)}</span>
                  <span>{pct.toFixed(1)}%</span>
                </div>
                <div className="lr-progress__track">
                  <div className="lr-progress__fill" style={{ width: `${pct}%` }} />
                </div>
              </div>
            );
          })()}

          {/* Loss chart */}
          <div className="lr-card">
            <div className="lr-card__label">Training Loss</div>
            <div className="lr-chart-wrap">
              <canvas ref={lossCanvasRef} />
            </div>
          </div>

          {/* Config */}
          {configText && (
            <div className="lr-card">
              <button className="lr-config-toggle" onClick={() => setConfigOpen(o => !o)}>
                <span>{configOpen ? '▼' : '▶'}</span> Config
              </button>
              <pre className={`lr-config-body ${configOpen ? 'lr-config-body--open' : ''}`}>
                {configText}
              </pre>
            </div>
          )}

          {/* Training log */}
          <div>
            <div className="lr-log__head">
              <span className="lr-card__label" style={{ marginBottom: 0 }}>Live Log</span>
              <button className="lr-btn-clear" onClick={() => setTrainLog([])}>Clear</button>
            </div>
            <div className="lr-log__body">
              {trainLog.map((l, i) => (
                <div key={i} className={`lr-log__line lr-log__line--${l.cls}`}>{l.text}</div>
              ))}
            </div>
          </div>

          {/* Checkpoints */}
          <div className="lr-card">
            <div className="lr-card__label">Checkpoints</div>
            {checkpoints.length === 0
              ? <div className="lr-empty">No checkpoints yet.</div>
              : (
                <table className="lr-table">
                  <thead>
                    <tr><th>Step</th><th>Loss</th><th>Grad Norm</th><th>Saved at</th></tr>
                  </thead>
                  <tbody>
                    {checkpoints.map(c => (
                      <tr key={c.step}>
                        <td>{fmtStep(c.step)}</td>
                        <td>{c.loss != null ? c.loss.toFixed(4) : '—'}</td>
                        <td>{c.gradNorm != null ? c.gradNorm.toFixed(4) : '—'}</td>
                        <td>{c.savedAt}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              )
            }
          </div>
        </div>

        {/* ════════ DEPLOY ════════ */}
        <div className={`lr-pane ${tab === 'deploy' ? 'lr-pane--active' : ''}`}>

          <div className="lr-two">
            {/* Inference counter */}
            <div className="lr-card">
              <div className="lr-card__label">Inferences</div>
              <div className="lr-infer-num">{inferences.toLocaleString()}</div>
              <div className="lr-infer-sub">Actions sent: <strong>{actions.toLocaleString()}</strong></div>
            </div>

            {/* Policy info */}
            <div className="lr-card">
              <div className="lr-card__label">Policy</div>
              {policyPath
                ? <code style={{ fontSize: '0.75rem', wordBreak: 'break-all', opacity: 0.85 }}>{policyPath}</code>
                : <div className="lr-empty">Waiting for deploy script…</div>
              }
            </div>
          </div>

          {/* Throughput chart */}
          <div className="lr-card">
            <div className="lr-card__label">Inference Throughput (10 s windows)</div>
            <div className="lr-bar-wrap">
              <canvas ref={throughputCanvasRef} />
            </div>
          </div>

          {/* Active topics */}
          <div className="lr-card">
            <div className="lr-card__label">Active Topics</div>
            <div className="lr-topics">
              {DEPLOY_TOPICS.map((t, i) => (
                <div className="lr-topic" key={i}>
                  <span className={`lr-dot ${dotClass(topicLastMs[i], now)}`} />
                  <span className="lr-topic__name">{t.name}</span>
                  <span className="lr-topic__dir">{t.dir}</span>
                  <span className="lr-topic__age">{ageText(topicLastMs[i], now)}</span>
                </div>
              ))}
            </div>
          </div>

          {/* Deploy log */}
          <div>
            <div className="lr-log__head">
              <span className="lr-card__label" style={{ marginBottom: 0 }}>Deploy Log</span>
              <button className="lr-btn-clear" onClick={() => setDeployLog([])}>Clear</button>
            </div>
            <div className="lr-log__body">
              {deployLog.map((l, i) => (
                <div key={i} className={`lr-log__line lr-log__line--${l.cls}`}>{l.text}</div>
              ))}
            </div>
          </div>

          {/* Session history */}
          <div className="lr-card">
            <div className="lr-card__label">Session History</div>
            {sessions.length === 0
              ? <div className="lr-empty">No sessions yet.</div>
              : (
                <table className="lr-table">
                  <thead>
                    <tr><th>Time</th><th>Inferences</th><th>Policy</th></tr>
                  </thead>
                  <tbody>
                    {sessions.map((s, i) => (
                      <tr key={i}>
                        <td>{s.time}</td>
                        <td>{s.inferences.toLocaleString()}</td>
                        <td style={{ maxWidth: 160, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>{s.policy}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              )
            }
          </div>
        </div>

      </div>{/* /lr-content */}
    </div>
  );
}
