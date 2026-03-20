/**
 * LeRobot / SO-ARM101 monitoring panel — extension-host side.
 *
 * Responsibilities (things that MUST live in the extension host):
 *   - Detecting a LeRobot workspace (.tensorfleet-robotic / .tensorfleet template)
 *   - Reading ROSBRIDGE_URL from the workspace .env
 *   - Scanning outputs/train/ for existing checkpoints on disk
 *   - File-system watching outputs/train/** → parse log lines → postMessage to webview
 *
 * The panel UI itself lives in panels-standalone/src/components/LeRobot/.
 */

import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';

// ── Panel registry (keyed by workspace folder path) ───────────────────────────

const openPanels = new Map<string, vscode.WebviewPanel>();

// ── Project detection ─────────────────────────────────────────────────────────

/** Returns the first workspace folder that is a LeRobot project, or null. */
export async function findLerobotWorkspace(): Promise<string | null> {
  for (const folder of vscode.workspace.workspaceFolders ?? []) {
    // Prefer explicit .tensorfleet-robotic marker
    try {
      await vscode.workspace.fs.stat(vscode.Uri.joinPath(folder.uri, '.tensorfleet-robotic'));
      return folder.uri.fsPath;
    } catch { /* not found */ }

    // Fall back: .tensorfleet JSON with template === "lerobot-arm"
    try {
      const raw = await vscode.workspace.fs.readFile(vscode.Uri.joinPath(folder.uri, '.tensorfleet'));
      const meta = JSON.parse(Buffer.from(raw).toString('utf8')) as { template?: string };
      if (meta.template === 'lerobot-arm') return folder.uri.fsPath;
    } catch { /* not found or not valid JSON */ }
  }
  return null;
}

// ── Env / checkpoint helpers ──────────────────────────────────────────────────

/** Read ROSBRIDGE_URL from the workspace .env (if present). */
function readRosbridgeUrl(workspaceFolder: string): string {
  try {
    const envFile = path.join(workspaceFolder, '.env');
    if (!fs.existsSync(envFile)) return '';
    for (const line of fs.readFileSync(envFile, 'utf8').split('\n')) {
      const m = line.match(/^\s*ROSBRIDGE_URL\s*=\s*(.+)/);
      if (m) return m[1].trim().replace(/^["']|["']$/g, '');
    }
  } catch { /* ignore */ }
  return '';
}

/** Scan outputs/train/<job>/checkpoints/ for step directories. */
function scanCheckpoints(workspaceFolder: string): Array<{ step: number; savedAt: string }> {
  const out: Array<{ step: number; savedAt: string }> = [];
  try {
    const trainDir = path.join(workspaceFolder, 'outputs', 'train');
    if (!fs.existsSync(trainDir)) return out;
    for (const job of fs.readdirSync(trainDir)) {
      const cpDir = path.join(trainDir, job, 'checkpoints');
      if (!fs.existsSync(cpDir)) continue;
      for (const entry of fs.readdirSync(cpDir)) {
        if (entry === 'last') continue;
        const step = parseInt(entry, 10);
        if (isNaN(step)) continue;
        let savedAt = '';
        try { savedAt = fs.statSync(path.join(cpDir, entry)).mtime.toLocaleString(); } catch { /* ok */ }
        out.push({ step, savedAt });
      }
    }
  } catch { /* ignore */ }
  return out.sort((a, b) => a.step - b.step);
}

// ── File watcher → webview messages ──────────────────────────────────────────

const METRIC_RE = /step:(\d+[Kk]?)\s+smpl:(\S+)\s+ep:(\d+)\s+epch:([\d.]+)\s+loss:([\d.]+)\s+grdn:([\d.]+)/;
const STEPS_RE  = /cfg\.steps=(\d+)/;
const FRAMES_RE = /dataset\.num_frames=(\d+)/;
const CKPT_RE   = /Checkpoint policy after step (\d+)/;
const DEPLOY_RE = /\[DEPLOY\]/i;

function watchTrainingLogs(workspaceFolder: string, panel: vscode.WebviewPanel): vscode.Disposable {
  const offsets = new Map<string, number>();

  function parseTail(filePath: string, mode: 'train' | 'deploy') {
    try {
      const size = fs.statSync(filePath).size;
      const offset = offsets.get(filePath) ?? 0;
      if (size <= offset) return;
      const buf = Buffer.alloc(size - offset);
      const fd = fs.openSync(filePath, 'r');
      fs.readSync(fd, buf, 0, buf.length, offset);
      fs.closeSync(fd);
      offsets.set(filePath, size);

      for (const line of buf.toString('utf8').split('\n')) {
        const t = line.trim();
        if (!t) continue;
        const lmode = DEPLOY_RE.test(t) ? 'deploy' : mode;
        panel.webview.postMessage({ type: 'stdout', mode: lmode, line: t });
        if (lmode !== 'train') continue;

        const m = METRIC_RE.exec(t);
        if (m) {
          const raw = m[1];
          panel.webview.postMessage({
            type: 'metrics',
            step: /[Kk]$/.test(raw) ? parseInt(raw) * 1000 : parseInt(raw),
            samples: m[2], episode: parseInt(m[3], 10),
            epoch: parseFloat(m[4]), loss: parseFloat(m[5]), gradNorm: parseFloat(m[6]),
          });
        }
        const sm = STEPS_RE.exec(t);
        if (sm) panel.webview.postMessage({ type: 'config', totalSteps: parseInt(sm[1], 10) });
        const fm = FRAMES_RE.exec(t);
        if (fm) panel.webview.postMessage({ type: 'config', datasetFrames: parseInt(fm[1], 10) });
        const cm = CKPT_RE.exec(t);
        if (cm) panel.webview.postMessage({ type: 'checkpoint', step: parseInt(cm[1], 10) });
      }
    } catch { /* file not ready yet */ }
  }

  function scanExistingLogs() {
    const trainDir = path.join(workspaceFolder, 'outputs', 'train');
    if (!fs.existsSync(trainDir)) return;
    for (const job of fs.readdirSync(trainDir)) {
      for (const name of ['train.log', 'output.log', 'lerobot.log']) {
        const f = path.join(trainDir, job, name);
        if (fs.existsSync(f)) parseTail(f, 'train');
      }
    }
  }

  const watcher = vscode.workspace.createFileSystemWatcher(
    new vscode.RelativePattern(workspaceFolder, 'outputs/train/**')
  );
  watcher.onDidChange(uri => parseTail(uri.fsPath, DEPLOY_RE.test(uri.fsPath) ? 'deploy' : 'train'));
  watcher.onDidCreate(uri => {
    parseTail(uri.fsPath, DEPLOY_RE.test(uri.fsPath) ? 'deploy' : 'train');
    panel.webview.postMessage({ type: 'checkpoints_scan', checkpoints: scanCheckpoints(workspaceFolder) });
  });

  panel.webview.postMessage({ type: 'watching', active: true });
  setTimeout(scanExistingLogs, 600);
  return watcher;
}

// ── Panel entry point ─────────────────────────────────────────────────────────

/**
 * Open (or reveal) the AI Model Ops panel.
 * If a LeRobot workspace is found it loads the React monitor; otherwise shows a plain hint.
 *
 * @param context  Extension context (for resource roots + token/nodeId injection).
 * @param getHtml  Delegate to extension.ts's getStandalonePanelHtml — keeps the file reading
 *                 and TENSORFLEET global injection logic in one place.
 * @param noProjectHtml  Delegate for the "no project" fallback HTML.
 */
export async function openLerobotAIPanel(
  context: vscode.ExtensionContext,
  getHtml: (webview: vscode.Webview, csp: string) => Promise<string>,
  noProjectHtml: (webview: vscode.Webview) => string,
): Promise<void> {
  const workspaceFolder = await findLerobotWorkspace();
  const panelKey = workspaceFolder ?? '__no-project__';

  // Reveal existing panel if open
  const existing = openPanels.get(panelKey);
  if (existing) { existing.reveal(); return; }

  const roots = [
    vscode.Uri.joinPath(context.extensionUri, 'media'),
    vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist'),
    vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist', 'assets'),
  ];

  const panel = vscode.window.createWebviewPanel(
    'tensorfleetPanel.tensorfleet-aiops',
    workspaceFolder ? 'AI Model Ops · LeRobot' : 'AI Model Ops',
    vscode.ViewColumn.Active,
    { enableScripts: true, retainContextWhenHidden: true, localResourceRoots: roots },
  );
  openPanels.set(panelKey, panel);
  panel.onDidDispose(() => openPanels.delete(panelKey));

  if (!workspaceFolder) {
    panel.webview.html = noProjectHtml(panel.webview);
    return;
  }

  // Inject LeRobot-specific globals on top of the TENSORFLEET ones
  const rosbridgeUrl = readRosbridgeUrl(workspaceFolder) || 'ws://localhost:9090';
  let html = await getHtml(panel.webview, panel.webview.cspSource);
  html = html.replace('</head>', `<script>
    window.LEROBOT_ROSBRIDGE_URL = ${JSON.stringify(rosbridgeUrl)};
    window.LEROBOT_WORKSPACE_FOLDER = ${JSON.stringify(workspaceFolder)};
  </script>\n</head>`);
  panel.webview.html = html;

  const checkpoints = scanCheckpoints(workspaceFolder);
  const watcher = watchTrainingLogs(workspaceFolder, panel);
  panel.onDidDispose(() => watcher.dispose());

  const sendInit = () => panel.webview.postMessage({
    type: 'init', workspaceFolder, rosbridgeUrl, checkpoints,
  });
  panel.webview.onDidReceiveMessage(msg => { if (msg.command === 'lerobot.ready') sendInit(); });
  setTimeout(sendInit, 400);
}
