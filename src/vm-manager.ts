import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';
import * as http from 'http';
import * as https from 'https';
import * as os from 'os';
import * as net from 'net';
import { ChildProcess, spawn, spawnSync } from 'child_process';

interface VmRecord {
  id: string;
  name: string;
  status: string;
  user_id?: string;
  ip_address?: string;
  provider?: string;
  region?: string;
  created_at?: string;
}

interface VmListResponse {
  count: number;
  vms: VmRecord[];
}

interface VmCreateResponse {
  message: string;
  vm: VmRecord;
}

interface VmStopResponse {
  message: string;
  vm_id: string;
  status: string;
}

interface VmManagerMessage {
  command: string;
  payload?: any;
}

interface VmManagerEnvironment {
  id: string;
  label: string;
  apiBaseUrl: string;
  authEndpoint?: string;
  defaultEmail?: string;
}

interface LoginMetadataEntry {
  email?: string;
  timestamp?: number;
  userId?: string;
}

interface RequestOptions {
  skipAuth?: boolean;
  baseUrl?: string;
  fullUrl?: string;
  headers?: http.OutgoingHttpHeaders;
}

const SECRET_TOKEN_PREFIX = 'tensorfleet.vmManager.token';
const LOGIN_METADATA_KEY = 'tensorfleet.vmManager.loginMetadata';

export class VMManagerIntegration {
  private process: ChildProcess | null = null;
  private pendingShutdown: Promise<void> | null = null;
  private sudoTerminal: vscode.Terminal | null = null;
  private readonly outputChannel: vscode.OutputChannel;
  private readonly templatePath: string;
  private readonly webviews = new Set<vscode.Webview>();
  private resolvedRepoPath: string | null = null;
  private repoWarningShown = false;
  private readonly authWarningIssued = new Set<string>();
  private baseRootfsWarningShown = false;

  constructor(private readonly context: vscode.ExtensionContext) {
    this.outputChannel = vscode.window.createOutputChannel('TensorFleet VM Manager');
    this.templatePath = path.join(__dirname, '..', 'src', 'templates', 'vm-manager.html');
    this.context.subscriptions.push(
      vscode.window.onDidCloseTerminal((terminal) => {
        if (terminal === this.sudoTerminal) {
          this.sudoTerminal = null;
          void this.broadcastState();
        }
      })
    );
  }

  registerPanel(panel: vscode.WebviewPanel) {
    this.webviews.add(panel.webview);
    const disposable = panel.onDidDispose(() => {
      this.webviews.delete(panel.webview);
    });
    this.context.subscriptions.push(disposable);
  }

  getPanelHtml(_webview: vscode.Webview, cspSource: string): string {
    let template = fs.readFileSync(this.templatePath, 'utf8');
    template = template.replace(/{{cspSource}}/g, cspSource);
    return template;
  }

  async start() {
    if (this.pendingShutdown) {
      this.outputChannel.appendLine('[VM Manager] Waiting for previous process to stop before starting a new one.');
      try {
        await this.pendingShutdown;
      } catch (error) {
        this.outputChannel.appendLine(
          `[VM Manager] Previous shutdown reported an error: ${this.formatError(error)}`
        );
      }
    }

    if (this.process) {
      vscode.window.showInformationMessage('TensorFleet VM Manager is already running.');
      void this.broadcastState();
      return;
    }

    if (this.sudoTerminal) {
      vscode.window.showInformationMessage(
        'TensorFleet VM Manager is already running via the sudo terminal. Stop it before starting another instance.'
      );
      void this.broadcastState();
      return;
    }

    const repoPath = this.resolveRepoPath();
    if (!repoPath) {
      vscode.window.showWarningMessage(
        'Local vm-manager repo not detected. Local service controls are for dev mode only. Configure tensorfleet.vmManager.repoPath if you need to run the Go service yourself.'
      );
      void this.broadcastState();
      return;
    }

    const goCommand = String(process.platform) === 'win32' ? 'go.exe' : 'go';
    if (!this.ensureGoAvailable(goCommand)) {
      return;
    }
    this.outputChannel.appendLine(`[VM Manager] Starting at ${new Date().toISOString()}`);
    this.outputChannel.appendLine(`[VM Manager] Working directory: ${repoPath}`);

    const spawnEnv = await this.buildLocalServiceEnv(repoPath);
    const serverPort = this.getServerPort(spawnEnv);
    if (!(await this.ensureServerPortAvailable(serverPort))) {
      await this.broadcastState();
      return;
    }

    try {
      this.process = spawn(goCommand, ['run', './cmd/vm-manager'], {
        cwd: repoPath,
        env: spawnEnv
      });
    } catch (error) {
      this.process = null;
      vscode.window.showErrorMessage(`Failed to start VM Manager: ${this.formatError(error)}`);
      return;
    }

    let portConflictWarned = false;
    let tapPermissionPrompted = false;
    this.process.stdout?.on('data', (data) => {
      this.outputChannel.append(data.toString());
    });

    this.process.stderr?.on('data', (data) => {
      const text = data.toString();
      this.outputChannel.append(text);
      if (!portConflictWarned && /address already in use/i.test(text)) {
        portConflictWarned = true;
        vscode.window.showErrorMessage(
          'TensorFleet VM Manager could not bind one of its ports (often the ZMQ proxy on tcp://*:5556). Stop any lingering vm-manager processes or other tools using that port and try again.'
        );
      }
      if (!tapPermissionPrompted && this.isTapPermissionError(text)) {
        tapPermissionPrompted = true;
        this.showSudoNetworkingPrompt();
      }
    });

    this.process.on('error', (error) => {
      const message = this.formatError(error);
      this.outputChannel.appendLine(`[VM Manager] Process error: ${message}`);
      vscode.window.showErrorMessage(`TensorFleet VM Manager failed: ${message}`);
      this.process = null;
      void this.broadcastState();
    });

    this.process.on('exit', (code) => {
      this.outputChannel.appendLine(`[VM Manager] Process exited with code ${code ?? 'unknown'}`);
      this.process = null;
      void this.broadcastState();
    });

    vscode.window.showInformationMessage('TensorFleet VM Manager starting…');
    this.outputChannel.show(true);
    void this.broadcastState();
  }

  async stop(options: { quiet?: boolean } = {}) {
    if (this.pendingShutdown) {
      if (!options.quiet) {
        vscode.window.showInformationMessage('TensorFleet VM Manager is already stopping. Waiting for shutdown to finish.');
      }
      try {
        await this.pendingShutdown;
      } catch (error) {
        this.outputChannel.appendLine(`[VM Manager] Previous shutdown failed: ${this.formatError(error)}`);
      }
      return;
    }

    if (!this.process) {
      if (this.sudoTerminal) {
        await this.stopSudoTerminal(options);
        return;
      }
      if (!options.quiet) {
        vscode.window.showInformationMessage('TensorFleet VM Manager is not running.');
      }
      return;
    }

    const child = this.process;
    this.process = null;
    const shutdownPromise = this.terminateProcessTree(child);
    this.pendingShutdown = shutdownPromise;

    if (!options.quiet) {
      vscode.window.showInformationMessage('TensorFleet VM Manager is stopping…');
    }

    try {
      await shutdownPromise;
      this.outputChannel.appendLine('[VM Manager] Local service stopped.');
    } catch (error) {
      this.outputChannel.appendLine(`[VM Manager] Failed to stop process: ${this.formatError(error)}`);
      if (!options.quiet) {
        vscode.window.showErrorMessage(`TensorFleet VM Manager failed to stop: ${this.formatError(error)}`);
      }
    } finally {
      this.pendingShutdown = null;
      await this.broadcastState();
    }
  }

  showLogs() {
    this.outputChannel.show(true);
  }

  isRunning() {
    return Boolean(this.process);
  }

  handleWebviewMessage(viewId: string, message: VmManagerMessage, webview: vscode.Webview): boolean {
    if (viewId !== 'tensorfleet-vm-manager') {
      return false;
    }

    switch (message.command) {
      case 'vmManager:getStatus':
        void this.sendState(webview);
        break;
      case 'vmManager:refresh':
        this.fetchVMs()
          .then((response) => {
            webview.postMessage({ type: 'vmManager:vms', payload: response });
          })
          .catch((error) => this.sendError(webview, error));
        break;
      case 'vmManager:create':
        this.createVM(message.payload)
          .then((response) => {
            const vmName = response.vm?.name ?? response.vm?.id ?? 'VM';
            webview.postMessage({
              type: 'vmManager:success',
              payload: response.message || `${vmName} created`
            });
            return this.fetchVMs();
          })
          .then((response) => {
            webview.postMessage({ type: 'vmManager:vms', payload: response });
          })
          .catch((error) => this.sendError(webview, error));
        break;
      case 'vmManager:stopVm':
        if (!message.payload?.id) {
          this.sendError(webview, new Error('Missing VM ID'));
          break;
        }
        this.stopVm(message.payload.id)
          .then((result) => {
            webview.postMessage({
              type: 'vmManager:success',
              payload: result.message || 'VM stop requested'
            });
            return this.fetchVMs();
          })
          .then((response) => {
            webview.postMessage({ type: 'vmManager:vms', payload: response });
          })
          .catch((error) => this.sendError(webview, error));
        break;
      case 'vmManager:startService':
        void this.start();
        break;
      case 'vmManager:startServiceWithSudo':
        void this.startServiceWithSudoTerminal();
        break;
      case 'vmManager:stopService':
        void this.stop();
        break;
      case 'vmManager:openLogs':
        this.outputChannel.show(true);
        break;
      case 'vmManager:setEnvironment':
        if (!message.payload?.id || typeof message.payload.id !== 'string') {
          this.sendError(webview, new Error('Missing environment id'));
          break;
        }
        this.applyEnvironmentSelection(message.payload.id, { silent: true })
          .then(() => {
            webview.postMessage({
              type: 'vmManager:success',
              payload: 'Environment updated.'
            });
            return this.fetchVMs();
          })
          .then((response) => {
            if (response) {
              webview.postMessage({ type: 'vmManager:vms', payload: response });
            }
          })
          .catch((error) => this.sendError(webview, error));
        break;
      case 'vmManager:login':
        this.loginFromWebview(message.payload, webview);
        break;
      case 'vmManager:logout':
        this.logout(webview).catch((error) => this.sendError(webview, error));
        break;
      case 'vmManager:setApiBaseUrl':
        if (!message.payload?.apiBaseUrl) {
          this.sendError(webview, new Error('Missing API URL'));
          break;
        }
        void this.updateApiBaseUrl(message.payload.apiBaseUrl, message.payload.environmentId, webview)
          .catch((error) => this.sendError(webview, error));
        break;
      default:
        return false;
    }

    return true;
  }

  private async startServiceWithSudoTerminal() {
    if (this.process) {
      vscode.window.showInformationMessage('TensorFleet VM Manager already running via VS Code.');
      return;
    }

    if (this.sudoTerminal) {
      this.sudoTerminal.show(true);
      vscode.window.showInformationMessage('TensorFleet VM Manager sudo terminal is already open.');
      return;
    }

    if (String(process.platform) === 'win32') {
      vscode.window.showWarningMessage('Starting TensorFleet VM Manager with sudo is only supported on Unix-like systems.');
      return;
    }

    const repoPath = this.resolveRepoPath();
    if (!repoPath) {
      vscode.window.showWarningMessage(
        'Local vm-manager repo not detected. Configure tensorfleet.vmManager.repoPath before starting with sudo.'
      );
      return;
    }

    const goCommand = String(process.platform) === 'win32' ? 'go.exe' : 'go';
    const envOverrides = await this.getLocalServiceEnvOverrides(repoPath);
    const terminalEnv = {
      ...process.env,
      LOCAL_DEV: '1',
      ...envOverrides
    };
    const serverPort = this.getServerPort(terminalEnv);
    if (!(await this.ensureServerPortAvailable(serverPort))) {
      return;
    }
    const terminal = vscode.window.createTerminal({
      name: 'TensorFleet VM Manager (sudo)',
      cwd: repoPath,
      env: terminalEnv
    });

    this.sudoTerminal = terminal;
    terminal.show(true);
    terminal.sendText(`sudo -E ${goCommand} run ./cmd/vm-manager`);
    vscode.window.showInformationMessage('Opened terminal to start TensorFleet VM Manager with sudo.');
    void this.broadcastState();
  }

  private async stopSudoTerminal(options: { quiet?: boolean } = {}) {
    const terminal = this.sudoTerminal;
    if (!terminal) {
      return;
    }

    this.sudoTerminal = null;
    if (!options.quiet) {
      vscode.window.showInformationMessage('TensorFleet VM Manager (sudo terminal) is stopping…');
    }

    try {
      terminal.sendText('\u0003', false);
    } catch (error) {
      this.outputChannel.appendLine(
        `[VM Manager] Failed to send stop signal to sudo terminal: ${this.formatError(error)}`
      );
    }

    try {
      terminal.dispose();
    } catch (error) {
      this.outputChannel.appendLine(
        `[VM Manager] Failed to dispose sudo terminal: ${this.formatError(error)}`
      );
    }

    this.outputChannel.appendLine('[VM Manager] Sudo terminal closed.');
    await this.broadcastState();
  }

  private isTapPermissionError(output: string): boolean {
    if (!output) {
      return false;
    }
    const lower = output.toLowerCase();
    const permissionBlocked =
      lower.includes('operation not permitted') || lower.includes('permission denied');
    if (!permissionBlocked) {
      return false;
    }
    return lower.includes('tap') || lower.includes('tun') || lower.includes('tunsetiff');
  }

  private showSudoNetworkingPrompt() {
    const action = 'Open sudo terminal';
    vscode.window
      .showWarningMessage(
        'Firecracker networking needs root access to create TAP devices. Use "Start as root (terminal)" in the VM Manager panel or open it now.',
        action
      )
      .then((selection) => {
        if (selection !== action) {
          return;
        }
        void (async () => {
          try {
            await this.stop({ quiet: true });
          } catch (error) {
            this.outputChannel.appendLine(
              `[VM Manager] Failed to stop process before sudo restart: ${this.formatError(error)}`
            );
          }
          await this.startServiceWithSudoTerminal();
        })();
      });
  }

  dispose() {
    if (this.process || this.pendingShutdown || this.sudoTerminal) {
      void this.stop({ quiet: true }).finally(() => {
        this.outputChannel.dispose();
      });
      return;
    }
    this.outputChannel.dispose();
  }

  private async broadcastState() {
    const work = Array.from(this.webviews).map((webview) => this.sendState(webview));
    await Promise.all(work);
  }

  private async sendState(webview: vscode.Webview) {
    const environment = this.getActiveEnvironment();
    const environments = this.getConfiguredEnvironments();
    const hasToken = environment?.id ? Boolean(await this.getAuthToken(environment.id)) : false;
    const loginMetadata = environment?.id ? this.getLoginMetadataForEnv(environment.id) : undefined;
    const repoPath = this.resolveRepoPath();

    await webview.postMessage({
      type: 'vmManager:state',
      payload: {
        running: this.isRunning(),
        sudoTerminalActive: Boolean(this.sudoTerminal),
        apiBaseUrl: this.getApiBaseUrl(),
        repoPath: repoPath ?? undefined,
        localServiceAvailable: Boolean(repoPath),
        environment: environment
          ? { id: environment.id, label: environment.label, apiBaseUrl: environment.apiBaseUrl }
          : undefined,
        environments: environments.map((env) => ({
          id: env.id,
          label: env.label,
          apiBaseUrl: env.apiBaseUrl,
          defaultEmail: env.defaultEmail
        })),
        auth: {
          loggedIn: hasToken,
          email: loginMetadata?.email,
          lastLogin: loginMetadata?.timestamp ?? null,
          userId: loginMetadata?.userId
        }
      }
    });
  }

  private sendError(webview: vscode.Webview, error: unknown) {
    const message = this.formatError(error);
    webview.postMessage({ type: 'vmManager:error', payload: message });
    vscode.window.showErrorMessage(`[VM Manager] ${message}`);
  }

  private resolveRepoPath(): string | null {
    if (this.resolvedRepoPath && this.isValidRepoPath(this.resolvedRepoPath)) {
      return this.resolvedRepoPath;
    }

    const { path: candidate, searched } = this.findRepoPath();
    this.resolvedRepoPath = candidate;

    if (!candidate && !this.repoWarningShown) {
      const locations = searched.map((item) => `• ${item}`).join('\n');
      if (locations) {
        this.outputChannel.appendLine(`[VM Manager] Searched for vm-manager repo in:\n${locations}`);
      }
      this.repoWarningShown = true;
    }

    if (candidate) {
      this.repoWarningShown = false;
    }

    return candidate;
  }

  private findRepoPath(): { path: string | null; searched: string[] } {
    const config = vscode.workspace.getConfiguration('tensorfleet');
    const configuredPath = config.get<string>('vmManager.repoPath') ?? '../vm-manager';
    const envPath = process.env.TENSORFLEET_VM_MANAGER_PATH;
    const searched: string[] = [];
    const addCandidate = (inputPath: string | undefined) => {
      if (!inputPath) {
        return;
      }
      const normalized = path.isAbsolute(inputPath)
        ? path.normalize(inputPath)
        : path.resolve(this.context.extensionPath, inputPath);
      searched.push(normalized);
    };

    if (envPath) {
      addCandidate(envPath);
    }

    if (configuredPath) {
      if (path.isAbsolute(configuredPath)) {
        searched.push(path.normalize(configuredPath));
      } else {
        searched.push(path.resolve(this.context.extensionPath, configuredPath));
        if (vscode.workspace.workspaceFolders) {
          vscode.workspace.workspaceFolders.forEach((folder) => {
            searched.push(path.resolve(folder.uri.fsPath, configuredPath));
          });
        }
      }
    }

    if (!configuredPath?.includes('vm-manager')) {
      searched.push(path.resolve(this.context.extensionPath, '../vm-manager'));
      if (vscode.workspace.workspaceFolders) {
        vscode.workspace.workspaceFolders.forEach((folder) => {
          searched.push(path.join(folder.uri.fsPath, 'vm-manager'));
        });
      }
    }

    for (const candidate of searched) {
      if (this.isValidRepoPath(candidate)) {
        return { path: candidate, searched };
      }
    }

    return { path: null, searched };
  }

  private isValidRepoPath(candidate: string): boolean {
    try {
      const stats = fs.statSync(candidate);
      if (!stats.isDirectory()) {
        return false;
      }
      return fs.existsSync(path.join(candidate, 'cmd', 'vm-manager'));
    } catch {
      return false;
    }
  }

  private getApiBaseUrl() {
    return this.getActiveEnvironment()?.apiBaseUrl ?? this.getLegacyApiBaseUrl();
  }

  private getLegacyApiBaseUrl() {
    const config = vscode.workspace.getConfiguration('tensorfleet');
    return config.get<string>('vmManager.apiBaseUrl', 'http://localhost:8080');
  }

  private getConfiguredEnvironments(): VmManagerEnvironment[] {
    const config = vscode.workspace.getConfiguration('tensorfleet');
    const environments = config.get<VmManagerEnvironment[]>('vmManager.environments') ?? [];
    const sanitized = environments
      .filter((env): env is VmManagerEnvironment => Boolean(env?.id && env?.label && env?.apiBaseUrl))
      .map((env) => ({
        ...env,
        authEndpoint: env.authEndpoint || '/login'
      }));

    if (sanitized.length > 0) {
      return sanitized;
    }

    const fallbackUrl = this.getLegacyApiBaseUrl();
    return [
      {
        id: 'default',
        label: 'Default',
        apiBaseUrl: fallbackUrl,
        authEndpoint: '/login'
      }
    ];
  }

  private getActiveEnvironmentId(): string | undefined {
    const config = vscode.workspace.getConfiguration('tensorfleet');
    return config.get<string>('vmManager.activeEnvironment');
  }

  private getActiveEnvironment(): VmManagerEnvironment | undefined {
    const environments = this.getConfiguredEnvironments();
    if (environments.length === 0) {
      return undefined;
    }

    const activeId = this.getActiveEnvironmentId();
    const match = activeId ? environments.find((env) => env.id === activeId) : undefined;
    return match ?? environments[0];
  }

  private fetchVMs(): Promise<VmListResponse> {
    return this.request<VmListResponse>('GET', '/dev/vms');
  }

  private createVM(payload: any): Promise<VmCreateResponse> {
    return this.request<VmCreateResponse>('POST', '/dev/vms', payload);
  }

  private stopVm(id: string): Promise<VmStopResponse> {
    return this.request<VmStopResponse>('PATCH', `/dev/vms/${id}/stop`, {});
  }

  private async request<T>(method: string, endpoint: string, body?: any, options: RequestOptions = {}): Promise<T> {
    const environment = this.getActiveEnvironment();
    const baseReference = options.baseUrl ?? environment?.apiBaseUrl ?? this.getLegacyApiBaseUrl();
    const targetUrl = options.fullUrl ? new URL(options.fullUrl) : this.buildUrl(endpoint, baseReference);
    const isHttps = targetUrl.protocol === 'https:';
    const lib = isHttps ? https : http;
    const data = body ? JSON.stringify(body) : undefined;

    const headers: http.OutgoingHttpHeaders = {
      Accept: 'application/json',
      ...options.headers
    };

    if (data) {
      headers['Content-Type'] = 'application/json';
      headers['Content-Length'] = Buffer.byteLength(data);
    }

    if (!options.skipAuth && environment?.id) {
      const token = await this.getAuthToken(environment.id);
      if (token) {
        headers.Authorization = `Bearer ${token}`;
      }
    }

    return new Promise<T>((resolve, reject) => {
      const req = lib.request(
        {
          method,
          hostname: targetUrl.hostname,
          port: targetUrl.port || (isHttps ? 443 : 80),
          path: `${targetUrl.pathname}${targetUrl.search}`,
          headers
        },
        (res) => {
          const chunks: Buffer[] = [];
          res.on('data', (chunk) => chunks.push(chunk));
          res.on('end', () => {
            const bodyText = Buffer.concat(chunks).toString('utf8');
            if (res.statusCode && res.statusCode >= 200 && res.statusCode < 300) {
              if (!bodyText) {
                resolve(undefined as T);
                return;
              }

              try {
                const parsed = JSON.parse(bodyText);
                resolve(parsed);
              } catch (error) {
                reject(error);
              }
              return;
            }

            if (res.statusCode === 401 && !options.skipAuth && environment?.id) {
              void this.handleUnauthorized(environment.id);
            }

            const status = res.statusCode ?? 'unknown';
            reject(new Error(`Request failed (${status}): ${bodyText || res.statusMessage || 'Unknown error'}`));
          });
        }
      );

      req.on('error', (error) => {
        reject(this.createNetworkError(error, targetUrl));
      });
      req.setTimeout(5000, () => {
        req.destroy(new Error(`Request to ${targetUrl.toString()} timed out`));
      });

      if (data) {
        req.write(data);
      }
      req.end();
    });
  }

  async selectEnvironment() {
    const environments = this.getConfiguredEnvironments();
    if (environments.length === 0) {
      vscode.window.showWarningMessage('No VM Manager environments are configured. Update tensorfleet.vmManager.environments to continue.');
      return;
    }

    const active = this.getActiveEnvironment();
    const selection = await vscode.window.showQuickPick(
      environments.map((env) => ({
        label: env.label,
        description: env.apiBaseUrl,
        picked: env.id === active?.id,
        envId: env.id
      })),
      {
        placeHolder: 'Select TensorFleet VM Manager environment',
        ignoreFocusOut: true
      }
    );

    if (selection?.envId) {
      await this.applyEnvironmentSelection(selection.envId);
    }
  }

  async promptForLogin() {
    const environment = this.getActiveEnvironment();
    if (!environment) {
      vscode.window.showErrorMessage('No VM Manager environment is active.');
      return;
    }

    const metadata = environment.id ? this.getLoginMetadataForEnv(environment.id) : undefined;
    const email = await vscode.window.showInputBox({
      prompt: `Email for ${environment.label}`,
      placeHolder: 'you@example.com',
      value: metadata?.email ?? environment.defaultEmail ?? '',
      ignoreFocusOut: true
    });

    if (!email) {
      return;
    }

    const password = await vscode.window.showInputBox({
      prompt: `Password for ${environment.label}`,
      password: true,
      ignoreFocusOut: true
    });

    if (!password) {
      return;
    }

    await this.executeLogin(email.trim(), password);
  }

  async logout(webview?: vscode.Webview) {
    const environment = this.getActiveEnvironment();
    if (!environment) {
      vscode.window.showWarningMessage('No VM Manager environment is active.');
      return;
    }

    await this.setAuthToken(environment.id, null);
    await this.updateLoginMetadata(environment.id, null);
    this.authWarningIssued.delete(environment.id);

    const message = `Logged out of ${environment.label}`;
    this.outputChannel.appendLine(`[VM Manager] ${message}`);
    if (webview) {
      webview.postMessage({ type: 'vmManager:success', payload: message });
    } else {
      vscode.window.showInformationMessage(message);
    }
    await this.broadcastState();
  }

  private loginFromWebview(payload: any, webview: vscode.Webview) {
    const email = typeof payload?.email === 'string' ? payload.email.trim() : '';
    const password = typeof payload?.password === 'string' ? payload.password : '';
    if (!email || !password) {
      this.sendError(webview, new Error('Email and password are required to log in.'));
      return;
    }

    this.executeLogin(email, password, webview).catch((error) => this.sendError(webview, error));
  }

  private async executeLogin(email: string, password: string, webview?: vscode.Webview) {
    const environment = this.getActiveEnvironment();
    if (!environment) {
      throw new Error('No VM Manager environment configured.');
    }

    const authEndpoint = environment.authEndpoint || '/login';
    const requestOptions: RequestOptions = { skipAuth: true };
    const targetEndpoint = this.isAbsoluteUrl(authEndpoint) ? '/' : authEndpoint;
    if (this.isAbsoluteUrl(authEndpoint)) {
      requestOptions.fullUrl = authEndpoint;
    }

    const response = await this.request<any>('POST', targetEndpoint, { email, password }, requestOptions);
    const token = this.extractTokenFromResponse(response);
    if (!token) {
      throw new Error('Login response did not include an access token.');
    }

    const userId = this.extractUserIdFromResponse(response, token);
    await this.setAuthToken(environment.id, token);
    await this.updateLoginMetadata(environment.id, { email, timestamp: Date.now(), userId });
    this.authWarningIssued.delete(environment.id);
    const successMessage = `Logged in to ${environment.label} as ${email}`;
    this.outputChannel.appendLine(`[VM Manager] ${successMessage}`);

    if (webview) {
      webview.postMessage({ type: 'vmManager:success', payload: successMessage });
    } else {
      vscode.window.showInformationMessage(successMessage);
    }

    await this.broadcastState();
  }

  private extractTokenFromResponse(response: any): string | undefined {
    if (!response) {
      return undefined;
    }

    if (typeof response === 'string') {
      return response;
    }

    const tokenCandidates = [
      response.token,
      response.accessToken,
      response.access_token,
      response.jwt,
      response.session?.access_token,
      response.data?.token,
      response.data?.access_token
    ];

    for (const candidate of tokenCandidates) {
      if (typeof candidate === 'string' && candidate.trim()) {
        return candidate.trim();
      }
    }

    if (typeof response === 'object') {
      // Search nested objects for a token field
      for (const value of Object.values(response)) {
        if (typeof value === 'object') {
          const nested = this.extractTokenFromResponse(value);
          if (nested) {
            return nested;
          }
        }
      }
    }

    return undefined;
  }

  private extractUserIdFromResponse(response: any, token?: string): string | undefined {
    const candidates: Array<unknown> = [];
    if (response) {
      const nestedCandidates = [
        response.user?.id,
        response.user?.uuid,
        response.user?.user_id,
        response.user_id,
        response.userId,
        response.session?.user?.id,
        response.session?.user?.uuid,
        response.data?.user?.id,
        response.data?.user?.uuid
      ];
      candidates.push(...nestedCandidates);
    }

    for (const candidate of candidates) {
      if (typeof candidate === 'string' && candidate.trim()) {
        return candidate.trim();
      }
    }

    if (!token || !token.includes('.')) {
      return undefined;
    }

    const [, payloadSegment] = token.split('.', 2);
    if (!payloadSegment) {
      return undefined;
    }

    try {
      const normalized = payloadSegment.replace(/-/g, '+').replace(/_/g, '/');
      const padded = normalized.padEnd(Math.ceil(normalized.length / 4) * 4, '=');
      const decoded = Buffer.from(padded, 'base64').toString('utf8');
      const payload = JSON.parse(decoded);
      const jwtCandidates = [payload.sub, payload.user_id, payload.userId];
      for (const candidate of jwtCandidates) {
        if (typeof candidate === 'string' && candidate.trim()) {
          return candidate.trim();
        }
      }
    } catch (error) {
      this.outputChannel.appendLine(`[VM Manager] Failed to extract user id from token: ${this.formatError(error)}`);
    }

    return undefined;
  }

  private async applyEnvironmentSelection(envId: string, options: { silent?: boolean } = {}) {
    const environments = this.getConfiguredEnvironments();
    const target = environments.find((env) => env.id === envId);
    if (!target) {
      throw new Error(`Unknown environment: ${envId}`);
    }

    const config = vscode.workspace.getConfiguration('tensorfleet');
    await config.update('vmManager.activeEnvironment', envId, vscode.ConfigurationTarget.Global);
    this.outputChannel.appendLine(`[VM Manager] Active environment set to ${target.label} (${target.apiBaseUrl})`);
    this.authWarningIssued.delete(envId);
    if (!options.silent) {
      vscode.window.showInformationMessage(`TensorFleet VM Manager environment set to ${target.label}`);
    }

    await this.broadcastState();
  }

  private async updateApiBaseUrl(apiBaseUrl: string, environmentId?: string, webview?: vscode.Webview) {
    const normalized = apiBaseUrl.trim();
    if (!normalized) {
      throw new Error('API URL cannot be empty.');
    }
    const config = vscode.workspace.getConfiguration('tensorfleet');
    const environments = config.get<VmManagerEnvironment[]>('vmManager.environments') ?? [];
    if (environmentId) {
      const index = environments.findIndex((env) => env.id === environmentId);
      if (index >= 0) {
        const updated = [...environments];
        updated[index] = { ...updated[index], apiBaseUrl: normalized };
        await config.update('vmManager.environments', updated, vscode.ConfigurationTarget.Global);
        this.outputChannel.appendLine(`[VM Manager] Environment ${environmentId} API URL updated to ${normalized}`);
        await this.broadcastState();
        webview?.postMessage({ type: 'vmManager:success', payload: 'API URL updated.' });
        return;
      }
    }
    await config.update('vmManager.apiBaseUrl', normalized, vscode.ConfigurationTarget.Global);
    this.outputChannel.appendLine(`[VM Manager] API URL updated to ${normalized}`);
    await this.broadcastState();
    webview?.postMessage({ type: 'vmManager:success', payload: 'API URL updated.' });
  }

  private buildUrl(endpoint: string, baseUrl: string): URL {
    const normalizedBase = baseUrl.endsWith('/') ? baseUrl : `${baseUrl}/`;
    if (!endpoint) {
      return new URL(normalizedBase);
    }
    if (this.isAbsoluteUrl(endpoint)) {
      return new URL(endpoint);
    }
    return new URL(endpoint, normalizedBase);
  }

  private isAbsoluteUrl(value?: string): boolean {
    return typeof value === 'string' && /^https?:\/\//i.test(value);
  }

  private getTokenKey(envId: string) {
    return `${SECRET_TOKEN_PREFIX}.${envId}`;
  }

  private async buildLocalServiceEnv(repoPath: string): Promise<NodeJS.ProcessEnv> {
    const overrides = await this.getLocalServiceEnvOverrides(repoPath);
    return {
      ...process.env,
      LOCAL_DEV: '1',
      ...overrides
    };
  }

  private async getLocalServiceEnvOverrides(repoPath: string): Promise<Record<string, string>> {
    return this.getRootfsEnvOverrides(repoPath);
  }

  private getServerPort(env?: NodeJS.ProcessEnv): number {
    const rawValue = env?.PORT ?? process.env.PORT ?? '8080';
    const candidate = String(rawValue).trim().replace(/^.*:/, '');
    const parsed = Number.parseInt(candidate || '8080', 10);
    if (Number.isFinite(parsed) && parsed > 0 && parsed <= 65535) {
      return parsed;
    }
    if (candidate) {
      this.outputChannel.appendLine(
        `[VM Manager] Invalid PORT value "${rawValue}". Falling back to 8080.`
      );
    }
    return 8080;
  }

  private async ensureServerPortAvailable(port: number): Promise<boolean> {
    if (!Number.isFinite(port)) {
      return true;
    }

    if (await this.isExistingVmManagerRunning(port)) {
      const message = `TensorFleet VM Manager is already running on port ${port}. Stop it before starting another instance.`;
      this.outputChannel.appendLine(`[VM Manager] ${message}`);
      void vscode.window.showInformationMessage(message);
      return false;
    }

    if (!(await this.isPortAvailable(port))) {
      const message = `Port ${port} is already in use. Stop the process using it before starting TensorFleet VM Manager.`;
      this.outputChannel.appendLine(`[VM Manager] ${message}`);
      void vscode.window.showErrorMessage(message);
      return false;
    }

    return true;
  }

  private async isExistingVmManagerRunning(port: number): Promise<boolean> {
    return new Promise((resolve) => {
      const request = http.request(
        {
          host: '127.0.0.1',
          port,
          path: '/health',
          method: 'GET',
          timeout: 1000
        },
        (response) => {
          response.resume();
          const healthy =
            (response.statusCode ?? 0) >= 200 && (response.statusCode ?? 0) < 300;
          resolve(healthy);
        }
      );

      request.once('error', () => resolve(false));
      request.once('timeout', () => {
        request.destroy();
        resolve(false);
      });

      request.end();
    });
  }

  private getRootfsEnvOverrides(repoPath: string): Record<string, string> {
    const envVars: Record<string, string> = {};
    const rootfsDir = this.getDefaultRootfsDir();
    try {
      fs.mkdirSync(rootfsDir, { recursive: true });
      envVars.FIRECRACKER_ROOTFS_DIR = rootfsDir;
    } catch (error) {
      this.outputChannel.appendLine(
        `[VM Manager] Failed to prepare local rootfs directory (${rootfsDir}): ${this.formatError(error)}`
      );
    }

    if (!process.env.FIRECRACKER_BASE_ROOTFS) {
      const detectedBase = this.findLocalBaseRootfs(repoPath);
      if (detectedBase) {
        envVars.FIRECRACKER_BASE_ROOTFS = detectedBase;
        this.baseRootfsWarningShown = false;
      } else if (!this.baseRootfsWarningShown) {
        this.baseRootfsWarningShown = true;
        this.outputChannel.appendLine(
          '[VM Manager] Could not find a Firecracker base rootfs under ../firecracker-vm/build. ' +
            'Set FIRECRACKER_BASE_ROOTFS if VM creation needs a custom image.'
        );
      }
    }

    return envVars;
  }

  private async isPortAvailable(port: number): Promise<boolean> {
    return new Promise((resolve) => {
      const tester = net.createServer();
      tester.unref?.();

      tester.once('error', (error: NodeJS.ErrnoException) => {
        if (error.code !== 'EADDRINUSE' && error.code !== 'EACCES') {
          this.outputChannel.appendLine(
            `[VM Manager] Port availability check for ${port} failed: ${error.message}`
          );
        }
        resolve(false);
      });

      tester.listen(port, '0.0.0.0', () => {
        tester.close(() => resolve(true));
      });
    });
  }

  private getDefaultRootfsDir(): string {
    const home = os.homedir();
    if (!home) {
      return path.resolve(this.context.globalStorageUri.fsPath, 'rootfs');
    }
    return path.join(home, '.tensorfleet', 'rootfs');
  }

  private findLocalBaseRootfs(repoPath: string): string | undefined {
    const candidates: string[] = [];
    const repoSibling = path.resolve(repoPath, '..', 'firecracker-vm', 'build');
    candidates.push(repoSibling);

    const extensionSibling = path.resolve(this.context.extensionPath, '..', 'firecracker-vm', 'build');
    if (extensionSibling !== repoSibling) {
      candidates.push(extensionSibling);
    }

    for (const dir of candidates) {
      if (!fs.existsSync(dir)) {
        continue;
      }
      let stats: fs.Stats | undefined;
      try {
        stats = fs.statSync(dir);
      } catch {
        continue;
      }
      if (!stats.isDirectory()) {
        continue;
      }
      const preferred = path.join(dir, 'gazebo-rootfs.ext4');
      if (fs.existsSync(preferred)) {
        return preferred;
      }
      const entries = fs.readdirSync(dir);
      const fallback = entries.find((entry) => entry.endsWith('.ext4'));
      if (fallback) {
        return path.join(dir, fallback);
      }
    }

    return undefined;
  }

  private async getAuthToken(envId?: string): Promise<string | undefined> {
    const targetEnvId = envId ?? this.getActiveEnvironment()?.id;
    if (!targetEnvId) {
      return undefined;
    }
    return this.context.secrets.get(this.getTokenKey(targetEnvId));
  }

  private async setAuthToken(envId: string, token: string | null) {
    const key = this.getTokenKey(envId);
    if (!token) {
      await this.context.secrets.delete(key);
      return;
    }
    await this.context.secrets.store(key, token);
  }

  private getLoginMetadataMap(): Record<string, LoginMetadataEntry> {
    return this.context.globalState.get<Record<string, LoginMetadataEntry>>(LOGIN_METADATA_KEY, {});
  }

  private getLoginMetadataForEnv(envId: string): LoginMetadataEntry | undefined {
    const metadata = this.getLoginMetadataMap();
    return metadata[envId];
  }

  private async updateLoginMetadata(envId: string, entry?: LoginMetadataEntry | null) {
    const metadata = this.getLoginMetadataMap();
    if (!entry) {
      delete metadata[envId];
    } else {
      metadata[envId] = entry;
    }
    await this.context.globalState.update(LOGIN_METADATA_KEY, metadata);
  }

  private async handleUnauthorized(envId: string) {
    await this.setAuthToken(envId, null);
    await this.updateLoginMetadata(envId, null);
    if (!this.authWarningIssued.has(envId)) {
      this.authWarningIssued.add(envId);
      this.outputChannel.appendLine('[VM Manager] API returned 401. Cleared cached token.');
      void vscode.window.showWarningMessage('TensorFleet VM Manager authentication expired. Please log in again.');
    }
    await this.broadcastState();
  }

  private ensureGoAvailable(goCommand: string): boolean {
    try {
      const result = spawnSync(goCommand, ['version'], { stdio: 'ignore' });
      if (result.error) {
        throw result.error;
      }
      if (result.status !== 0) {
        throw new Error(`Go command exited with status ${result.status}`);
      }
      return true;
    } catch (error) {
      const message = this.formatError(error) || 'Go CLI not found';
      vscode.window.showErrorMessage(
        `Unable to start VM Manager because "${goCommand}" is unavailable. Install Go and ensure it is on PATH. (${message})`
      );
      this.outputChannel.appendLine(`[VM Manager] Go verification failed: ${message}`);
      return false;
    }
  }

  private async terminateProcessTree(child: ChildProcess): Promise<void> {
    const pid = child.pid;
    if (!pid) {
      child.kill();
      return;
    }

    const exitPromise = new Promise<void>((resolve) => {
      const cleanup = () => resolve();
      child.once('exit', cleanup);
      child.once('close', cleanup);
      child.once('error', cleanup);
    });

    try {
      if (process.platform === 'win32') {
        await this.killWindowsProcessTree(pid);
      } else {
        await this.killPosixProcessTree(pid);
      }
    } catch (error) {
      this.outputChannel.appendLine(`[VM Manager] Failed to terminate child processes gracefully: ${this.formatError(error)}`);
      try {
        child.kill('SIGKILL');
      } catch {
        // Ignore follow-up failures
      }
    }

    const completed = await Promise.race([
      exitPromise.then(() => true),
      new Promise<boolean>((resolve) => setTimeout(() => resolve(false), 5000))
    ]);

    if (!completed) {
      this.outputChannel.appendLine('[VM Manager] Force killing vm-manager process after timeout.');
      if (process.platform === 'win32') {
        await this.killWindowsProcessTree(pid);
      } else {
        try {
          process.kill(pid, 'SIGKILL');
        } catch (error) {
          this.outputChannel.appendLine(`[VM Manager] Force kill failed: ${this.formatError(error)}`);
        }
      }
      await exitPromise;
    }
  }

  private async killWindowsProcessTree(pid: number): Promise<void> {
    await new Promise<void>((resolve) => {
      const killer = spawn('taskkill', ['/PID', pid.toString(), '/T', '/F']);
      let stderr = '';
      killer.stderr?.on('data', (chunk) => {
        stderr += chunk.toString();
      });
      killer.once('close', (code) => {
        if (code !== 0 && stderr.trim()) {
          this.outputChannel.appendLine(`[VM Manager] taskkill: ${stderr.trim()}`);
        }
        resolve();
      });
      killer.once('error', (error) => {
        this.outputChannel.appendLine(`[VM Manager] taskkill failed: ${this.formatError(error)}`);
        resolve();
      });
    });
  }

  private async killPosixProcessTree(rootPid: number): Promise<void> {
    const descendants = await this.collectDescendantPids(rootPid).catch((error) => {
      this.outputChannel.appendLine(`[VM Manager] Failed to enumerate descendant processes: ${this.formatError(error)}`);
      return [];
    });

    for (const pid of descendants.reverse()) {
      try {
        process.kill(pid, 'SIGTERM');
      } catch (error) {
        if ((error as NodeJS.ErrnoException).code !== 'ESRCH') {
          throw error;
        }
      }
    }

    try {
      process.kill(rootPid, 'SIGTERM');
    } catch (error) {
      if ((error as NodeJS.ErrnoException).code !== 'ESRCH') {
        throw error;
      }
    }
  }

  private async collectDescendantPids(rootPid: number): Promise<number[]> {
    return new Promise<number[]>((resolve, reject) => {
      const ps = spawn('ps', ['-eo', 'pid=,ppid=']);
      const chunks: Buffer[] = [];
      const errorChunks: Buffer[] = [];

      ps.stdout?.on('data', (chunk) => chunks.push(chunk));
      ps.stderr?.on('data', (chunk) => errorChunks.push(chunk));
      ps.once('error', reject);
      ps.once('close', (code) => {
        if (code !== 0) {
          const errorText = errorChunks.length ? Buffer.concat(errorChunks).toString('utf8').trim() : '';
          reject(new Error(errorText || `ps exited with code ${code}`));
          return;
        }

        const map = new Map<number, number[]>();
        Buffer.concat(chunks)
          .toString('utf8')
          .split('\n')
          .forEach((line) => {
            const trimmed = line.trim();
            if (!trimmed) {
              return;
            }
            const parts = trimmed.split(/\s+/);
            if (parts.length < 2) {
              return;
            }
            const pid = Number(parts[0]);
            const ppid = Number(parts[1]);
            if (Number.isNaN(pid) || Number.isNaN(ppid)) {
              return;
            }
            if (!map.has(ppid)) {
              map.set(ppid, []);
            }
            map.get(ppid)!.push(pid);
          });

        const result: number[] = [];
        const queue: number[] = [];
        const visited = new Set<number>();
        const firstLevel = map.get(rootPid) ?? [];
        queue.push(...firstLevel);

        while (queue.length > 0) {
          const current = queue.shift()!;
          if (visited.has(current)) {
            continue;
          }
          visited.add(current);
          result.push(current);
          const children = map.get(current);
          if (children && children.length) {
            queue.push(...children);
          }
        }

        resolve(result);
      });
    });
  }

  private createNetworkError(error: unknown, url: URL): Error {
    if (error && typeof error === 'object' && 'code' in error) {
      const code = String((error as NodeJS.ErrnoException).code);
      if (code === 'ECONNREFUSED') {
        return new Error(`VM Manager API is not responding at ${url.origin}. Start the Go service or update tensorfleet.vmManager.apiBaseUrl.`);
      }
      if (code === 'ETIMEDOUT') {
        return new Error(`VM Manager request to ${url.origin} timed out. Check the service status.`);
      }
    }

    if (error instanceof Error) {
      return error;
    }
    return new Error(this.formatError(error));
  }

  private formatError(error: unknown): string {
    if (error instanceof Error) {
      return error.message;
    }
    if (typeof error === 'string') {
      return error;
    }
    try {
      return JSON.stringify(error);
    } catch {
      return 'Unknown error';
    }
  }
}
