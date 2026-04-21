import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';
import { spawn, ChildProcess } from 'child_process';
import { MCPBridge } from './mcp-bridge';
import { VMManagerIntegration, VMConfig } from './vm-manager';
import * as auth from './auth';
import * as help from './help';
import { UnifiedStatusCoordinator } from './unified-status';
import { TelemetryService } from './telemetry';
import * as regions from './regions';
import { initializeEnv, isDev, env, registerDevCommand, getMode } from './env';

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

type PanelKind = 'standard' | 'terminalTabs';

function isRuntimeWorldSwitchAllowed(configId: string | undefined): boolean {
  return configId !== 'lerobot';
}

// Menu action types for deterministic option handling
type MenuAction =
  | 'auth.login'
  | 'auth.logout'
  | 'vm.retryStatus'
  | 'vm.start'
  | 'vm.stop'
  | 'vm.retryStart'
  | 'vm.refresh'
  | 'vm.chooseConfig'
  | 'region.change'
  | 'onboarding.reset'
  | 'noop.header';

type ActionItem = vscode.QuickPickItem & { action: MenuAction; actionData?: any };

function item(label: string, action: MenuAction, detail?: string, kind?: vscode.QuickPickItemKind, actionData?: any): ActionItem {
  return { label, detail, kind, action, actionData };
}

type DroneViewport = {
  id: string;
  title: string;
  description: string;
  image: string;
  command: string;
  actionLabel: string;
  panelKind?: PanelKind;
  htmlTemplate?: string;
};

type TerminalConfig = {
  id: string;
  name: string;
  startupCommands?: string[];
};

/**
 * New type: panels with unique functionality.
 * They do NOT reuse DroneViewport, do NOT have a generic "open" command,
 * and MUST render via their own function.
 */
type PanelHtmlRenderer = (args: {
  webview: vscode.Webview;
  context: vscode.ExtensionContext;
  panelDef: UniquePanel;
}) => string | Promise<string>;

type UniquePanel = {
  /** View id to register as WebviewViewProvider */
  id: string;
  /** Title (optional; useful for templates) */
  title?: string;
  /**
   * Function that returns the webview HTML (dynamic).
   * You can share this function across items or make one per item.
   */
  render: PanelHtmlRenderer;
  /**
   * Optional message handler override. If omitted, default handler runs.
   */
  onMessage?: (message: any, api: {
    context: vscode.ExtensionContext;
    webview: vscode.Webview;
    telemetry?: TelemetryService | null;
  }) => void | Promise<void>;
  /**
   * Optional extra local resource roots
   */
  localResourceRoots?: (ctx: { context: vscode.ExtensionContext }) => vscode.Uri[];
};

type TensorfleetMetadata = {
  template?: string;
  version?: number;
  managedEnv?: boolean;
  env?: {
    region?: string;
    baseUrl?: string;
    vmManagerUrl?: string;
    proxyUrl?: string;
    rosbridgeUrl?: string;
    rosbridgePort?: number;
    nodeId?: string;
    r2bHost?: string;
  };
};

// -----------------------------------------------------------------------------
// Collections
// -----------------------------------------------------------------------------

const DRONE_VIEWS: DroneViewport[] = [
  {
    id: 'tensorfleet-gzweb-panel',
    title: 'Simulation view',
    description: 'Render Gazebo scenes via gzweb with direct WS or VM manager login shims.',
    image: 'gazebo-placeholder.svg',
    command: 'tensorfleet.openGzWebPanel',
    actionLabel: 'Open Simulation View',
    panelKind: 'standard',
    htmlTemplate: 'gzweb-standalone'
  },
  {
    id: 'tensorfleet-aiops',
    title: 'AI Model Ops',
    description: 'Run TensorFleet AI models on live or recorded video feeds, and inspect inference metrics.',
    image: 'ai-models-placeholder.svg',
    command: 'tensorfleet.openAIPanel',
    actionLabel: 'Run AI Analysis'
  },
  {
    id: 'tensorfleet-ros2-baselines',
    title: 'ROS 2 & Stable Baselines',
    description: 'Switch between ROS 2 middleware terminals and Stable Baselines reinforcement learning workflows.',
    image: 'ros2-baselines-placeholder.svg',
    command: 'tensorfleet.openROS2Panel',
    actionLabel: 'Launch Robotics Lab',
    panelKind: 'terminalTabs'
  },
  {
    id: 'tensorfleet-teleops-panel',
    title: 'Teleops Panel',
    description: 'Control drone with keyboard - custom React implementation for precise control.',
    image: 'tensorfleet-icon.svg',
    command: 'tensorfleet.openTeleopsPanel',
    actionLabel: 'Open Teleops Panel',
    panelKind: 'standard',
    htmlTemplate: 'teleops-standalone'
  },
  {
    id: 'tensorfleet-image-panel',
    title: 'Image Panel',
    description: 'Display camera feeds with custom React components - lightweight, deeply integrated.',
    image: 'tensorfleet-icon.svg',
    command: 'tensorfleet.openImagePanel',
    actionLabel: 'Open Image Panel',
    panelKind: 'standard',
    htmlTemplate: 'image-standalone'
  },
  {
    id: "tensorfleet-map-panel",
    title: 'Map view',
    description: 'Display world map view with msision control elements.',
    image: 'connected_vehicle.tasoHGVc.jpg',
    command: 'tensorfleet.openMapPanel',
    actionLabel: 'Open Map Panel',
    panelKind: 'standard',
    htmlTemplate: 'map-standalone'
  },
  {
    id: "tensorfleet-sensor-3d-panel",
    title: "3D sensor view",
    description: "3D view of the collected data from drone sensors",
    image: 'tensorfleet-icon.svg',
    command: 'tensorfleet.openSensor3DPanel',
    actionLabel: 'Open 3D Sensor view',
    panelKind: 'standard',
    htmlTemplate: 'sensor-3d-standalone'
  },
  {
    id: "tensorfleet-raw-messages-panel",
    title: 'Raw Messages',
    description: 'Display raw ROS2 messages in real-time - monitor and debug message traffic.',
    image: 'tensorfleet-icon.svg',
    command: 'tensorfleet.openRawMessagesPanel',
    actionLabel: 'Open Raw Messages Panel',
    panelKind: 'standard',
    htmlTemplate: 'raw-messages-standalone'
  },
  {
    id: 'tensorfleet-featured-entities-panel',
    title: 'Featured entities',
    description: 'Display and manage important simulation entities with detailed information and controls.',
    image: 'tensorfleet-icon.svg',
    command: 'tensorfleet.openFeaturedEntitiesPanel',
    actionLabel: 'Open Featured Entities Panel',
    panelKind: 'standard',
    htmlTemplate: 'featured-entities-standalone'
  }
];

/**
 * Unique panels that will have their own html rendering functions.
 */
/**
 * Message handler for server settings panel
 */
async function serverSettingsMessageHandler(message: any, api: {
  context: vscode.ExtensionContext;
  webview: vscode.Webview;
  telemetry?: TelemetryService | null;
}) {
  if (!message || !message.command) return;

  const { command } = message;
  const { webview, telemetry } = api;

  try {
    switch (command) {
      case 'getRegions': {
        const regionsList = regions.getAvailableRegions();
        const currentRegion = regions.getSelectedRegion();
        const regionData = Object.values(regionsList).map(region => ({
          id: region.id,
          name: region.name,
          icon: region.icon,
          description: region.description
        }));

        webview.postMessage({
          command: 'updateRegions',
          regions: regionData,
          currentRegion: {
            id: currentRegion.id,
            name: currentRegion.name,
            icon: currentRegion.icon,
            description: currentRegion.description
          }
        });
        break;
      }

      case 'getVmTypes': {
        if (!vmManagerIntegration) {
          webview.postMessage({
            command: 'updateVmTypes',
            vmTypes: [],
            currentVmType: { id: 'unknown', name: 'Unknown', description: 'VM Manager not available' }
          });
          return;
        }

        const vmConfigs = Object.values(VMManagerIntegration.VM_CONFIGS);
        const currentConfig = vmManagerIntegration.getLastUsedConfig();

        webview.postMessage({
          command: 'updateVmTypes',
          vmTypes: vmConfigs.map(config => ({
            id: config.id,
            name: config.name,
            description: config.description
          })),
          currentVmType: {
            id: currentConfig.id,
            name: currentConfig.name,
            description: currentConfig.description
          }
        });
        break;
      }

      case 'getStatus': {
        if (!unifiedStatusCoordinator) {
          webview.postMessage({
            command: 'updateStatus',
            status: {
              auth: 'not_authenticated',
              connection: 'disconnected',
              vmState: 'unknown'
            }
          });
          return;
        }

        const state = unifiedStatusCoordinator.getState();
        webview.postMessage({
          command: 'updateStatus',
          status: {
            auth: state.auth,
            connection: state.connection,
            vmState: state.vmState,
            ipAddress: state.ipAddress,
            provider: state.provider,
            region: state.region,
            uptimeSeconds: state.uptimeSeconds,
            error: state.error
          }
        });
        break;
      }

      case 'getSimulationWorldData': {
        if (!vmManagerIntegration) {
          webview.postMessage({
            command: 'updateSimulationWorldData',
            presets: [],
            selection: { mode: 'default' },
            available: false,
            reason: 'VM Manager not available'
          });
          return;
        }

        const currentConfig = vmManagerIntegration.getLastUsedConfig();
        if (!isRuntimeWorldSwitchAllowed(currentConfig?.id)) {
          webview.postMessage({
            command: 'updateSimulationWorldData',
            presets: [],
            selection: { mode: 'default' },
            available: false,
            reason: 'Simulation world switching is disabled for Lerobot arm VMs'
          });
          return;
        }

        const state = unifiedStatusCoordinator?.getState();
        const vmRunning = state?.vmState === 'running';
        if (!vmRunning) {
          webview.postMessage({
            command: 'updateSimulationWorldData',
            presets: [],
            selection: { mode: 'default' },
            available: false,
            reason: 'Start the VM to manage runtime simulation worlds'
          });
          return;
        }

        try {
          const [presets, selection] = await Promise.all([
            vmManagerIntegration.listGazeboPresets(),
            vmManagerIntegration.getGazeboSelection()
          ]);

          webview.postMessage({
            command: 'updateSimulationWorldData',
            presets,
            selection,
            available: true
          });
        } catch (error) {
          const message = error instanceof Error ? error.message : String(error);
          webview.postMessage({
            command: 'updateSimulationWorldData',
            presets: [],
            selection: { mode: 'default' },
            available: false,
            reason: `Runtime world switching unavailable: ${message}`
          });
        }
        break;
      }

      case 'setRegion': {
        const { regionId } = message;
        if (!regionId) return;

        telemetry?.trackEvent('serverSettings.region.set', { regionId, phase: 'start' });
        await regions.setSelectedRegion(regionId);

        telemetry?.trackEvent('serverSettings.region.set', { regionId, phase: 'success' });

        // Refresh VM Manager status
        if (vmManagerIntegration) {
          vmManagerIntegration.refreshStatus(false);
        }

        // Send updated region data
        await serverSettingsMessageHandler({ command: 'getRegions' }, api);
        break;
      }

      case 'setVmType': {
        const { vmTypeId } = message;
        if (!vmTypeId || !vmManagerIntegration) return;

        telemetry?.trackEvent('serverSettings.vmType.set', { vmTypeId, phase: 'start' });
        vmManagerIntegration.setLastUsedConfig(vmTypeId);
        telemetry?.trackEvent('serverSettings.vmType.set', { vmTypeId, phase: 'success' });

        // Send updated VM types data
        await serverSettingsMessageHandler({ command: 'getVmTypes' }, api);
        await serverSettingsMessageHandler({ command: 'getSimulationWorldData' }, api);
        break;
      }

      case 'startVm': {
        if (!vmManagerIntegration) return;

        telemetry?.trackEvent('serverSettings.vm.start', { phase: 'start' });
        try {
          await vmManagerIntegration.startVm();
          telemetry?.trackEvent('serverSettings.vm.start', { phase: 'success' });

          // Send updated status back to webview
          if (unifiedStatusCoordinator) {
            const state = unifiedStatusCoordinator.getState();
            webview.postMessage({
              command: 'updateStatus',
              status: {
                auth: state.auth,
                connection: state.connection,
                vmState: state.vmState,
                ipAddress: state.ipAddress,
                provider: state.provider,
                region: state.region,
                uptimeSeconds: state.uptimeSeconds,
                error: state.error
              }
            });
          }
        } catch (error) {
          telemetry?.captureError(error, { source: 'serverSettings.vm.start' });
          telemetry?.trackEvent('serverSettings.vm.start', { phase: 'error' });
          throw error;
        }
        break;
      }

      case 'stopVm': {
        if (!vmManagerIntegration) return;

        telemetry?.trackEvent('serverSettings.vm.stop', { phase: 'start' });
        try {
          await vmManagerIntegration.stopVm();
          telemetry?.trackEvent('serverSettings.vm.stop', { phase: 'success' });

          // Send updated status back to webview
          if (unifiedStatusCoordinator) {
            const state = unifiedStatusCoordinator.getState();
            webview.postMessage({
              command: 'updateStatus',
              status: {
                auth: state.auth,
                connection: state.connection,
                vmState: state.vmState,
                ipAddress: state.ipAddress,
                provider: state.provider,
                region: state.region,
                uptimeSeconds: state.uptimeSeconds,
                error: state.error
              }
            });
          }
        } catch (error) {
          telemetry?.captureError(error, { source: 'serverSettings.vm.stop' });
          telemetry?.trackEvent('serverSettings.vm.stop', { phase: 'error' });
          throw error;
        }
        break;
      }

      case 'setSimulationWorldPreset': {
        const { preset } = message;
        if (!preset || !vmManagerIntegration) return;
        const currentConfig = vmManagerIntegration.getLastUsedConfig();
        if (!isRuntimeWorldSwitchAllowed(currentConfig?.id)) {
          void vscode.window.showInformationMessage('Simulation world switching is disabled for Lerobot arm VMs');
          await serverSettingsMessageHandler({ command: 'getSimulationWorldData' }, api);
          break;
        }

        telemetry?.trackEvent('serverSettings.simulationWorld.setPreset', { preset, phase: 'start' });
        const resultMessage = await vmManagerIntegration.setGazeboPreset(preset);
        await refreshSimulationViewPanels(api.context);
        telemetry?.trackEvent('serverSettings.simulationWorld.setPreset', { preset, phase: 'success' });
        void vscode.window.showInformationMessage(resultMessage);
        await serverSettingsMessageHandler({ command: 'getSimulationWorldData' }, api);
        break;
      }

      case 'resetSimulationWorld': {
        if (!vmManagerIntegration) return;
        const currentConfig = vmManagerIntegration.getLastUsedConfig();
        if (!isRuntimeWorldSwitchAllowed(currentConfig?.id)) {
          void vscode.window.showInformationMessage('Simulation world switching is disabled for Lerobot arm VMs');
          await serverSettingsMessageHandler({ command: 'getSimulationWorldData' }, api);
          break;
        }

        telemetry?.trackEvent('serverSettings.simulationWorld.reset', { phase: 'start' });
        const resultMessage = await vmManagerIntegration.resetGazeboSelection();
        await refreshSimulationViewPanels(api.context);
        telemetry?.trackEvent('serverSettings.simulationWorld.reset', { phase: 'success' });
        void vscode.window.showInformationMessage(resultMessage);
        await serverSettingsMessageHandler({ command: 'getSimulationWorldData' }, api);
        break;
      }

      default: {
        console.log('[ServerSettings] Unknown message:', command);
        break;
      }
    }
  } catch (error) {
    telemetry?.captureError(error, { source: 'serverSettingsMessageHandler', command });
    console.error('[ServerSettings] Message handler error:', error);
    if (!['getStatus', 'getRegions', 'getVmTypes', 'getSimulationWorldData'].includes(command)) {
      const message = error instanceof Error ? error.message : String(error);
      void vscode.window.showErrorMessage(`Server settings action failed: ${message}`);
    }
  }
}

const UNIQUE_PANELS: UniquePanel[] = [
  {
    id: 'tensorfleet-account',
    title: 'TensorFleet Account',
    render: async ({ webview, context, panelDef }) => {
      const cspSource = webview.cspSource;
      const styles = getBaseStyles();
      const avatarImg = webview
        .asWebviewUri(vscode.Uri.joinPath(context.extensionUri, 'media', 'avatar_solid.svg'))
        .toString();
      const cspMeta = `<meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src ${cspSource} 'unsafe-inline'; script-src ${cspSource} 'unsafe-inline' 'unsafe-eval'; img-src ${cspSource} data: https:; font-src ${cspSource} data:; connect-src ${cspSource} https: http: ws: wss:;">`;

      // Check if user is authenticated
      const isAuth = await auth.isAuthenticated(context);

      // Get real user profile from JWT token
      const userProfile = await auth.getUserProfile(context);
      const userName = userProfile?.name || 'TensorFleet User';
      const userEmail = userProfile?.email || '';
      // Use user's profile picture if available, otherwise fall back to default avatar
      const userAvatar = userProfile?.picture || avatarImg;

      if (!isAuth) {
        // Not authenticated - show login button
        return `
<!doctype html>
<html>
<head>
  <meta charset="UTF-8" />
  ${cspMeta}
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>${panelDef.title ?? 'Tensorfleet Login'}</title>
  ${styles}
  <style>
    .account-panel {
      display: flex;
      flex-direction: column;
      align-items: center;
      text-align: center;
      padding: 20px 16px;
      gap: 16px;
    }
    .account-avatar {
      width: 64px;
      height: 64px;
      border-radius: 50%;
      background: var(--vscode-button-background);
      display: flex;
      align-items: center;
      justify-content: center;
      overflow: hidden;
    }
    .account-avatar img {
      width: 100%;
      height: 100%;
    }
    .account-info {
      display: flex;
      flex-direction: column;
      gap: 4px;
    }
    .account-name {
      font-size: 1.1rem;
      font-weight: 600;
      margin: 0;
    }
    .account-email {
      font-size: 0.9rem;
      opacity: 0.7;
      margin: 0;
    }
    .account-actions {
      display: flex;
      flex-direction: column;
      gap: 8px;
      width: 100%;
      max-width: 200px;
    }
    .btn {
      padding: 8px 16px;
      border-radius: 4px;
      border: none;
      cursor: pointer;
      font-size: 0.95rem;
      width: 100%;
    }
    .btn-primary {
      background: var(--vscode-button-background);
      color: var(--vscode-button-foreground);
    }
    .btn-primary:hover {
      background: var(--vscode-button-hoverBackground);
    }
    .btn-secondary {
      background: transparent;
      color: var(--vscode-foreground);
      border: 1px solid var(--vscode-button-border, var(--vscode-button-background));
    }
    .btn-secondary:hover {
      background: var(--vscode-toolbar-hoverBackground, rgba(255, 255, 255, 0.08));
    }
    .login-message {
      opacity: 0.8;
      font-size: 0.9rem;
    }
  </style>
</head>
<body>
  <div class="account-panel">
    <div class="account-avatar">
      <img src="${avatarImg}" alt="Account" />
    </div>
    <p class="login-message">Sign in to access your TensorFleet account</p>
    <div class="account-actions">
      <button class="btn btn-primary" id="tf-login">Login</button>
    </div>
  </div>
  <script>
    const vscode = acquireVsCodeApi();
    document.getElementById('tf-login')?.addEventListener('click', () => {
      vscode.postMessage({ command: 'login' });
    });
  </script>
</body>
</html>
        `;
      }

      // Authenticated - show profile with logout
      return `
<!doctype html>
<html>
<head>
  <meta charset="UTF-8" />
  ${cspMeta}
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>${panelDef.title ?? 'TensorFleet Account'}</title>
  ${styles}
  <style>
    .account-panel {
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 20px 16px;
      gap: 28px;
    }
    .account-header {
      display: flex;
      align-items: center;
      gap: 12px;
    }
    .account-avatar {
      width: 56px;
      height: 56px;
      border-radius: 50%;
      background: #ffffff;
      display: flex;
      align-items: center;
      justify-content: center;
      overflow: hidden;
      flex-shrink: 0;
    }
    .account-avatar img {
      width: 100%;
      height: 100%;
    }
    .account-info {
      display: flex;
      flex-direction: column;
      gap: 2px;
    }
    .account-name {
      font-size: 1.25rem;
      font-weight: 600;
      margin: 0;
      color: var(--vscode-foreground);
    }
    .account-email {
      font-size: 0.85rem;
      opacity: 0.7;
      margin: 0;
      color: #4fc3f7;
    }
    .account-actions {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 8px;
      width: 100%;
    }
    .btn {
      padding: 10px 16px;
      border-radius: 4px;
      border: none;
      cursor: pointer;
      font-size: 0.9rem;
      text-align: center;
      width: 100%;
      max-width: 220px;
    }
    .btn-dashboard {
      background: #315bab;
      color: #ffffff;
    }
    .btn-dashboard:hover {
      background: #3d6fc4;
    }
    .btn-secondary {
      background: rgba(255, 255, 255, 0.08);
      color: var(--vscode-foreground);
      border: 1px solid rgba(255, 255, 255, 0.2);
    }
    .btn-secondary:hover {
      background: rgba(255, 255, 255, 0.12);
    }
    .back-link {
      display: inline-flex;
      align-items: center;
      gap: 4px;
      color: var(--vscode-foreground);
      opacity: 0.8;
      font-size: 0.9rem;
      cursor: pointer;
      background: none;
      border: none;
      padding: 4px 0;
      margin-top: 12px;
    }
    .back-link:hover {
      opacity: 1;
    }
  </style>
</head>
<body>
  <div class="account-panel">
    <div class="account-header">
      <div class="account-avatar">
        <img src="${userAvatar}" alt="Account" />
      </div>
      <div class="account-info">
        <p class="account-name">${userName}</p>
        <p class="account-email">${userEmail}</p>
      </div>
    </div>
    <div class="account-actions">
      <button class="btn btn-dashboard" id="tf-dashboard">Open Dashboard</button>
      <button class="btn btn-secondary" id="tf-logout">Logout</button>
    </div>
    <button class="back-link" id="tf-back">← Back</button>
  </div>
  <script>
    const vscode = acquireVsCodeApi();
    document.getElementById('tf-dashboard')?.addEventListener('click', () => {
      vscode.postMessage({ command: 'openDashboard' });
    });
    document.getElementById('tf-logout')?.addEventListener('click', () => {
      vscode.postMessage({ command: 'logout' });
    });
    document.getElementById('tf-back')?.addEventListener('click', () => {
      vscode.postMessage({ command: 'back' });
    });
  </script>
</body>
</html>

      `;
    },
    onMessage: async (message, api) => {
      if (!message || !message.command) return;
      if (message.command === 'login') {
        await vscode.commands.executeCommand('tensorfleet.login');
      } else if (message.command === 'logout') {
        await vscode.commands.executeCommand('tensorfleet.logout');
      } else if (message.command === 'openDashboard') {
        vscode.env.openExternal(vscode.Uri.parse('https://app.tensorfleet.net/'));
      } else if (message.command === 'back') {
        await vscode.commands.executeCommand('tensorfleet.closeAccountPanel');
      }
    },
    localResourceRoots: ({ context }) => [
      vscode.Uri.joinPath(context.extensionUri, 'media')
    ]
  },
  {
    id: 'tensorfleet-view-3d',
    title: '3D View',
    render: htmlRenderer('visualization-dashboard.html')
  },
  {
    id: 'tensorfleet-drone-view-list',
    title: 'Drone and ROS views',
    render: htmlRenderer('drone-view-list.html')
  },
  {
    id: 'tensorfleet-server-settings',
    title: 'Virtual machine settings',
    render: htmlRenderer('server-settings.html'),
    onMessage: serverSettingsMessageHandler
  },
  {
    id: 'tensorfleet-help-panel',
    title: 'Help panel',
    render: htmlRenderer('help-panel.html')
  }
];

// -----------------------------------------------------------------------------
// Globals / services
// -----------------------------------------------------------------------------

const MANAGED_ENV_KEYS = [
  'TENSORFLEET_BASE_URL',
  'TENSORFLEET_JWT'
] as const;

const TERMINAL_CONFIGS: Record<string, TerminalConfig> = {
  ros2: {
    id: 'tensorfleet-ros2-terminal',
    name: 'TensorFleet ROS 2',
    startupCommands: ['# Placeholder: source your ROS 2 environment here']
  },
  baselines: {
    id: 'tensorfleet-baselines-terminal',
    name: 'TensorFleet Stable Baselines',
    startupCommands: ['# Placeholder: activate stable baselines venv here']
  }
};

const terminalRegistry = new Map<string, vscode.Terminal>();
let mcpServerProcess: ChildProcess | null = null;
let mcpBridge: MCPBridge | null = null;
let vmManagerIntegration: VMManagerIntegration | null = null;
let telemetryService: TelemetryService | null = null;
let envRefreshTimer: NodeJS.Timeout | null = null;



// Status bar items for TensorFleet projects
let rosVersionStatusBar: vscode.StatusBarItem | null = null;
let droneStatusBar: vscode.StatusBarItem | null = null;
let projectWatcher: vscode.FileSystemWatcher | null = null;

// Unified status coordinator
let unifiedStatusCoordinator: UnifiedStatusCoordinator | null = null;

// Panel registry for unique panels (to enable refresh on auth changes)
const uniquePanelRegistry = new Map<string, UniqueViewProvider>();
const dedicatedPanelRegistry = new Map<string, Set<vscode.WebviewPanel>>();

function registerDedicatedPanel(viewId: string, panel: vscode.WebviewPanel) {
  let panels = dedicatedPanelRegistry.get(viewId);
  if (!panels) {
    panels = new Set<vscode.WebviewPanel>();
    dedicatedPanelRegistry.set(viewId, panels);
  }

  panels.add(panel);
  panel.onDidDispose(() => {
    const existing = dedicatedPanelRegistry.get(viewId);
    if (!existing) {
      return;
    }
    existing.delete(panel);
    if (existing.size === 0) {
      dedicatedPanelRegistry.delete(viewId);
    }
  });
}

async function refreshDedicatedPanels(view: DroneViewport, context: vscode.ExtensionContext): Promise<void> {
  const panels = dedicatedPanelRegistry.get(view.id);
  if (!panels || panels.size === 0) {
    return;
  }

  for (const panel of panels) {
    const webview = panel.webview;
    const imageUri = webview.asWebviewUri(vscode.Uri.joinPath(context.extensionUri, 'media', view.image)).toString();
    const cspSource = webview.cspSource;

    if (view.panelKind === 'terminalTabs') {
      webview.html = getTerminalPanelHtml(view, imageUri, cspSource);
      continue;
    }

    if (view.htmlTemplate) {
      webview.html = await getCustomPanelHtml(view, webview, context, cspSource);
      continue;
    }

    webview.html = getStandardPanelHtml(view, imageUri, cspSource);
  }
}

async function refreshSimulationViewPanels(context: vscode.ExtensionContext): Promise<void> {
  const simView = DRONE_VIEWS.find((view) => view.id === 'tensorfleet-gzweb-panel');
  if (!simView) {
    return;
  }

  await refreshDedicatedPanels(simView, context);
}



// -----------------------------------------------------------------------------
// Activation
// -----------------------------------------------------------------------------

export function activate(context: vscode.ExtensionContext) {
  // Initialize environment/mode detection first (must be before any isDev() calls)
  initializeEnv(context);

  // Periodic debug console print
  const debugInterval = setInterval(() => {
    console.log('[TensorFleet][DEBUG] dummy text print');
  }, 5000);

  context.subscriptions.push(new vscode.Disposable(() => clearInterval(debugInterval)));


  env.log('Extension activating in', getMode(), 'mode');

  telemetryService = new TelemetryService(context);
  context.subscriptions.push(telemetryService);
  telemetryService.trackEvent('extension.activate', { mode: getMode() });
  help.ensureOnboardingProgressInitialized(context);

  // Start MCP bridge for communication between MCP server and VS Code
  mcpBridge = new MCPBridge(context);
  mcpBridge
    .start()
    .then(() => {
      telemetryService?.trackEvent('mcpBridge.start', { status: 'success' });
      console.log('TensorFleet MCP Bridge started');
    })
    .catch((error) => {
      telemetryService?.captureError(error, { source: 'mcpBridge.start' });
      console.error('Failed to start MCP Bridge:', error);
    });

  // Panels handle ROS2 networking internally via panels-standalone Foxglove client.

  // Initialize status bar items for TensorFleet projects
  initializeStatusBarItems(context)
    .then(() => {
      telemetryService?.trackEvent('statusBar.initialize', { status: 'success' });
    })
    .catch((error) => {
      telemetryService?.captureError(error, { source: 'initializeStatusBarItems' });
      console.error('[TensorFleet] Failed to initialize status bars:', error);
    });

  // Initialize unified status coordinator (replaces separate auth and VM status bars)
  unifiedStatusCoordinator = new UnifiedStatusCoordinator(context);
  context.subscriptions.push(
    unifiedStatusCoordinator.onStateChange(() => scheduleEnvRefresh(context, 'unified-status'))
  );

  // Initialize auth state
  vscode.commands.executeCommand('setContext', 'tensorfleet.current_panel', "")
  updateUnifiedAuthStatus(context);
  updateAuthenticatedContext(context);

  // Update auth status periodically (every 30 seconds)
  const authInterval = setInterval(() => {
    updateUnifiedAuthStatus(context);
  }, 30000);
  context.subscriptions.push(new vscode.Disposable(() => clearInterval(authInterval)));

  vmManagerIntegration = new VMManagerIntegration(context, unifiedStatusCoordinator, telemetryService);
  try {
    vmManagerIntegration.initialize();
    telemetryService?.trackEvent('vmManager.initialize', { status: 'success' });
  } catch (error) {
    telemetryService?.captureError(error, { source: 'vmManager.initialize' });
  }

  DRONE_VIEWS.forEach((view) => {
    const provider = new DashboardViewProvider(view, context);
    context.subscriptions.push(
      vscode.window.registerWebviewViewProvider(view.id, provider, {
        webviewOptions: { retainContextWhenHidden: true }
      })
    );

    context.subscriptions.push(
      registerTensorFleetCommand(
        view.command,
        () => openDedicatedPanel(view, context),
        { feature: 'panel' }
      )
    );
  });

  // Register Unique Panels (function-driven, no "open" command)
  UNIQUE_PANELS.forEach((panelDef) => {
    const provider = new UniqueViewProvider(panelDef, context);
    uniquePanelRegistry.set(panelDef.id, provider);
    context.subscriptions.push(
      vscode.window.registerWebviewViewProvider(panelDef.id, provider, {
        webviewOptions: { retainContextWhenHidden: true }
      })
    );
  });

  // Tooling / misc
  const toolingProvider = new ToolingViewProvider(context);

  context.subscriptions.push(
    vscode.window.registerWebviewViewProvider('tensorfleet-tooling-view', toolingProvider, {
      webviewOptions: { retainContextWhenHidden: true }
    })
  );

  context.subscriptions.push(
    registerTensorFleetCommand('tensorfleet.installTools', () => installBundledTools(context), {
      feature: 'tooling'
    })
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.openToolsPanel', async () => {
      await vscode.commands.executeCommand('setContext', 'tensorfleet.current_panel', 'tooling');
      await vscode.commands.executeCommand('tensorfleet-tooling-view.focus');
    })
  );

  context.subscriptions.push(
    registerTensorFleetCommand('tensorfleet.createNewProject', () => createNewProject(context), {
      feature: 'projects'
    })
  );

  context.subscriptions.push(
    registerTensorFleetCommand('tensorfleet.createAndOpenNewProject', () => createNewProject(context, true), {
      feature: 'projects'
    })
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.createNewRoboticProject', () => createNewRoboticProject(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.createNewRoboticPythonProject', () => createNewRoboticPythonProject(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.createNewRoboticArmProject', () => createNewRoboticArmProject(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.openAllPanels', () => openAllPanels(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.startMCPServer', () => startMCPServer(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.stopMCPServer', () => stopMCPServer())
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.getMCPConfig', () => showMCPConfiguration(context))
  );

  // Region selection command (accessible via unified menu)
  context.subscriptions.push(
    registerTensorFleetCommand('tensorfleet.selectRegion', () => selectRegion(context), {
      feature: 'region'
    })
  );
  context.subscriptions.push(
    regions.onRegionChange(() => scheduleEnvRefresh(context, 'region-change'))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.selectRosVersion', () => selectRosVersion())
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.showDroneStatus', () => showDroneStatus())
  );

  // Unified menu command (replaces separate auth and VM menu commands)
  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.showUnifiedMenu', () => showUnifiedMenu(context))
  );

  // Keep VM manager menu for backward compatibility (but it will use unified coordinator)
  if (vmManagerIntegration) {
    context.subscriptions.push(
      vscode.commands.registerCommand('tensorfleet.showVMManagerMenu', () => showUnifiedMenu(context))
    );
  }

  // Auth commands
  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.login', () => handleLogin(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.logout', () => handleLogout(context))
  );

  // Initialize account panel state (closed by default)
  vscode.commands.executeCommand('setContext', 'tensorfleet.accountPanelOpen', false);

  // Keep auth status command for backward compatibility
  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.authStatus', () => showUnifiedMenu(context))
  );

  // Dev-only debug command (only registered in development mode)
  context.subscriptions.push(
    registerDevCommand('tensorfleet.debugInfo', () => showDebugInfo(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.openAccountPanel', () => showAccountPanel(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.closeAccountPanel', () => closeAccountPanel(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.openHelpPanel', () => showHelpPanel(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.closeHelpPanel', () => closeHelpPanel(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.openDroneViewsPanel', () => openDroneViewsPanel(context))
  );



  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.openServerSettingsPanel', () => openServerSettingsPanel(context))
  );

  // Test command for server settings panel
  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.testServerSettings', async () => {
      const panel = vscode.window.createWebviewPanel(
        'tensorfleet-test-server-settings',
        'Test Server Settings',
        vscode.ViewColumn.One,
        {
          enableScripts: true,
          localResourceRoots: [
            vscode.Uri.joinPath(context.extensionUri, 'media'),
            vscode.Uri.joinPath(context.extensionUri, 'src', 'templates')
          ]
        }
      );

      const htmlPath = path.join(__dirname, '..', 'src', 'templates', 'server-settings.html');
      let html = fs.readFileSync(htmlPath, 'utf8');

      // Add CSP meta tag
      const cspSource = panel.webview.cspSource;
      const cspMeta = `<meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src ${cspSource} 'unsafe-inline'; script-src ${cspSource} 'unsafe-inline' 'unsafe-eval'; img-src ${cspSource} data: https:; font-src ${cspSource} data:; connect-src ${cspSource} https: http: ws: wss:;">`;
      if (!html.includes('Content-Security-Policy')) {
        html = html.replace('<head>', `<head>\n    ${cspMeta}`);
      }

      panel.webview.html = html;

      // Handle messages from the webview
      panel.webview.onDidReceiveMessage(async (message) => {
        try {
          await serverSettingsMessageHandler(message, {
            context,
            webview: panel.webview,
            telemetry: telemetryService
          });
        } catch (error) {
          console.error('Test panel message error:', error);
        }
      });
    })
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.openTutorialsGuide', () => openTutorialsGuide(context))
  )
  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.resetOnboarding', () => resetOnboarding(context))
  )

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.openWebsite', () => openWebsite(context))
  )

  context.subscriptions.push(
    vscode.workspace.onDidChangeWorkspaceFolders(() => scheduleEnvRefresh(context, 'workspace-folders'))
  );


  // ROS bridge commands removed; panels use embedded Foxglove networking.

  context.subscriptions.push(
    vscode.window.onDidCloseTerminal((closedTerminal) => {
      for (const [key, terminal] of terminalRegistry.entries()) {
        if (terminal === closedTerminal) {
          terminalRegistry.delete(key);
          telemetryService?.trackEvent('terminal.closed', { target: key });
          break;
        }
      }
    })
  );

  scheduleEnvRefresh(context, 'activation');


  // Welcome page.
  if (context.extensionMode === vscode.ExtensionMode.Development) {
    // Always show in dev mode
    showWelcomePage(context);
  } else {
    // If onboarding hasn't ended show it
    const lastStep = help.loadOnboardingProgress(context).lastCompletedStep;
    if (lastStep === 'none' || lastStep === 'project') {
      showWelcomePage(context);
    }
  }
}

export function deactivate() {
  telemetryService?.trackEvent('extension.deactivate');

  if (envRefreshTimer) {
    clearTimeout(envRefreshTimer);
    envRefreshTimer = null;
  }

  // Clean up MCP bridge
  if (mcpBridge) {
    mcpBridge
      .stop()
      .catch((error) => {
        telemetryService?.captureError(error, { source: 'mcpBridge.stop' });
        console.error(error);
      });
    mcpBridge = null;
  }

  // Clean up MCP server if running
  if (mcpServerProcess) {
    try {
      mcpServerProcess.kill();
      telemetryService?.trackEvent('mcpServer.stop', { reason: 'deactivate' });
    } catch (error) {
      telemetryService?.captureError(error, { source: 'mcpServer.stop' });
    }
    mcpServerProcess = null;
  }

  // Clean up status bar items
  if (rosVersionStatusBar) {
    rosVersionStatusBar.dispose();
    rosVersionStatusBar = null;
  }
  if (droneStatusBar) {
    droneStatusBar.dispose();
    droneStatusBar = null;
  }
  if (projectWatcher) {
    projectWatcher.dispose();
    projectWatcher = null;
  }

  if (unifiedStatusCoordinator) {
    unifiedStatusCoordinator.dispose();
    unifiedStatusCoordinator = null;
  }

  if (vmManagerIntegration) {
    vmManagerIntegration.dispose();
    vmManagerIntegration = null;
  }

  telemetryService?.dispose();
  telemetryService = null;
}

function getTelemetry() {
  return telemetryService;
}

function registerTensorFleetCommand(
  commandId: string,
  handler: (...args: any[]) => unknown,
  options?: { feature?: string }
) {
  return vscode.commands.registerCommand(commandId, async (...args: unknown[]) => {
    const telemetry = getTelemetry();
    const feature = options?.feature ?? 'core';
    telemetry?.trackEvent('command.execute', { commandId, feature, phase: 'start' });
    try {
      const result = await Promise.resolve(handler(...args));
      telemetry?.trackEvent('command.execute', { commandId, feature, phase: 'success' });
      return result;
    } catch (error) {
      telemetry?.captureError(error, { commandId, feature });
      telemetry?.trackEvent('command.execute', { commandId, feature, phase: 'error' });
      throw error;
    }
  });
}

function getImageUrl(
  webview: vscode.Webview,
  context: vscode.ExtensionContext,
  image: string
): string {
  return webview
    .asWebviewUri(vscode.Uri.joinPath(context.extensionUri, 'media', image))
    .toString();
}

class DashboardViewProvider implements vscode.WebviewViewProvider {
  constructor(private readonly config: DroneViewport, private readonly context: vscode.ExtensionContext) { }

  resolveWebviewView(webviewView: vscode.WebviewView) {
    webviewView.webview.options = {
      enableScripts: true,
      localResourceRoots: [vscode.Uri.joinPath(this.context.extensionUri, 'media')]
    };

    webviewView.webview.html = this.renderHtml(webviewView.webview);

    webviewView.webview.onDidReceiveMessage((message) => {
      const telemetry = getTelemetry();

      // Basic sanity check
      if (!message || typeof message.command !== 'string') {
        console.warn('[TensorFleet] Webview message missing command:', message);
        return;
      }

      const command = message.command as string;

      if (command === 'openPanel') {
        telemetry?.trackEvent('webview.action', { viewId: this.config.id, action: 'openPanel' });
        vscode.commands.executeCommand(this.config.command).then(undefined, (error) => {
          vscode.window.showErrorMessage(
            `Failed to open panel: ${error instanceof Error ? error.message : String(error)}`
          );
        });
      } else if (command === 'openAllPanels') {
        telemetry?.trackEvent('webview.action', { viewId: this.config.id, action: 'openAllPanels' });
        vscode.commands.executeCommand('tensorfleet.openAllPanels').then(undefined, (error) => {
          vscode.window.showErrorMessage(
            `Failed to open all dashboards: ${error instanceof Error ? error.message : String(error)}`
          );
        });
      } else if (command.startsWith('tensorfleet.')) {
        // Generic forward for any tensorfleet.* command
        telemetry?.trackEvent('webview.action', { viewId: this.config.id, action: command });

        // Optional: support args passed from the webview
        const args: unknown[] =
          Array.isArray(message.args)
            ? message.args
            : message.args !== undefined
              ? [message.args]
              : [];

        vscode.commands.executeCommand(command, ...args).then(undefined, (error) => {
          vscode.window.showErrorMessage(
            `Failed to execute command "${command}": ${error instanceof Error ? error.message : String(error)}`
          );
        });
      } else {
        console.log('[TensorFleet] Ignoring webview command:', command, 'payload:', message);
      }
    });

  }

  private renderHtml(webview: vscode.Webview): string {
    const imageUri = webview
      .asWebviewUri(vscode.Uri.joinPath(this.context.extensionUri, 'media', this.config.image))
      .toString();
    const cspSource = webview.cspSource;
    const styles = getBaseStyles();

    return loadTemplate('dashboard-view.html', {
      cspSource,
      title: this.config.title,
      styles,
      imageUri,
      description: this.config.description,
      actionLabel: this.config.actionLabel
    });
  }
}

/**
 * Provider for function-driven unique panels
 */
class UniqueViewProvider implements vscode.WebviewViewProvider {
  private webviewView: vscode.WebviewView | null = null;

  constructor(private readonly def: UniquePanel, private readonly context: vscode.ExtensionContext) { }

  async refresh(): Promise<void> {
    if (!this.webviewView) {
      return;
    }

    try {
      const html = await Promise.resolve(
        this.def.render({ webview: this.webviewView.webview, context: this.context, panelDef: this.def })
      );
      this.webviewView.webview.html = html;
    } catch (err) {
      getTelemetry()?.captureError(err, { source: 'UniqueViewProvider.refresh', id: this.def.id });
      this.webviewView.webview.html = this.renderFallbackHtml(
        this.webviewView.webview,
        String(err ?? 'Failed to render')
      );
    }
  }


  resolveWebviewView(webviewView: vscode.WebviewView) {
    this.webviewView = webviewView;

    // Clear reference when disposed
    webviewView.onDidDispose(() => {
      this.webviewView = null;
    });

    const defaultRoots = [
      vscode.Uri.joinPath(this.context.extensionUri, 'media'),
      vscode.Uri.joinPath(this.context.extensionUri, 'src', 'templates'),
      vscode.Uri.joinPath(this.context.extensionUri, 'panels-standalone', 'dist'),
      vscode.Uri.joinPath(this.context.extensionUri, 'panels-standalone', 'dist', 'assets')
    ];
    const extraRoots = this.def.localResourceRoots?.({ context: this.context }) ?? [];
    webviewView.webview.options = {
      enableScripts: true,
      localResourceRoots: [...defaultRoots, ...extraRoots]
    };

    // render by function
    Promise.resolve(
      this.def.render({ webview: webviewView.webview, context: this.context, panelDef: this.def })
    ).then(
      (html) => {
        webviewView.webview.html = html;
      },
      (err) => {
        getTelemetry()?.captureError(err, { source: 'UniqueViewProvider.render', id: this.def.id });
        webviewView.webview.html = this.renderFallbackHtml(webviewView.webview, String(err ?? 'Failed to render'));
      }
    );

    // message piping
    webviewView.webview.onDidReceiveMessage(async (message) => {
      const telemetry = getTelemetry();

      try {
        if (this.def.onMessage) {
          await this.def.onMessage(message, {
            context: this.context,
            webview: webviewView.webview,
            telemetry
          });
          return;
        }

        if (message.command === 'checkAuth') {
          webviewView.webview.postMessage({
            type: 'authStatus',
            authenticated: await auth.isAuthenticated(this.context)
          });
          return;
        }

        // DEFAULT BEHAVIOR WHEN onMessage IS NOT PROVIDED

        if (!message || typeof message.command !== 'string') {
          console.warn('[TensorFleet] Unique panel webview message missing command:', message);
          return;
        }

        const command = message.command as string;
        const args: unknown[] =
          Array.isArray(message.args)
            ? message.args
            : message.args !== undefined
              ? [message.args]
              : [];

        telemetry?.trackEvent('webview.action', {
          viewId: this.def.id,
          action: command
        });

        // Forward any tensorfleet.* command
        if (command.startsWith('tensorfleet.')) {
          vscode.commands.executeCommand(command, ...args).then(undefined, (error) => {
            vscode.window.showErrorMessage(
              `Failed to execute command "${command}": ${error instanceof Error ? error.message : String(error)
              }`
            );
          });
          return;
        }

        // Convenience alias: openAllPanels → tensorfleet.openAllPanels
        if (command === 'openAllPanels') {
          vscode.commands.executeCommand('tensorfleet.openAllPanels').then(undefined, (error) => {
            vscode.window.showErrorMessage(
              `Failed to execute command "tensorfleet.openAllPanels": ${error instanceof Error ? error.message : String(error)
              }`
            );
          });
          return;
        }


        // default handling: just log
        console.log(`[UniquePanel:${this.def.id}] message`, message);
      } catch (e) {
        getTelemetry()?.captureError(e, { source: 'UniqueViewProvider.onMessage', id: this.def.id });
        vscode.window.showErrorMessage(
          `Panel action failed: ${e instanceof Error ? e.message : String(e)}`
        );
      }
    });
  }

  private renderFallbackHtml(webview: vscode.Webview, msg: string) {
    const cspSource = webview.cspSource;
    const styles = getBaseStyles();
    const cspMeta = `<meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src ${cspSource} 'unsafe-inline'; script-src ${cspSource} 'unsafe-inline' 'unsafe-eval'; img-src ${cspSource} data:;">`;
    return `
<!doctype html>
<html>
<head>
  <meta charset="UTF-8" />
  ${cspMeta}
  <title>Panel Error</title>
  ${styles}
</head>
<body>
  <div class="viewport viewport--panel">
    <h1 class="viewport__title">Panel failed to render</h1>
    <p class="viewport__description">${msg}</p>
  </div>
</body>
</html>`;
  }
}

/**
 * Welcome page
 */
async function showWelcomePage(context: vscode.ExtensionContext) {
  const lastState = help.loadOnboardingProgress(context);
  if (lastState.lastCompletedStep === 'none') {
    // If the state is none switch it to the starting page.
    lastState.lastCompletedStep = 'account';
    help.saveOnboardingProgress(context, lastState);
  }
  let lastStep: 'none' | 'account' | 'project' | 'panels' | 'end' = lastState.lastCompletedStep;
  if (lastStep === 'end') {
    lastStep = 'panels';
  }

  const telemetry = getTelemetry();

  const panel = vscode.window.createWebviewPanel(
    'tensorfleet.welcome',
    'Welcome to TensorFleet',
    vscode.ViewColumn.Active,
    {
      enableScripts: true,
      retainContextWhenHidden: true,
      localResourceRoots: [
        vscode.Uri.joinPath(context.extensionUri, 'media'),
        vscode.Uri.joinPath(context.extensionUri, 'src', 'templates')
      ]
    }
  );

  const webview = panel.webview;
  const cspSource = webview.cspSource;
  const styles = getBaseStyles();

  panel.webview.html = loadTemplate('welcome.html', {
    cspSource,
    styles,
    title: `Welcome to TensorFleet ${lastStep}`,
    initialPage: lastStep,
    onboarding_panels_image: getImageUrl(webview, context, 'onboarding-drone-views.png'),
    onboarding_new_project_button_image: getImageUrl(webview, context, 'onboarding-create-new-project-button.png'),
    onboarding_open_new_project: getImageUrl(webview, context, 'onboarding_open_new_project.png')
  });

  panel.webview.onDidReceiveMessage(async (msg) => {
    if (!msg || !msg.command) return;

    switch (msg.command) {
      case 'welcome.login': {
        await vscode.commands.executeCommand('tensorfleet.login');
        panel.webview.postMessage({
          type: 'authStatus',
          authenticated: await auth.isAuthenticated(context)
        });
        break;
      }

      case 'welcome.checkAuth': {
        panel.webview.postMessage({
          type: 'authStatus',
          authenticated: await auth.isAuthenticated(context)
        });
        break;
      }

      case 'welcome.checkProject': {
        panel.webview.postMessage({
          type: 'projectStatus',
          hasProject: await hasTensorfleetMarker()
        });
        break;
      }

      case 'welcome.setPage': {
        const page: string = msg.page;
        const validPages: ('none' | 'account' | 'project' | 'panels' | 'end')[] = ['none', 'account', 'project', 'panels', 'end'];
        if (!validPages.includes(page as any)) {
          console.warn('[TensorFleet][Welcome] Invalid page:', page);
          break;
        }
        const state = help.loadOnboardingProgress(context);
        state.lastCompletedStep = page as 'none' | 'account' | 'project' | 'panels' | 'end';
        help.saveOnboardingProgress(context, state);
        break;
      }

      case 'welcome.createProject': {
        await vscode.commands.executeCommand('tensorfleet.createAndOpenNewProject');
        panel.webview.postMessage({ type: 'projectCreated' });
        break;
      }

      case 'welcome.openStarterPanels': {
        const commandsToRun = new Set<string>();

        commandsToRun.add('tensorfleet.openGzWebPanel');
        commandsToRun.add('tensorfleet.openMapPanel');
        commandsToRun.add('tensorfleet.openTutorialsGuide');

        for (const cmd of commandsToRun) {
          vscode.commands.executeCommand(cmd).then(
            undefined,
            (err) => {
              console.error('[TensorFleet][Welcome] Failed to call command ', cmd, err);
              telemetry?.captureError(err, { source: 'welcome.openStarterPanels', command: cmd });
            }
          );
        }

        panel.dispose();
        break;
      }

      case 'welcome.close': {
        panel.dispose();
        break;
      }

      default: {
        console.log('[TensorFleet][Welcome] Unknown message:', msg);
        break;
      }
    }
  });
}


/**
 * Tooling side view (unchanged)
 */
class ToolingViewProvider implements vscode.WebviewViewProvider {
  constructor(private readonly context: vscode.ExtensionContext) { }

  resolveWebviewView(webviewView: vscode.WebviewView) {
    webviewView.webview.options = {
      enableScripts: true,
      localResourceRoots: [vscode.Uri.joinPath(this.context.extensionUri, 'media')]
    };

    webviewView.webview.html = this.renderHtml(webviewView.webview);
    webviewView.webview.onDidReceiveMessage((message) => {
      if (message?.command === 'createProjectWizard') {
        getTelemetry()?.trackEvent('webview.action', { viewId: 'tensorfleet-tooling-view', action: 'createProjectWizard' });
        vscode.commands.executeCommand('tensorfleet.createNewProject');
      } else if (message?.command === 'newProject') {
        getTelemetry()?.trackEvent('webview.action', { viewId: 'tensorfleet-tooling-view', action: 'newProject' });
        createNewDroneProject(this.context);
      } else if (message?.command === 'newRoboticProject') {
        vscode.commands.executeCommand('tensorfleet.createNewRoboticProject');
      } else if (message?.command === 'newRoboticPythonProject') {
        vscode.commands.executeCommand('tensorfleet.createNewRoboticPythonProject');
      } else if (message?.command === 'newRoboticArmProject') {
        vscode.commands.executeCommand('tensorfleet.createNewRoboticArmProject');
      } else if (message?.command === 'installTools') {
        getTelemetry()?.trackEvent('webview.action', { viewId: 'tensorfleet-tooling-view', action: 'installTools' });
        vscode.commands.executeCommand('tensorfleet.installTools');
      } else if (message?.command === 'openAllPanels') {
        getTelemetry()?.trackEvent('webview.action', { viewId: 'tensorfleet-tooling-view', action: 'openAllPanels' });
        vscode.commands.executeCommand('tensorfleet.openAllPanels');
      }
    });
  }

  private renderHtml(webview: vscode.Webview): string {
    const styles = getBaseStyles();
    const cspSource = webview.cspSource;

    const drone_icon = webview.asWebviewUri(vscode.Uri.joinPath(this.context.extensionUri, 'media', 'drone.png')).toString();
    const js_icon = webview.asWebviewUri(vscode.Uri.joinPath(this.context.extensionUri, 'media', 'javascript.png')).toString();
    const plus_icon = webview.asWebviewUri(vscode.Uri.joinPath(this.context.extensionUri, 'media', 'plus.png')).toString();
    const python_icon = webview.asWebviewUri(vscode.Uri.joinPath(this.context.extensionUri, 'media', 'python.png')).toString();

    return loadTemplate('tooling-view.html', {
      cspSource,
      styles,
      drone_icon,
      js_icon,
      plus_icon,
      python_icon
    });
  }
}

async function openDedicatedPanel(
  view: DroneViewport,
  context: vscode.ExtensionContext,
  viewColumn: vscode.ViewColumn = vscode.ViewColumn.Active,
  preserveFocus = false
) {
  const telemetry = getTelemetry();
  telemetry?.trackEvent('panel.open', {
    panelId: view.id,
    kind: view.panelKind ?? 'standard',
    template: view.htmlTemplate ?? 'standard',
    phase: 'start'
  });

  try {
    const viewType = `tensorfleetPanel.${view.id.replace(/[^A-Za-z0-9.-]/g, '-')}`;
    // Set up local resource roots based on panel type
    const localResourceRoots = [vscode.Uri.joinPath(context.extensionUri, 'media')];
    if (view.htmlTemplate) {
      localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'src', 'templates'));
      if (view.htmlTemplate === 'teleops-standalone') {
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist'));
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist', 'assets'));
      }

      if (view.htmlTemplate === 'image-standalone') {
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist'));
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist', 'assets'));
      }

      if (view.htmlTemplate == 'raw-messages-standalone') {
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist'));
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist', 'assets'));
      }

      if (view.htmlTemplate === 'gzweb-standalone') {
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist'));
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist', 'assets'));
      }

      if (view.htmlTemplate == 'map-standalone') {
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist'));
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist', 'assets'));
      }

      if (view.htmlTemplate == 'sensor-3d-standalone') {
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist'));
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist', 'assets'));
      }

      if (view.htmlTemplate == 'raw-messages-standalone') {
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist'));
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist', 'assets'));
      }

      if (view.htmlTemplate == 'featured-entities-standalone') {
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist'));
        localResourceRoots.push(vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist', 'assets'));
      }
    }

    const panel = vscode.window.createWebviewPanel(
      viewType,
      view.title,
      { viewColumn, preserveFocus },
      {
        enableScripts: true,
        retainContextWhenHidden: true,
        localResourceRoots
      }
    );
    registerDedicatedPanel(view.id, panel);

    const webview = panel.webview;
    const imageUri = webview.asWebviewUri(vscode.Uri.joinPath(context.extensionUri, 'media', view.image)).toString();
    const cspSource = webview.cspSource;

    panel.webview.onDidReceiveMessage((message) => {
      telemetry?.trackEvent('panel.message', {
        panelId: view.id,
        command: message?.command ?? 'unknown'
      });
      if (message?.command === 'launchTerminal' && typeof message.target === 'string') {
        launchTerminalSession(message.target);
      } else if (message?.command === 'openAllPanels') {
        void vscode.commands.executeCommand('tensorfleet.openAllPanels');
      } else {
        // Handle Option 3 panel messages
        handleOption3Message(panel, message, context);
      }
    });

    if (view.panelKind === 'terminalTabs') {
      panel.webview.html = getTerminalPanelHtml(view, imageUri, cspSource);
      telemetry?.trackEvent('panel.open', {
        panelId: view.id,
        kind: view.panelKind,
        template: view.htmlTemplate ?? 'standard',
        phase: 'success'
      });
      return panel;
    }

    // Check if view has a custom HTML template
    if (view.htmlTemplate) {
      panel.webview.html = await getCustomPanelHtml(view, panel.webview, context, cspSource);
      telemetry?.trackEvent('panel.open', {
        panelId: view.id,
        kind: view.panelKind ?? 'standard',
        template: view.htmlTemplate,
        phase: 'success'
      });
      return panel;
    }

    panel.webview.html = getStandardPanelHtml(view, imageUri, cspSource);
    telemetry?.trackEvent('panel.open', {
      panelId: view.id,
      kind: view.panelKind ?? 'standard',
      template: 'standard',
      phase: 'success'
    });
    return panel;
  } catch (error) {
    telemetry?.captureError(error, { source: 'openDedicatedPanel', panelId: view.id });
    telemetry?.trackEvent('panel.open', {
      panelId: view.id,
      kind: view.panelKind ?? 'standard',
      template: view.htmlTemplate ?? 'standard',
      phase: 'error'
    });
    throw error;
  }
}

function getStandardPanelHtml(view: DroneViewport, imageUri: string, cspSource: string): string {
  return loadTemplate('standard-panel.html', {
    cspSource,
    title: view.title,
    styles: getBaseStyles(),
    imageUri,
    description: view.description
  });
}

function getTerminalPanelHtml(view: DroneViewport, imageUri: string, cspSource: string): string {
  return loadTemplate('terminal-panel.html', {
    cspSource,
    title: view.title,
    styles: getBaseStyles(),
    imageUri,
    description: view.description
  });
}

async function getCustomPanelHtml(view: DroneViewport, webview: vscode.Webview, context: vscode.ExtensionContext, cspSource: string): Promise<string> {
  if (!view.htmlTemplate) {
    throw new Error('No HTML template specified for custom panel');
  }

  if (view.htmlTemplate === 'teleops-standalone') {
    return getStandalonePanelHtml('teleops', webview, context, cspSource);
  }

  if (view.htmlTemplate === 'image-standalone') {
    return getStandalonePanelHtml('image', webview, context, cspSource);
  }

  if (view.htmlTemplate === 'map-standalone') {
    return getStandalonePanelHtml('mission_control', webview, context, cspSource);
  }

  if (view.htmlTemplate === 'sensor-3d-standalone') {
    return getStandalonePanelHtml('sensor_view_3d', webview, context, cspSource);
  }

  if (view.htmlTemplate === 'raw-messages-standalone') {
    return getStandalonePanelHtml('raw_messages', webview, context, cspSource);
  }

  if (view.htmlTemplate === 'featured-entities-standalone') {
    return getStandalonePanelHtml('featured_entities', webview, context, cspSource);
  }

  if (view.htmlTemplate === 'gzweb-standalone') {
    return getStandalonePanelHtml('gzweb', webview, context, cspSource);
  }

  // Load the custom HTML template directly
  const templatePath = path.join(__dirname, '..', 'src', 'templates', view.htmlTemplate);
  let template = fs.readFileSync(templatePath, 'utf8');

  // Convert asset paths to webview URIs
  template = template.replace(
    /src="\/assets\/([^"]+)"/g,
    (match, assetPath) => {
      const assetUri = webview.asWebviewUri(
        vscode.Uri.joinPath(context.extensionUri, 'src', 'templates', 'assets', assetPath)
      );
      return `src="${assetUri}"`;
    }
  );

  template = template.replace(
    /href="\/assets\/([^"]+)"/g,
    (match, assetPath) => {
      const assetUri = webview.asWebviewUri(
        vscode.Uri.joinPath(context.extensionUri, 'src', 'templates', 'assets', assetPath)
      );
      return `href="${assetUri}"`;
    }
  );

  // Add CSP meta tag for security
  const cspMeta = `<meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src ${cspSource} 'unsafe-inline'; script-src ${cspSource} 'unsafe-inline' 'unsafe-eval'; img-src ${cspSource} data: https:; font-src ${cspSource} data:; connect-src ${cspSource} https:; frame-src ${cspSource};">`;

  // Insert CSP meta tag in head if not already present
  if (!template.includes('Content-Security-Policy')) {
    template = template.replace('<head>', `<head>\n    ${cspMeta}`);
  }

  return template;
}

async function getStandalonePanelHtml(
  panelName: 'teleops' | 'image' | 'mission_control' | 'raw_messages' | 'sensor_view_3d' | 'gzweb' | 'featured_entities',
  webview: vscode.Webview,
  context: vscode.ExtensionContext,
  cspSource: string
): Promise<string> {
  const htmlPath = path.join(__dirname, '..', 'panels-standalone', 'dist', `${panelName}.html`);

  if (!fs.existsSync(htmlPath)) {
    throw new Error(`Standalone panel build not found: ${htmlPath}. Run 'bun run build' inside panels-standalone/`);
  }

  let html = fs.readFileSync(htmlPath, 'utf8');

  html = html.replace(
    /(src|href)="\/assets\/([^"]+)"/g,
    (match, attr, assetPath) => {
      const assetUri = webview.asWebviewUri(
        vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'dist', 'assets', assetPath)
      );
      return `${attr}="${assetUri}"`;
    }
  );

  const remoteScriptSrc = panelName === 'gzweb' ? ' https://esm.sh' : '';
  const cspMeta = `<meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src ${cspSource} 'unsafe-inline'; script-src ${cspSource}${remoteScriptSrc} 'unsafe-inline' 'unsafe-eval'; img-src ${cspSource} data: https:; font-src ${cspSource} data:; connect-src ${cspSource} https: http: ws: wss:;">`;
  if (html.includes('Content-Security-Policy')) {
    html = html.replace(/<meta[^>]+Content-Security-Policy[^>]+>/i, cspMeta);
  } else {
    html = html.replace('<head>', `<head>\n  ${cspMeta}`);
  }

  // Inject TensorFleet VM manager configuration into the webview so that
  // panels-standalone ROS2 bridge can discover proxy settings.
  const vmManagerUrl = regions.getVmManagerUrl();
  const nodeId = vmManagerIntegration?.snapshot.nodeId ?? '';
  const token = await auth.getToken(context);
  const currentVmConfig = vmManagerIntegration?.getLastUsedConfig() ?? null;
  const vmConfigId = currentVmConfig?.id ?? '';

  const serializedVmManagerUrl = JSON.stringify(vmManagerUrl);
  const serializedVmConfig = JSON.stringify(currentVmConfig ?? null).replace(/</g, '\\u003c');
  const serializedVmConfigId = JSON.stringify(vmConfigId);

  const tfConfigScript = `
  <script>
    window.TENSORFLEET_VM_MANAGER_URL = ${serializedVmManagerUrl};
    window.TENSORFLEET_VM_CONFIG = ${serializedVmConfig};
    ${nodeId ? `window.TENSORFLEET_NODE_ID = ${JSON.stringify(nodeId)};` : ''}
    ${token ? `window.TENSORFLEET_JWT = ${JSON.stringify(token)};` : ''}
    ${vmConfigId ? `window.TENSORFLEET_VM_CONFIG_ID = ${serializedVmConfigId};` : ''}
  </script>
  `;

  if (html.includes('</head>')) {
    html = html.replace('</head>', `${tfConfigScript}\n</head>`);
  } else {
    html = `${tfConfigScript}\n${html}`;
  }

  return html;
}

async function sendVmManagerInfoToWebview(
  webview: vscode.Webview,
  context: vscode.ExtensionContext,
  payload?: { vmBase?: string; token?: string }
) {
  const vmBase = (payload?.vmBase?.trim() || regions.getVmManagerUrl()).replace(/\/+$/, '');
  let token = payload?.token?.trim() || '';
  if (!token) {
    try {
      token = (await auth.getToken(context)) || '';
    } catch (error) {
      console.warn('[TensorFleet] Failed to read auth token for webview:', error);
    }
  }
  if (!vmManagerIntegration) {
    webview.postMessage({
      command: 'tensorfleet.vmManagerInfo',
      payload: {
        error: 'VM Manager integration not initialized',
        vmBase,
        token
      }
    });
    return;
  }

  try {
    // Get basic VM info from snapshot
    const snapshot = vmManagerIntegration.snapshot;
    const info = {
      vmId: snapshot.nodeId,
      vmBase,
      token,
      vmState: snapshot.vmState,
      ipAddress: snapshot.ipAddress,
      nodeId: snapshot.nodeId,
      provider: snapshot.provider,
      region: snapshot.region,
      uptimeSeconds: snapshot.uptimeSeconds,
      error: snapshot.error
    };
    webview.postMessage({ command: 'tensorfleet.vmManagerInfo', payload: info });
  } catch (error) {
    webview.postMessage({
      command: 'tensorfleet.vmManagerInfo',
      payload: {
        error: error instanceof Error ? error.message : String(error),
        vmBase,
        token
      }
    });
  }
}

async function handleOption3Message(panel: vscode.WebviewPanel, message: any, context: vscode.ExtensionContext) {
  if (!message || !message.command) {
    console.warn('[TensorFleet] Invalid message received from webview');
    return;
  }
  getTelemetry()?.trackEvent('panel.option3Message', { command: message.command });
  if (message.command === 'tensorfleet.vmManagerInfo') {
    await sendVmManagerInfoToWebview(panel.webview, context, message.payload);
    return;
  }
  // Standalone panels manage ROS2 via embedded Foxglove. No extension-side action.
  console.log('[TensorFleet] Webview message (handled in panel):', message.command);
}

/**
 * Get the active ROS2 bridge (native DDS, WebSocket, or Foxglove)
 */
// (Removed extension-side ROS helpers; panels handle ROS entirely.)

async function openAllPanels(context: vscode.ExtensionContext) {
  const telemetry = getTelemetry();
  telemetry?.trackEvent('panels.openAll', { phase: 'start' }, { totalPanels: DRONE_VIEWS.length });
  try {
    await vscode.commands.executeCommand('vscode.setEditorLayout', {
      orientation: 0,
      groups: [
        {
          orientation: 1,
          groups: [{}, {}]
        },
        {
          orientation: 1,
          groups: [{}, {}]
        }
      ]
    });

    const columns: vscode.ViewColumn[] = [
      vscode.ViewColumn.One,
      vscode.ViewColumn.Two,
      vscode.ViewColumn.Three,
      vscode.ViewColumn.Four
    ];

    for (let index = 0; index < DRONE_VIEWS.length; index += 1) {
      const view = DRONE_VIEWS[index];
      const column = columns[index] ?? vscode.ViewColumn.Active;
      const preserveFocus = index !== 0;
      await openDedicatedPanel(view, context, column, preserveFocus);
    }

    telemetry?.trackEvent('panels.openAll', { phase: 'success' }, { totalPanels: DRONE_VIEWS.length });
  } catch (error) {
    telemetry?.captureError(error, { source: 'openAllPanels' });
    telemetry?.trackEvent('panels.openAll', { phase: 'error' });
    throw error;
  }
}

function launchTerminalSession(target: string) {
  const telemetry = getTelemetry();
  const config = TERMINAL_CONFIGS[target as keyof typeof TERMINAL_CONFIGS];
  if (!config) {
    vscode.window.showErrorMessage(`Unknown terminal target: ${target}`);
    telemetry?.trackEvent('terminal.launch', { target, phase: 'invalid' });
    return;
  }
  telemetry?.trackEvent('terminal.launch', { target: config.id, phase: 'start' });

  let terminal = terminalRegistry.get(config.id);
  let created = false;
  if (!terminal) {
    terminal = vscode.window.createTerminal({ name: config.name });
    terminalRegistry.set(config.id, terminal);
    config.startupCommands?.forEach((command) => {
      terminal?.sendText(command);
    });
    created = true;
  }

  terminal.show();
  telemetry?.trackEvent('terminal.launch', {
    target: config.id,
    phase: created ? 'created' : 'reused'
  });
}

async function createNewProject(context: vscode.ExtensionContext, openNew: boolean = false) {
  // Show project type selection
  const projectTypes = [
    {
      id: 'drone-js',
      label: '🚁 Drone (JavaScript)',
      description: 'JavaScript-based drone control with PX4 and ROS 2',
      detail: 'Perfect for drone automation, computer vision, and AI integration'
    },
    {
      id: 'robotic-js',
      label: '🤖 Simple Robot (JavaScript)',
      description: 'Ground robot demos with ROS 2, obstacle avoidance, and vision',
      detail: 'Great for basic navigation, teleop, and perception'
    },
    {
      id: 'robotic-py',
      label: '🤖 Simple Robot (Python)',
      description: 'Python ground robot template with ROS 2 and perception demos',
      detail: 'A clean starting point for scripts and autonomy'
    },
    {
      id: 'lerobot-arm',
      label: '🦾 LeRobot Arm (Python)',
      description: 'LeRobot SO-ARM101 teleop and calibration',
      detail: 'Keyboard, leader, and follower control wired to ROS 2'
    }
  ];

  const selectedType = await vscode.window.showQuickPick(projectTypes, {
    placeHolder: 'Select the type of project you want to create',
    title: 'TensorFleet: New Project Type',
    matchOnDescription: true,
    matchOnDetail: true
  });

  if (!selectedType) {
    return; // User cancelled
  }

  // Route to appropriate creation function based on selection
  switch (selectedType.id) {
    case 'drone-js':
      await createNewProjectInternal(context, {
        kindLabel: 'drone',
        defaultName: 'my-drone-project',
        commandLabel: 'TensorFleet Drone Project',
      }, openNew);
      break;
    case 'robotic-js':
      await createNewProjectInternal(context, {
        kindLabel: 'simple robot',
        defaultName: 'my-simple-robot-project',
        commandLabel: 'TensorFleet Simple Robot Project (JavaScript)',
        templateSubdir: 'robotic-js-project-templates'
      }, openNew);
      break;
    case 'robotic-py':
      await createNewProjectInternal(context, {
        kindLabel: 'simple robot',
        defaultName: 'my-simple-robot-project',
        commandLabel: 'TensorFleet Simple Robot Project (Python)',
        templateSubdir: 'robotic-project-templates'
      }, openNew);
      break;
    case 'lerobot-arm':
      await createNewProjectInternal(context, {
        kindLabel: 'robotic arm',
        defaultName: 'my-lerobot-arm-project',
        commandLabel: 'TensorFleet LeRobot Arm Project (Python)',
        templateSubdir: 'lerobot-arm-project-templates'
      }, openNew);
      break;
  }
}

async function createNewDroneProject(context: vscode.ExtensionContext, openNew: boolean = false) {
  const confirm = await vscode.window.showQuickPick([{
    label: '🚁 Drone (JavaScript)',
    description: 'JavaScript-based drone control with PX4 and ROS 2',
    detail: 'Perfect for drone automation, computer vision, and AI integration'
  }], {
    title: 'TensorFleet: New Drone Project',
    placeHolder: 'Press Enter to continue or Escape to cancel'
  });
  if (!confirm) return;

  await createNewProjectInternal(context, {
    kindLabel: 'drone',
    defaultName: 'my-drone-project',
    commandLabel: 'TensorFleet Drone Project',
    templateSubdir: 'drone-js-project-templates'
  }, openNew);
}

async function createNewRoboticProject(context: vscode.ExtensionContext, openNew: boolean = false) {
  const confirm = await vscode.window.showQuickPick([{
    label: '🤖 Simple Robot (JavaScript)',
    description: 'Ground robot demos with ROS 2, obstacle avoidance, and vision',
    detail: 'Great for basic navigation, teleop, and perception'
  }], {
    title: 'TensorFleet: New Simple Robot Project (JavaScript)',
    placeHolder: 'Press Enter to continue or Escape to cancel'
  });
  if (!confirm) return;

  await createNewProjectInternal(context, {
    kindLabel: 'simple robot',
    defaultName: 'my-simple-robot-project',
    commandLabel: 'TensorFleet Simple Robot Project (JavaScript)',
    templateSubdir: 'robotic-js-project-templates'
  }, openNew);
}

async function createNewRoboticPythonProject(context: vscode.ExtensionContext, openNew: boolean = false) {
  const confirm = await vscode.window.showQuickPick([{
    label: '🤖 Simple Robot (Python)',
    description: 'Python ground robot template with ROS 2 and perception demos',
    detail: 'A clean starting point for scripts and autonomy'
  }], {
    title: 'TensorFleet: New Simple Robot Project (Python)',
    placeHolder: 'Press Enter to continue or Escape to cancel'
  });
  if (!confirm) return;

  await createNewProjectInternal(context, {
    kindLabel: 'simple robot',
    defaultName: 'my-simple-robot-project',
    commandLabel: 'TensorFleet Simple Robot Project (Python)',
    templateSubdir: 'robotic-project-templates'
  }, openNew);
}

async function createNewRoboticArmProject(context: vscode.ExtensionContext, openNew: boolean = false) {
  const confirm = await vscode.window.showQuickPick([{
    label: '🦾 LeRobot Arm (Python)',
    description: 'LeRobot SO-ARM101 teleop and calibration',
    detail: 'Keyboard, leader, and follower control wired to ROS 2'
  }], {
    title: 'TensorFleet: New LeRobot Arm Project',
    placeHolder: 'Press Enter to continue or Escape to cancel'
  });
  if (!confirm) return;

  await createNewProjectInternal(context, {
    kindLabel: 'robotic arm',
    defaultName: 'my-lerobot-arm-project',
    commandLabel: 'TensorFleet LeRobot Arm Project (Python)',
    templateSubdir: 'lerobot-arm-project-templates'
  }, openNew);
}

type NewProjectOptions = {
  kindLabel: string;
  defaultName: string;
  commandLabel: string;
  templateSubdir?: string;
};

/**
 * Check if the project template is for JavaScript-based projects
 */
function isJavaScriptProject(templateSubdir?: string): boolean {
  const resolved = templateSubdir ?? 'drone-js-project-templates';
  return resolved === 'drone-js-project-templates' || resolved === 'robotic-js-project-templates';
}


/**
 * Check if the current workspace is a tensorfleet project.
 */
async function hasTensorfleetMarker(): Promise<boolean> {
  const folders = vscode.workspace.workspaceFolders;
  if (!folders || folders.length === 0) return false;

  for (const folder of folders) {
    const markerUri = vscode.Uri.joinPath(folder.uri, '.tensorfleet');
    try {
      await vscode.workspace.fs.stat(markerUri);
      return true;
    } catch {
      // Not a TensorFleet workspace, keep scanning
    }
  }

  return false;
}

function buildProxyWebSocketUrl(vmManagerUrl: string): string {
  if (!vmManagerUrl) return '';
  try {
    const url = new URL(vmManagerUrl);

    if (url.protocol === 'ws:' || url.protocol === 'wss:') {
      if (!url.pathname || url.pathname === '/') {
        url.pathname = '/ws';
      }
      return url.toString();
    }

    const protocol = url.protocol === 'https:' ? 'wss:' : 'ws:';
    const basePath = url.pathname.endsWith('/') ? url.pathname.slice(0, -1) : url.pathname;
    const pathName = basePath.endsWith('/ws') ? basePath : `${basePath}/ws`;

    return `${protocol}//${url.host}${pathName}`;
  } catch {
    return '';
  }
}

async function readTensorfleetMetadata(folder: vscode.Uri): Promise<TensorfleetMetadata> {
  const markerUri = vscode.Uri.joinPath(folder, '.tensorfleet');
  try {
    const buf = await vscode.workspace.fs.readFile(markerUri);
    const text = Buffer.from(buf).toString('utf8').trim();
    if (!text) return {};
    return JSON.parse(text);
  } catch (error) {
    const code = (error as Partial<vscode.FileSystemError>)?.code;
    if (code === 'FileNotFound' || code === 'ENOENT') {
      return {};
    }
    console.warn('[TensorFleet] Failed to read .tensorfleet metadata:', error);
    return {};
  }
}

async function writeTensorfleetMetadata(folder: vscode.Uri, metadata: TensorfleetMetadata): Promise<void> {
  const markerUri = vscode.Uri.joinPath(folder, '.tensorfleet');
  const normalized: TensorfleetMetadata = {
    template: metadata.template || 'tensorfleet',
    version: metadata.version ?? 1,
    managedEnv: metadata.managedEnv ?? true,
    env: metadata.env
  };
  const content = JSON.stringify(normalized, null, 2) + '\n';
  await vscode.workspace.fs.writeFile(markerUri, Buffer.from(content, 'utf8'));
}

function parseEnvFile(content: string): Record<string, string> {
  const envVars: Record<string, string> = {};
  const lines = content.replace(/\r\n/g, '\n').split('\n');

  for (const rawLine of lines) {
    const line = rawLine.trim();
    if (!line || line.startsWith('#')) continue;

    const match =
      line.match(/^export\s+([A-Za-z_][A-Za-z0-9_]*)=(.*)$/) ||
      line.match(/^([A-Za-z_][A-Za-z0-9_]*)=(.*)$/);

    if (match) {
      const [, key, value] = match;
      envVars[key] = value;
    }
  }

  return envVars;
}

function collectUnmanagedLines(content: string, managedKeys: Set<string>): string[] {
  const lines = content.replace(/\r\n/g, '\n').split('\n');
  const extras: string[] = [];
  const ignoreCommentPrefixes = [
    '# TensorFleet environment (managed',
    '# Values refresh automatically when login',
    '# User overrides and additional variables',
    '# Add custom variables below.'
  ];
  let hasPreservedContent = false;

  for (const rawLine of lines) {
    const line = rawLine.trim();
    if (!line) {
      if (hasPreservedContent) {
        extras.push(rawLine);
      }
      continue;
    }

    if (line.startsWith('#')) {
      if (ignoreCommentPrefixes.some((prefix) => line.startsWith(prefix))) {
        continue;
      }
      hasPreservedContent = true;
      extras.push(rawLine);
      continue;
    }

    const match =
      line.match(/^export\s+([A-Za-z_][A-Za-z0-9_]*)=/) ||
      line.match(/^([A-Za-z_][A-Za-z0-9_]*)=/);

    if (match) {
      const key = match[1];
      if (managedKeys.has(key)) {
        continue;
      }
    }

    hasPreservedContent = true;
    extras.push(rawLine);
  }

  // Trim trailing blank lines to keep output tidy
  while (extras.length > 0 && extras[extras.length - 1].trim() === '') {
    extras.pop();
  }

  return extras;
}

async function getTensorfleetProjectFolders(additionalFolders: vscode.Uri[] = []): Promise<vscode.Uri[]> {
  const seen = new Map<string, vscode.Uri>();
  const workspaceUris = vscode.workspace.workspaceFolders?.map((f) => f.uri) ?? [];
  const candidates = [...workspaceUris, ...additionalFolders];

  for (const folder of candidates) {
    const key = folder.fsPath;
    if (seen.has(key)) continue;

    try {
      await vscode.workspace.fs.stat(vscode.Uri.joinPath(folder, '.tensorfleet'));
      seen.set(key, folder);
    } catch {
      // Not a TensorFleet project
    }
  }

  return Array.from(seen.values());
}

/**
 * Resolve connection values from various sources with region change handling
 */
function resolveConnectionValues(
  inputs: {
    region: ReturnType<typeof regions.getSelectedRegion>;
    proxyUrl: string;
    nodeId?: string;
    ipAddress?: string;
  },
  markerEnv: Record<string, any>,
  existingEnv: Record<string, string>,
  regionChanged: boolean
) {
  const rosbridgePort =
    Number(inputs.region.ros2Port) ||
    Number(markerEnv.rosbridgePort) ||
    Number(existingEnv.ROSBRIDGE_PORT) ||
    9091;

  const r2bHost = regionChanged
    ? ''
    : inputs.ipAddress ||
    markerEnv.r2bHost ||
    existingEnv.R2B_HOST ||
    '';

  const rosbridgeUrl = regionChanged
    ? ''
    : (inputs.ipAddress ? regions.getRos2WebsocketUrl(inputs.ipAddress) : '') ||
    existingEnv.ROSBRIDGE_URL ||
    markerEnv.rosbridgeUrl ||
    (r2bHost ? `ws://${r2bHost}:${rosbridgePort}` : '');

  const baseUrl =
    inputs.region.vmManagerUrl ||
    existingEnv.TENSORFLEET_BASE_URL ||
    markerEnv.baseUrl ||
    existingEnv.TENSORFLEET_VM_MANAGER_URL ||
    markerEnv.vmManagerUrl ||
    '';

  const nodeId = regionChanged
    ? ''
    : inputs.nodeId ||
    markerEnv.nodeId ||
    existingEnv.TENSORFLEET_NODE_ID ||
    '';

  const vmManagerUrl =
    inputs.region.vmManagerUrl ||
    markerEnv.vmManagerUrl ||
    existingEnv.TENSORFLEET_VM_MANAGER_URL ||
    baseUrl ||
    '';

  const proxyUrl =
    inputs.proxyUrl ||
    markerEnv.proxyUrl ||
    buildProxyWebSocketUrl(vmManagerUrl) ||
    buildProxyWebSocketUrl(baseUrl);

  return { rosbridgePort, r2bHost, rosbridgeUrl, baseUrl, nodeId, vmManagerUrl, proxyUrl };
}

/**
 * Build the .env file content from managed keys and user extras
 */
function buildEnvFileContent(
  managed: Record<string, string | undefined>,
  managedKeys: Set<string>,
  existingContent: string
): string {
  const header = [
    '# TensorFleet environment (managed by the VS Code extension)',
    '# Values refresh automatically when login, region, or VM status changes.'
  ];

  const managedLines = MANAGED_ENV_KEYS.map((key) => {
    const value = managed[key];
    return value ? `${key}=${value}` : `# ${key}=`;
  });

  const extras = collectUnmanagedLines(existingContent, managedKeys);

  const lines = [...header, '', ...managedLines, ''];
  if (extras.length > 0) {
    lines.push('# User overrides and additional variables (preserved):', ...extras);
  } else {
    lines.push('# Add custom variables below. They will be preserved on refresh.');
  }

  let content = lines.join('\n');
  if (!content.endsWith('\n')) {
    content += '\n';
  }

  return content;
}

/**
 * Build updated .tensorfleet metadata
 */
function buildTensorfleetMetadata(
  metadata: TensorfleetMetadata,
  markerEnv: Record<string, any>,
  inputs: {
    region: ReturnType<typeof regions.getSelectedRegion>;
  },
  values: {
    baseUrl: string;
    vmManagerUrl: string;
    proxyUrl: string;
    rosbridgeUrl: string;
    rosbridgePort: number;
    nodeId: string;
    r2bHost: string;
  },
  regionChanged: boolean
): TensorfleetMetadata {
  return {
    template: metadata.template || 'tensorfleet',
    version: metadata.version ?? 1,
    managedEnv: metadata.managedEnv ?? true,
    env: {
      ...markerEnv,
      region: inputs.region.id,
      baseUrl: values.baseUrl,
      vmManagerUrl: values.vmManagerUrl,
      proxyUrl: values.proxyUrl || markerEnv.proxyUrl,
      rosbridgeUrl: regionChanged ? undefined : values.rosbridgeUrl || markerEnv.rosbridgeUrl,
      rosbridgePort: values.rosbridgePort,
      nodeId: regionChanged ? undefined : values.nodeId || markerEnv.nodeId,
      r2bHost: regionChanged ? undefined : values.r2bHost || markerEnv.r2bHost
    }
  };
}

async function writeEnvForFolder(
  folder: vscode.Uri,
  inputs: {
    region: ReturnType<typeof regions.getSelectedRegion>;
    proxyUrl: string;
    nodeId?: string;
    ipAddress?: string;
    token?: string | undefined;
  },
  reason: string
): Promise<void> {
  const envUri = vscode.Uri.joinPath(folder, '.env');
  const metadata = await readTensorfleetMetadata(folder);
  const markerEnv = metadata.env ?? {};

  env.log('[TensorFleet] Syncing .env for', folder.fsPath, `(reason: ${reason})`);

  // Read existing .env file
  let existingContent = '';
  try {
    const buf = await vscode.workspace.fs.readFile(envUri);
    existingContent = Buffer.from(buf).toString('utf8');
  } catch {
    // No existing .env, will create one
  }

  const existingEnv = parseEnvFile(existingContent);
  const managedKeys = new Set<string>(MANAGED_ENV_KEYS);
  const previousRegion = existingEnv.TENSORFLEET_REGION || markerEnv.region;
  const regionChanged = Boolean(previousRegion && previousRegion !== inputs.region.id);

  // Resolve all connection values
  const values = resolveConnectionValues(inputs, markerEnv, existingEnv, regionChanged);

  // Build managed env variables
  const managed: Record<(typeof MANAGED_ENV_KEYS)[number], string | undefined> = {
    TENSORFLEET_BASE_URL: values.baseUrl || undefined,
    TENSORFLEET_JWT: inputs.token || undefined
  };

  // Build .env file content
  const nextContent = buildEnvFileContent(managed, managedKeys, existingContent);

  // Write .env file if changed
  const normalizedExisting = existingContent.replace(/\r\n/g, '\n');
  if (normalizedExisting !== nextContent) {
    await vscode.workspace.fs.writeFile(envUri, Buffer.from(nextContent, 'utf8'));
  }

  // Build and write .tensorfleet metadata if changed
  const nextMetadata = buildTensorfleetMetadata(metadata, markerEnv, inputs, values, regionChanged);

  const previousMetadataString = JSON.stringify({
    template: metadata.template || 'tensorfleet',
    version: metadata.version ?? 1,
    managedEnv: metadata.managedEnv ?? true,
    env: markerEnv
  });

  const nextMetadataString = JSON.stringify(nextMetadata);

  if (previousMetadataString !== nextMetadataString) {
    await writeTensorfleetMetadata(folder, nextMetadata);
  }
}

async function refreshTensorfleetEnvFiles(
  context: vscode.ExtensionContext,
  reason: string,
  additionalFolders: vscode.Uri[] = []
): Promise<void> {
  try {
    const folders = await getTensorfleetProjectFolders(additionalFolders);
    if (folders.length === 0) {
      return;
    }

    const region = regions.getSelectedRegion();
    const token = await auth.getToken(context);
    const snapshot = vmManagerIntegration?.snapshot;

    const proxyUrl = buildProxyWebSocketUrl(region.vmManagerUrl);
    const nodeId = snapshot?.nodeId;
    const ipAddress = snapshot?.ipAddress;

    await Promise.all(
      folders.map((folder) =>
        writeEnvForFolder(
          folder,
          { region, proxyUrl, nodeId, ipAddress, token },
          reason
        )
      )
    );
  } catch (error) {
    console.warn(`[TensorFleet] Failed to refresh TensorFleet .env (${reason}):`, error);
  }
}

function scheduleEnvRefresh(
  context: vscode.ExtensionContext,
  reason: string,
  additionalFolders: vscode.Uri[] = []
) {
  if (envRefreshTimer) {
    clearTimeout(envRefreshTimer);
  }
  envRefreshTimer = setTimeout(() => {
    envRefreshTimer = null;
    void refreshTensorfleetEnvFiles(context, reason, additionalFolders);
  }, 300);
}


async function createNewProjectInternal(
  context: vscode.ExtensionContext,
  options: NewProjectOptions,
  openNew: boolean = false
) {
  const telemetry = getTelemetry();
  // Get project name from user
  const projectName = await vscode.window.showInputBox({
    prompt: `Enter a name for your new ${options.kindLabel} project`,
    placeHolder: options.defaultName,
    validateInput: (value) => {
      if (!value) {
        return 'Project name cannot be empty';
      }
      if (!/^[a-zA-Z0-9-_]+$/.test(value)) {
        return 'Project name can only contain letters, numbers, hyphens, and underscores';
      }
      return null;
    }
  });

  if (!projectName) {
    telemetry?.trackEvent('project.create', { phase: 'cancelled', reason: 'name' });
    return;
  }

  // Select location for the project
  const targetFolders = await vscode.window.showOpenDialog({
    canSelectFiles: false,
    canSelectFolders: true,
    canSelectMany: false,
    openLabel: 'Select Location for New Project'
  });

  if (!targetFolders || targetFolders.length === 0) {
    telemetry?.trackEvent('project.create', { phase: 'cancelled', reason: 'location' });
    return;
  }

  const targetFolder = targetFolders[0];
  const projectFolder = vscode.Uri.joinPath(targetFolder, projectName);
  const templateFolder = vscode.Uri.joinPath(
    context.extensionUri,
    'resources',
    options.templateSubdir ?? 'drone-js-project-templates'
  );

  try {
    // Check if folder already exists
    try {
      await vscode.workspace.fs.stat(projectFolder);
      const overwrite = await vscode.window.showWarningMessage(
        `A folder named "${projectName}" already exists. Do you want to overwrite it?`,
        { modal: true },
        'Overwrite',
        'Cancel'
      );
      if (overwrite !== 'Overwrite') {
        telemetry?.trackEvent('project.create', { phase: 'cancelled', reason: 'overwriteRejected' });
        return;
      }
    } catch {
      // Folder doesn't exist, which is fine
    }

    await vscode.window.withProgress(
      {
        location: vscode.ProgressLocation.Notification,
        title: `Creating ${options.commandLabel}`,
        cancellable: false
      },
      async (progress) => {
        progress.report({ message: 'Setting up project structure…' });
        await copyDirectory(templateFolder, projectFolder);

        // For JS projects, copy tensorfleet-util package
        if (isJavaScriptProject(options.templateSubdir)) {
          progress.report({ message: 'Including TensorFleet utilities…' });
          const packagesFolder = vscode.Uri.joinPath(projectFolder, 'packages');
          await vscode.workspace.fs.createDirectory(packagesFolder);
          const tensorfleetUtilSource = vscode.Uri.joinPath(context.extensionUri, 'panels-standalone', 'packages', 'tensorfleet-util');
          const tensorfleetUtilDest = vscode.Uri.joinPath(packagesFolder, 'tensorfleet-util');

          await copyDirectory(tensorfleetUtilSource, tensorfleetUtilDest, ['dist']);
        }

        try {
          await refreshTensorfleetEnvFiles(context, 'project-create', [projectFolder]);
        } catch (err) {
          console.warn('[TensorFleet] Failed to create default .env for new project:', err);
        }

        progress.report({ message: 'Project created successfully!' });
      }
    );

    telemetry?.trackEvent('project.create', { phase: 'success' });

    // Always open the project in current window after creation
    await vscode.commands.executeCommand('vscode.openFolder', projectFolder);
  } catch (error) {
    telemetry?.captureError(error, { source: 'createNewProject' });
    telemetry?.trackEvent('project.create', { phase: 'error' });
    vscode.window.showErrorMessage(
      `Failed to create project: ${error instanceof Error ? error.message : String(error)}`
    );
  }
}

async function installBundledTools(context: vscode.ExtensionContext) {
  const telemetry = getTelemetry();
  telemetry?.trackEvent('tools.install', { phase: 'start' });
  const targetFolders = await vscode.window.showOpenDialog({
    canSelectFiles: false,
    canSelectFolders: true,
    canSelectMany: false,
    openLabel: 'Select Install Location for TensorFleet Tools'
  });

  if (!targetFolders || targetFolders.length === 0) {
    telemetry?.trackEvent('tools.install', { phase: 'cancelled', reason: 'location' });
    return;
  }

  const targetFolder = targetFolders[0];
  const installFolder = vscode.Uri.joinPath(targetFolder, 'tensorfleet-tools');
  const sourceFolder = vscode.Uri.joinPath(context.extensionUri, 'resources', 'tools');

  try {
    await vscode.window.withProgress(
      {
        location: vscode.ProgressLocation.Notification,
        title: 'TensorFleet Toolchain',
        cancellable: false
      },
      async (progress) => {
        progress.report({ message: 'Preparing installation…' });
        await copyDirectory(sourceFolder, installFolder);
        progress.report({ message: 'Finishing up…' });
      }
    );

    telemetry?.trackEvent('tools.install', { phase: 'success' });
    vscode.window.showInformationMessage(`TensorFleet tools installed to ${installFolder.fsPath}`);
  } catch (error) {
    telemetry?.captureError(error, { source: 'installBundledTools' });
    telemetry?.trackEvent('tools.install', { phase: 'error' });
    vscode.window.showErrorMessage(
      `Failed to install TensorFleet tools: ${error instanceof Error ? error.message : String(error)}`
    );
  }
}

async function copyDirectory(source: vscode.Uri, destination: vscode.Uri, excludeDirs: string[] = []) {
  await vscode.workspace.fs.createDirectory(destination);
  const entries = await vscode.workspace.fs.readDirectory(source);

  for (const [name, fileType] of entries) {
    // Skip excluded directories
    if (fileType === vscode.FileType.Directory && excludeDirs.includes(name)) {
      continue;
    }

    const sourceEntry = vscode.Uri.joinPath(source, name);
    const destinationEntry = vscode.Uri.joinPath(destination, name);

    if (fileType === vscode.FileType.Directory) {
      await copyDirectory(sourceEntry, destinationEntry, excludeDirs);
    } else {
      await vscode.workspace.fs.copy(sourceEntry, destinationEntry, { overwrite: true });
    }
  }
}

function getBaseStyles(): string {
  return /* html */ `
    <style>
      :root {
        color-scheme: light dark;
        font-family: var(--vscode-font-family, Segoe WPC, sans-serif);
      }

      body {
        margin: 0;
        padding: 16px;
        color: var(--vscode-foreground);
        background: transparent;
      }

      .viewport {
        display: flex;
        flex-direction: column;
        align-items: flex-start;
        gap: 12px;
      }

      .viewport__artwork {
        width: 100%;
        border-radius: 8px;
        border: 1px solid var(--vscode-editorWidget-border);
        background: var(--vscode-editor-background);
      }

      .viewport__title {
        margin: 0;
        font-size: 1.2rem;
        font-weight: 600;
      }

      .viewport__description {
        margin: 0;
        opacity: 0.85;
        line-height: 1.4;
      }

      .viewport__hint {
        margin: 0;
        font-size: 0.9rem;
        opacity: 0.7;
      }

      .viewport__actions {
        display: flex;
        flex-wrap: wrap;
        gap: 8px;
        margin-top: 4px;
      }

      .viewport__action {
        margin-top: 4px;
        padding: 8px 12px;
        font-size: 0.95rem;
        color: var(--vscode-button-foreground);
        background: var(--vscode-button-background);
        border: none;
        border-radius: 4px;
        cursor: pointer;
      }

      .viewport__action:hover {
        background: var(--vscode-button-hoverBackground);
      }

      .viewport__action--secondary {
        background: transparent;
        border: 1px solid var(--vscode-button-border, var(--vscode-button-background));
        color: var(--vscode-button-foreground);
      }

      .viewport__action--secondary:hover {
        background: var(--vscode-toolbar-hoverBackground, rgba(255, 255, 255, 0.08));
      }

      .viewport__list {
        margin: 0;
        padding-left: 20px;
        opacity: 0.85;
      }

      .viewport--panel {
        max-width: 960px;
        margin: 0 auto;
      }
    </style>
  `;
}

function loadTemplate(templateName: string, replacements: Record<string, string>): string {
  const templatePath = path.join(__dirname, '..', 'src', 'templates', templateName);
  let template = fs.readFileSync(templatePath, 'utf8');

  for (const [key, value] of Object.entries(replacements)) {
    template = template.replace(new RegExp(`{{${key}}}`, 'g'), value);
  }

  return template;
}


function htmlRenderer(templateName: string): PanelHtmlRenderer {
  return ({ webview, context }) => {
    const cspSource = webview.cspSource;
    const styles = getBaseStyles();

    // This is the *root* of your extension as seen by the webview
    const baseUri = webview.asWebviewUri(context.extensionUri);
    let baseUrl = baseUri.toString();
    if (baseUrl.endsWith('/')) {
      baseUrl = baseUrl.slice(0, -1);
    }

    return loadTemplate(templateName, {
      cspSource,
      styles,
      base_url: baseUrl
    });
  };
}

function startMCPServer(context: vscode.ExtensionContext) {
  const telemetry = getTelemetry();
  if (mcpServerProcess) {
    telemetry?.trackEvent('mcpServer.start', { phase: 'skipped', reason: 'alreadyRunning' });
    vscode.window.showInformationMessage('TensorFleet MCP Server is already running');
    return;
  }

  const mcpServerPath = path.join(context.extensionPath, 'out', 'mcp-server.js');

  if (!fs.existsSync(mcpServerPath)) {
    telemetry?.trackEvent('mcpServer.start', { phase: 'error', reason: 'missingBinary' });
    vscode.window.showErrorMessage(
      'MCP server not found. Please compile the extension first (run "bun run compile")'
    );
    return;
  }

  try {
    telemetry?.trackEvent('mcpServer.start', { phase: 'spawn' });
    mcpServerProcess = spawn('node', [mcpServerPath], {
      stdio: ['pipe', 'pipe', 'pipe']
    });

    mcpServerProcess.stdout?.on('data', (data) => {
      console.log(`MCP Server: ${data}`);
    });

    mcpServerProcess.stderr?.on('data', (data) => {
      console.error(`MCP Server Error: ${data}`);
      telemetry?.trackEvent('mcpServer.stderr', { message: data.toString()?.trim() ?? '' });
    });

    mcpServerProcess.on('exit', (code) => {
      console.log(`MCP Server exited with code ${code}`);
      telemetry?.trackEvent('mcpServer.exit', { code: String(code ?? 0) });
      mcpServerProcess = null;
    });

    vscode.window
      .showInformationMessage(
        'TensorFleet MCP Server started! Configure it in Cursor or Claude Desktop.',
        'Show Config'
      )
      .then((selection) => {
        if (selection === 'Show Config') {
          showMCPConfiguration(context);
        }
      });
    telemetry?.trackEvent('mcpServer.start', { phase: 'success' });
  } catch (error) {
    telemetry?.captureError(error, { source: 'startMCPServer' });
    telemetry?.trackEvent('mcpServer.start', { phase: 'error' });
    vscode.window.showErrorMessage(
      `Failed to start MCP server: ${error instanceof Error ? error.message : String(error)}`
    );
  }
}

function stopMCPServer() {
  const telemetry = getTelemetry();
  if (!mcpServerProcess) {
    telemetry?.trackEvent('mcpServer.stop', { phase: 'skipped', reason: 'notRunning' });
    vscode.window.showInformationMessage('TensorFleet MCP Server is not running');
    return;
  }

  try {
    mcpServerProcess.kill();
    telemetry?.trackEvent('mcpServer.stop', { phase: 'success' });
  } catch (error) {
    telemetry?.captureError(error, { source: 'stopMCPServer' });
    telemetry?.trackEvent('mcpServer.stop', { phase: 'error' });
  } finally {
    mcpServerProcess = null;
  }
  vscode.window.showInformationMessage('TensorFleet MCP Server stopped');
}

// ============================================================================
// Status Bar Items for TensorFleet Projects
// ============================================================================

type DroneInfo = {
  id: string;
  name: string;
  status: 'idle' | 'armed' | 'flying' | 'offline';
  battery: number;
  mode: string;
};

type RosVersion = {
  name: string;
  distro: string;
  path?: string;
};

const AVAILABLE_ROS_VERSIONS: RosVersion[] = [
  { name: 'ROS 2 Humble', distro: 'humble', path: '/opt/ros/humble' },
  { name: 'ROS 2 Iron', distro: 'iron', path: '/opt/ros/iron' },
  { name: 'ROS 2 Jazzy', distro: 'jazzy', path: '/opt/ros/jazzy' },
  { name: 'ROS 2 Rolling', distro: 'rolling', path: '/opt/ros/rolling' },
  { name: 'ROS 1 Noetic', distro: 'noetic', path: '/opt/ros/noetic' }
];

let currentRosVersion: RosVersion = AVAILABLE_ROS_VERSIONS[0];
let drones: DroneInfo[] = [];

async function initializeStatusBarItems(context: vscode.ExtensionContext) {
  console.log('[TensorFleet] Initializing status bar items...');

  // Create ROS version status bar item
  rosVersionStatusBar = vscode.window.createStatusBarItem(
    vscode.StatusBarAlignment.Right,
    100
  );
  rosVersionStatusBar.command = 'tensorfleet.selectRosVersion';
  rosVersionStatusBar.tooltip = 'Click to change ROS version';
  context.subscriptions.push(rosVersionStatusBar);
  console.log('[TensorFleet] ROS version status bar created');

  // Create drone status bar item
  droneStatusBar = vscode.window.createStatusBarItem(
    vscode.StatusBarAlignment.Right,
    99
  );
  droneStatusBar.command = 'tensorfleet.showDroneStatus';
  droneStatusBar.tooltip = 'Click to view drone details';
  context.subscriptions.push(droneStatusBar);
  console.log('[TensorFleet] Drone status bar created');

  // Check if current workspace is a TensorFleet project - AWAIT this!
  await updateStatusBars();
  console.log('[TensorFleet] Initial status bar update complete');

  // Watch for workspace changes
  context.subscriptions.push(
    vscode.workspace.onDidChangeWorkspaceFolders(() => updateStatusBars())
  );

  // Watch for config file changes
  const configPattern = '**/config/drone_config.yaml';
  projectWatcher = vscode.workspace.createFileSystemWatcher(configPattern);

  projectWatcher.onDidCreate(() => updateStatusBars());
  projectWatcher.onDidChange(() => updateStatusBars());
  projectWatcher.onDidDelete(() => updateStatusBars());

  context.subscriptions.push(projectWatcher);

  // Update status periodically (every 5 seconds)
  const interval = setInterval(async () => {
    if (await isTensorFleetProject()) {
      await updateDroneStatus();
    }
  }, 5000);

  context.subscriptions.push(new vscode.Disposable(() => clearInterval(interval)));
}

async function isTensorFleetProject(): Promise<boolean> {
  if (!vscode.workspace.workspaceFolders) {
    console.log('[TensorFleet] No workspace folders open');
    return false;
  }

  console.log('[TensorFleet] Checking for TensorFleet project markers...');
  console.log('[TensorFleet] Workspace folders:', vscode.workspace.workspaceFolders.map(f => f.uri.fsPath));

  // Check for TensorFleet project markers
  const markers = [
    'config/drone_config.yaml',
    'src/main.py',
    'missions',
  ];

  for (const folder of vscode.workspace.workspaceFolders) {
    for (const marker of markers) {
      try {
        const markerPath = vscode.Uri.joinPath(folder.uri, marker);
        await vscode.workspace.fs.stat(markerPath);
        console.log(`[TensorFleet] ✓ Found marker: ${marker} in ${folder.uri.fsPath}`);
        console.log('[TensorFleet] Project detected! Status bars should appear.');
        return true;
      } catch {
        console.log(`[TensorFleet] ✗ Missing marker: ${marker} in ${folder.uri.fsPath}`);
        // File doesn't exist, continue checking
      }
    }
  }

  console.log('[TensorFleet] No TensorFleet project detected. Status bars will be hidden.');
  return false;
}

async function updateStatusBars() {
  console.log('[TensorFleet] Updating status bars...');
  const isTFProject = await isTensorFleetProject();

  if (isTFProject) {
    console.log('[TensorFleet] TensorFleet project detected, showing status bars');

    // Detect ROS version from config or system
    await detectRosVersion();

    // Initialize drone status
    await updateDroneStatus();

    // Show status bars
    if (rosVersionStatusBar) {
      rosVersionStatusBar.show();
      console.log('[TensorFleet] ROS version status bar shown:', rosVersionStatusBar.text);
    }
    if (droneStatusBar) {
      droneStatusBar.show();
      console.log('[TensorFleet] Drone status bar shown:', droneStatusBar.text);
    }
  } else {
    console.log('[TensorFleet] Not a TensorFleet project, hiding status bars');
    // Hide status bars when not in a TensorFleet project
    rosVersionStatusBar?.hide();
    droneStatusBar?.hide();
  }
}

async function detectRosVersion() {
  if (!vscode.workspace.workspaceFolders) {
    return;
  }

  try {
    // Try to read from drone config
    const configPath = vscode.Uri.joinPath(
      vscode.workspace.workspaceFolders[0].uri,
      'config',
      'drone_config.yaml'
    );

    const configContent = await vscode.workspace.fs.readFile(configPath);
    const configText = Buffer.from(configContent).toString('utf8');

    // Simple YAML parsing for ROS version (looking for ros_version or ros2: lines)
    const versionMatch = configText.match(/ros_?version:\s*["']?([^"'\n]+)["']?/i);
    if (versionMatch) {
      const versionName = versionMatch[1].toLowerCase();
      const found = AVAILABLE_ROS_VERSIONS.find((v) =>
        v.distro.toLowerCase() === versionName ||
        v.name.toLowerCase().includes(versionName)
      );
      if (found) {
        currentRosVersion = found;
      }
    }
  } catch (error) {
    getTelemetry()?.captureError(error, { source: 'detectRosVersion' });
    // Config not found or parse error, use default
  }

  // Update status bar
  if (rosVersionStatusBar) {
    rosVersionStatusBar.text = `$(layers) ${currentRosVersion.name}`;
    console.log('[TensorFleet] ROS version set to:', currentRosVersion.name);
  }
}

async function updateDroneStatus() {
  if (!vscode.workspace.workspaceFolders) {
    return;
  }

  try {
    // Try to read from config to get drone info
    const configPath = vscode.Uri.joinPath(
      vscode.workspace.workspaceFolders[0].uri,
      'config',
      'drone_config.yaml'
    );

    const configContent = await vscode.workspace.fs.readFile(configPath);
    const configText = Buffer.from(configContent).toString('utf8');

    // Extract drone ID/name from config
    const idMatch = configText.match(/id:\s*["']?([^"'\n]+)["']?/);
    const modelMatch = configText.match(/model:\s*["']?([^"'\n]+)["']?/);

    const droneId = idMatch ? idMatch[1] : 'drone_1';
    const droneModel = modelMatch ? modelMatch[1] : 'iris';

    // Default status (panels handle live telemetry via Foxglove inside webviews)
    let droneStatus: 'idle' | 'armed' | 'flying' | 'offline' = 'offline';
    let battery = 0;
    let mode = 'UNKNOWN';

    drones = [
      {
        id: droneId,
        name: droneModel,
        status: droneStatus,
        battery: battery,
        mode: mode
      }
    ];

    // Update status bar
    if (droneStatusBar) {
      const activeCount = drones.filter((d) => d.status !== 'offline').length;
      const flyingCount = drones.filter((d) => d.status === 'flying').length;

      let statusText = `$(radio-tower) ${activeCount} Drone${activeCount !== 1 ? 's' : ''}`;

      if (flyingCount > 0) {
        statusText += ` (${flyingCount} Flying)`;
      }

      droneStatusBar.text = statusText;
      console.log('[TensorFleet] Drone status set to:', statusText);
    }
  } catch (error) {
    getTelemetry()?.captureError(error, { source: 'updateDroneStatus' });
    // Config not found, show default
    drones = [
      {
        id: 'drone_1',
        name: 'iris',
        status: 'offline',
        battery: 0,
        mode: 'UNKNOWN'
      }
    ];

    if (droneStatusBar) {
      droneStatusBar.text = '$(radio-tower) 0 Drones';
    }
  }
}

async function selectRosVersion() {
  const telemetry = getTelemetry();
  telemetry?.trackEvent('ros.version.select', { phase: 'start' });
  const items = AVAILABLE_ROS_VERSIONS.map((version) => ({
    label: version.name,
    description: version.path,
    detail: version.distro === currentRosVersion.distro ? '$(check) Currently selected' : '',
    version
  }));

  const selected = await vscode.window.showQuickPick(items, {
    placeHolder: 'Select ROS version for your project',
    title: 'TensorFleet: ROS Version'
  });

  if (!selected) {
    telemetry?.trackEvent('ros.version.select', { phase: 'cancelled' });
    return;
  }

  currentRosVersion = selected.version;
  telemetry?.trackEvent('ros.version.select', {
    phase: 'selected',
    distro: currentRosVersion.distro
  });

  if (rosVersionStatusBar) {
    rosVersionStatusBar.text = `$(archive) ${currentRosVersion.name}`;
  }

  const shouldUpdateConfig = await vscode.window.showInformationMessage(
    `Switched to ${currentRosVersion.name}. Update drone_config.yaml?`,
    'Yes',
    'No'
  );

  if (shouldUpdateConfig === 'Yes') {
    telemetry?.trackEvent('ros.version.select', {
      phase: 'updateConfig',
      distro: currentRosVersion.distro
    });
    await updateConfigWithRosVersion(currentRosVersion);
  }

  vscode.window.showInformationMessage(
    `ROS version set to ${currentRosVersion.name}. Run: source ${currentRosVersion.path}/setup.bash`
  );
  telemetry?.trackEvent('ros.version.select', { phase: 'success', distro: currentRosVersion.distro });
}

async function updateConfigWithRosVersion(version: RosVersion) {
  if (!vscode.workspace.workspaceFolders) {
    return;
  }

  try {
    const configPath = vscode.Uri.joinPath(
      vscode.workspace.workspaceFolders[0].uri,
      'config',
      'drone_config.yaml'
    );

    const configContent = await vscode.workspace.fs.readFile(configPath);
    let configText = Buffer.from(configContent).toString('utf8');

    // Add or update ROS version in config
    if (configText.includes('ros_version:')) {
      configText = configText.replace(
        /ros_version:\s*["']?[^"'\n]+["']?/,
        `ros_version: "${version.distro}"`
      );
    } else {
      // Add after ros2: section if it exists
      if (configText.includes('ros2:')) {
        configText = configText.replace(
          /(ros2:\s*\n)/,
          `$1  ros_version: "${version.distro}"\n`
        );
      } else {
        // Add new ros2 section
        configText += `\nros2:\n  ros_version: "${version.distro}"\n`;
      }
    }

    await vscode.workspace.fs.writeFile(configPath, Buffer.from(configText, 'utf8'));
    getTelemetry()?.trackEvent('ros.version.configUpdate', { status: 'success', distro: version.distro });
    vscode.window.showInformationMessage('Updated drone_config.yaml with ROS version');
  } catch (error) {
    getTelemetry()?.captureError(error, { source: 'updateConfigWithRosVersion', distro: version.distro });
    getTelemetry()?.trackEvent('ros.version.configUpdate', { status: 'error', distro: version.distro });
    vscode.window.showErrorMessage(
      `Failed to update config: ${error instanceof Error ? error.message : String(error)}`
    );
  }
}

async function showDroneStatus() {
  const telemetry = getTelemetry();
  telemetry?.trackEvent('droneStatus.show', { phase: 'start', droneCount: drones.length.toString() });
  if (drones.length === 0) {
    telemetry?.trackEvent('droneStatus.show', { phase: 'empty' });
    vscode.window.showInformationMessage('No drones detected. Start a simulation to see drone status.');
    return;
  }

  const items = drones.map((drone) => {
    const statusIcon =
      drone.status === 'flying' ? '$(rocket)' :
        drone.status === 'armed' ? '$(target)' :
          drone.status === 'idle' ? '$(circle-outline)' :
            '$(circle-slash)';

    const batteryIcon =
      drone.battery > 50 ? '$(pulse)' :
        drone.battery > 25 ? '$(warning)' :
          '$(alert)';

    return {
      label: `${statusIcon} ${drone.name}`,
      description: `${drone.mode} | ${batteryIcon} ${drone.battery}%`,
      detail: `ID: ${drone.id} | Status: ${drone.status}`,
      drone
    };
  });

  items.push({
    label: '$(refresh) Refresh Status',
    description: 'Update drone information',
    detail: '',
    // @ts-ignore
    drone: null
  });

  items.push({
    label: '$(debug-start) Start Simulation',
    description: 'Launch Gazebo with drones',
    detail: '',
    // @ts-ignore
    drone: null
  });

  const selected = await vscode.window.showQuickPick(items, {
    placeHolder: 'Drone Status',
    title: 'TensorFleet: Connected Drones'
  });

  if (!selected) {
    telemetry?.trackEvent('droneStatus.show', { phase: 'dismissed' });
    return;
  }

  if (selected.label.includes('Refresh')) {
    telemetry?.trackEvent('droneStatus.action', { action: 'refresh' });
    await updateDroneStatus();
    vscode.window.showInformationMessage('Drone status refreshed');
  } else if (selected.label.includes('Start Simulation')) {
    telemetry?.trackEvent('droneStatus.action', { action: 'openSimulation' });
    vscode.commands.executeCommand('tensorfleet.openGazeboPanel');
  } else if (selected.drone) {
    telemetry?.trackEvent('droneStatus.action', { action: 'details', droneId: selected.drone.id });
    // Show detailed drone info
    showDetailedDroneInfo(selected.drone);
  }
}

function showDetailedDroneInfo(drone: DroneInfo) {
  const info = `
**Drone Information**

**ID:** ${drone.id}
**Model:** ${drone.name}
**Status:** ${drone.status}
**Battery:** ${drone.battery}%
**Mode:** ${drone.mode}

Click "Open Gazebo Workspace" to view in simulation.
  `.trim();

  vscode.window.showInformationMessage(info, 'Open Gazebo Workspace', 'Close').then((choice) => {
    if (choice === 'Open Gazebo Workspace') {
      getTelemetry()?.trackEvent('droneStatus.action', { action: 'openGazebo', droneId: drone.id });
      vscode.commands.executeCommand('tensorfleet.openGazeboPanel');
    }
  });
}

// ============================================================================
// ROS2 Connection Management
// ============================================================================

// Removed native ROS2 connect/disconnect; panels manage connections.

// ============================================================================
// ROS2 WebSocket Connection Management
// ============================================================================

// Removed rosbridge management; panels handle URLs internally.

// Removed Foxglove URL/config handlers; panels manage this internally.

// (Removed showROS2Topics; panel UI handles topic browsing.)

// PX4 telemetry monitoring removed from extension; handled in panels if needed.

// ============================================================================
// MCP Configuration
// ============================================================================

async function showMCPConfiguration(context: vscode.ExtensionContext) {
  const telemetry = getTelemetry();
  telemetry?.trackEvent('mcp.config', { phase: 'start' });
  try {
    const mcpServerPath = path.join(context.extensionPath, 'out', 'mcp-server.js');

    const config = {
      mcpServers: {
        'tensorfleet-drone': {
          command: 'node',
          args: [mcpServerPath],
          env: {}
        }
      }
    };

    const configText = JSON.stringify(config, null, 2);

    const document = await vscode.workspace.openTextDocument({
      content: configText,
      language: 'json'
    });

    await vscode.window.showTextDocument(document);

    vscode.window
      .showInformationMessage(
        'MCP Configuration copied! Add this to your Cursor or Claude Desktop config.',
        'Open Setup Guide'
      )
      .then((selection) => {
        telemetry?.trackEvent('mcp.config.guide', {
          action: selection === 'Open Setup Guide' ? 'openGuide' : 'dismiss'
        });
        if (selection === 'Open Setup Guide') {
          const setupPath = vscode.Uri.file(path.join(context.extensionPath, 'MCP_SETUP.md'));
          vscode.commands.executeCommand('markdown.showPreview', setupPath);
        }
      });
    telemetry?.trackEvent('mcp.config', { phase: 'success' });
  } catch (error) {
    telemetry?.captureError(error, { source: 'showMCPConfiguration' });
    telemetry?.trackEvent('mcp.config', { phase: 'error' });
    vscode.window.showErrorMessage(
      `Failed to show MCP configuration: ${error instanceof Error ? error.message : String(error)}`
    );
  }
}

// ============================================================================
// Unified Status Functions
// ============================================================================

/**
 * Update unified auth status
 */
async function updateUnifiedAuthStatus(context: vscode.ExtensionContext) {
  console.log('[TensorFleet] updateUnifiedAuthStatus called');
  if (!unifiedStatusCoordinator) {
    console.log('[TensorFleet] No unified status coordinator');
    return;
  }

  const isAuth = await auth.isAuthenticated(context);
  console.log('[TensorFleet] Setting auth status to:', isAuth ? 'authenticated' : 'not_authenticated');
  unifiedStatusCoordinator.updateAuth(isAuth ? 'authenticated' : 'not_authenticated');
}

async function showAccountPanel(context: vscode.ExtensionContext) {
  await vscode.commands.executeCommand('setContext', 'tensorfleet.current_panel', "account");
}

async function closeAccountPanel(context: vscode.ExtensionContext) {
  await vscode.commands.executeCommand('setContext', 'tensorfleet.current_panel', "");
}

async function showHelpPanel(context: vscode.ExtensionContext) {
  await vscode.commands.executeCommand('setContext', 'tensorfleet.current_panel', "help");
}

async function closeHelpPanel(context: vscode.ExtensionContext) {
  await vscode.commands.executeCommand('setContext', 'tensorfleet.current_panel', "");
}

async function openDroneViewsPanel(context: vscode.ExtensionContext) {
  await vscode.commands.executeCommand('setContext', 'tensorfleet.current_panel', "")
}

async function openServerSettingsPanel(context: vscode.ExtensionContext) {
  await vscode.commands.executeCommand('setContext', 'tensorfleet.current_panel', "server-settings" as any)
}

async function openTutorialsGuide(context: vscode.ExtensionContext) {
  if (!vscode.workspace.workspaceFolders) {
    vscode.window.showErrorMessage('No workspace folder found');
    return;
  }

  // Workspace root
  const workspaceFolder = vscode.workspace.workspaceFolders[0].uri;
  const readmeUri = vscode.Uri.joinPath(workspaceFolder, 'README.md');

  try {
    const doc = await vscode.workspace.openTextDocument(readmeUri);
    await vscode.window.showTextDocument(doc);
  } catch (err) {
    vscode.window.showErrorMessage('README.md not found or cannot be opened');
  }
}

async function resetOnboarding(context: vscode.ExtensionContext) {
  await help.resetOnboardingProgress(context);
  showWelcomePage(context);
}

async function openWebsite(context: vscode.ExtensionContext) {
  vscode.env.openExternal(
    vscode.Uri.parse('https://tensorfleet.net')
  );
}


/**
 * Show unified menu (adaptive based on state)
 */
async function showUnifiedMenu(context: vscode.ExtensionContext) {
  if (!unifiedStatusCoordinator) {
    return;
  }

  const state = unifiedStatusCoordinator.getState();
  const items: ActionItem[] = [];

  // Auth check in progress
  if (state.auth === 'checking') {
    items.push(item('$(sync~spin) Checking authentication...', 'noop.header', 'Verifying your TensorFleet login', vscode.QuickPickItemKind.Default));
    items.push({ label: '', kind: vscode.QuickPickItemKind.Separator } as any);
    items.push(item('$(sign-out) Cancel and Logout', 'auth.logout'));

    const selection = await vscode.window.showQuickPick(items as ActionItem[], {
      placeHolder: 'Checking authentication…',
      ignoreFocusOut: true
    });

    if (!selection) return;
    switch (selection.action) {
      case 'auth.logout':
        await handleLogout(context);
        return;
    }
    return;
  }

  // User not authenticated at all
  if (state.auth === 'not_authenticated') {
    items.push(item('$(key) Login', 'auth.login', 'Authenticate with TensorFleet', vscode.QuickPickItemKind.Default));

    const selection = await vscode.window.showQuickPick(items as ActionItem[], {
      placeHolder: 'Not Logged In',
      ignoreFocusOut: true
    });

    if (!selection) return;
    switch (selection.action) {
      case 'auth.login':
        await handleLogin(context);
        return;
    }
    return;
  }

  // VM Manager unavailable (user is logged in but VM Manager can't be reached)
  if (state.connection === 'not_authenticated') {
    const currentRegion = regions.getSelectedRegion();

    items.push(item('$(warning) VM Manager unavailable', 'noop.header', state.error || 'Service may not be deployed in this region', vscode.QuickPickItemKind.Default));
    items.push({ label: '', kind: vscode.QuickPickItemKind.Separator } as any);

    if (vmManagerIntegration) {
      items.push(item('$(refresh) Retry VM Status', 'vm.retryStatus', 'Retry with current authentication'));
    }

    items.push(item(`$(globe) Change Region`, 'region.change', `Current: ${currentRegion.name}`));
    items.push(item('$(sign-out) Logout', 'auth.logout', 'Logout from TensorFleet'));

    const selection = await vscode.window.showQuickPick(items as ActionItem[], {
      placeHolder: 'VM Manager unavailable',
      ignoreFocusOut: true
    });

    if (!selection) return;
    switch (selection.action) {
      case 'vm.retryStatus':
        if (vmManagerIntegration) {
          vmManagerIntegration.refreshStatus(false);
        }
        return;
      case 'region.change':
        await selectRegion(context);
        return;
      case 'auth.logout':
        await handleLogout(context);
        return;
    }
    return;
  }

  // API disconnected state
  if (state.connection === 'disconnected') {
    const currentRegion = regions.getSelectedRegion();

    items.push(item('$(refresh) Retry Connection', 'vm.refresh', state.error || 'Attempt to reconnect'));
    items.push({ label: '', kind: vscode.QuickPickItemKind.Separator } as any);
    items.push(item(`$(globe) Change Region`, 'region.change', `Current: ${currentRegion.name}`));
    items.push(item('$(sign-out) Logout', 'auth.logout', 'Logout from TensorFleet'));

    const selection = await vscode.window.showQuickPick(items as ActionItem[], {
      placeHolder: 'API Disconnected',
      ignoreFocusOut: true
    });

    if (!selection) return;
    switch (selection.action) {
      case 'vm.refresh':
        if (vmManagerIntegration) {
          vmManagerIntegration.refreshStatus(false);
        }
        return;
      case 'region.change':
        await selectRegion(context);
        return;
      case 'auth.logout':
        await handleLogout(context);
        return;
    }
    return;
  }

  // Authenticated and connected - show VM-specific menu
  const vmState = state.vmState;
  const uptime = formatUptime(state.uptimeSeconds);

  // Format header using helper function
  const { label: headerLabel, detail: headerDetail } = formatHeader(vmState, {
    ipAddress: state.ipAddress,
    provider: state.provider,
    region: state.region,
    uptime,
    error: state.error,
    timestamp: state.timestamp
  });

  // Header as regular item (separators don't render icons)
  items.push(item(headerLabel, 'noop.header', headerDetail, vscode.QuickPickItemKind.Default));

  // Add primary actions based on VM state
  switch (vmState) {
    case 'running':
      items.push(item('$(debug-stop) Stop VM', 'vm.stop'));
      break;
    case 'stopped':
    case 'pending':
    case 'unknown':
      items.push(item('$(play) Start VM', 'vm.start', 'Uses last selected configuration'));
      items.push(item('$(gear) Choose Configuration', 'vm.chooseConfig', 'Select VM configuration before starting'));
      break;
    case 'failed':
      items.push(item('$(refresh) Retry Start', 'vm.retryStart'));
      break;
  }

  // Add separator after primary actions if any exist
  if (vmState !== 'starting' && vmState !== 'stopping') {
    items.push({ label: '', kind: vscode.QuickPickItemKind.Separator } as any);
  }

  // Secondary actions (always shown)
  const currentRegion = regions.getSelectedRegion();
  items.push(item('$(refresh) Refresh Status', 'vm.refresh'));
  items.push(item(`$(globe) Change Region`, 'region.change', `Current: ${currentRegion.name}`));
  items.push(item('$(sign-out) Logout', 'auth.logout', 'Logout from TensorFleet'));

  // Development actions
  if (context.extensionMode === vscode.ExtensionMode.Development) {
    items.push(item('$(question) Reset Onboarding', 'onboarding.reset'));
  }

  const selection = await vscode.window.showQuickPick(items as ActionItem[], {
    placeHolder: headerLabel.replace(/\$\([^)]+\)\s*/, ''),
    ignoreFocusOut: true
  });

  if (!selection) return;

  try {
    switch (selection.action) {
      case 'noop.header':
        return; // Ignore header selection
      case 'vm.start':
        if (vmManagerIntegration) await vmManagerIntegration.startVm(selection.actionData);
        return;
      case 'vm.chooseConfig':
        await chooseVMConfiguration(context);
        return;
      case 'vm.stop':
        if (vmManagerIntegration) await vmManagerIntegration.stopVm();
        return;
      case 'vm.retryStart':
        if (vmManagerIntegration) await vmManagerIntegration.startVm();
        return;
      case 'vm.refresh':
        if (vmManagerIntegration) vmManagerIntegration.refreshStatus(false);
        return;
      case 'region.change':
        await selectRegion(context);
        return;
      case 'auth.logout':
        await handleLogout(context);
        return;
      case 'onboarding.reset':
        await resetOnboarding(context);
        return;
    }
  } catch (error) {
    void vscode.window.showErrorMessage(
      `Action failed: ${error instanceof Error ? error.message : String(error)}`
    );
  }
}

/**
 * Format uptime
 */
function formatUptime(seconds?: number | null): string | undefined {
  if (seconds == null) return undefined;
  const total = Math.max(0, Math.floor(seconds));
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  const parts: string[] = [];
  if (h > 0) parts.push(`${h}h`);
  if (m > 0) parts.push(`${m}m`);
  if (parts.length === 0 || s > 0) parts.push(`${s}s`);
  return parts.join(' ');
}

/**
 * Format header label and detail for VM state
 */
function formatHeader(
  vmState: string,
  meta: {
    ipAddress?: string;
    provider?: string;
    region?: string;
    uptime?: string | undefined;
    error?: string;
    timestamp?: number;
  }
): { label: string; detail: string } {
  const parts: string[] = [];
  let detail = '';

  switch (vmState) {
    case 'running':
      // Concise format: "IP · provider · uptime"
      if (meta.ipAddress) parts.push(meta.ipAddress);
      if (meta.provider) parts.push(meta.provider);
      if (meta.uptime) parts.push(meta.uptime);
      detail = parts.length > 0 ? parts.join(' · ') : '';
      return {
        label: '$(circle-filled) Running',
        detail
      };

    case 'stopped':
      // Skip detail line if no meaningful info (we don't track "last stopped" time)
      return {
        label: '$(circle-outline) Stopped',
        detail: ''
      };

    case 'starting':
      return {
        label: '$(loading~spin) Starting',
        detail: 'Usually takes 30-60 seconds'
      };

    case 'stopping':
      return {
        label: '$(loading~spin) Stopping',
        detail: 'Usually takes 10-20 seconds'
      };

    case 'failed':
      return {
        label: '$(error) Failed',
        detail: meta.error || 'Check logs for details'
      };

    case 'pending':
      return {
        label: '$(sync~spin) Pending',
        detail: 'VM exists but has not started yet'
      };

    default:
      return {
        label: '$(sync~spin) Checking...',
        detail: 'Determining VM status'
      };
  }
}

// ============================================================================
// Region Selection
// ============================================================================

/**
 * Show region selection quick pick
 */
async function selectRegion(_context: vscode.ExtensionContext) {
  const telemetry = getTelemetry();
  telemetry?.trackEvent('region.select', { phase: 'start' });

  const items = regions.getRegionQuickPickItems();

  const selected = await vscode.window.showQuickPick(items, {
    placeHolder: 'Select TensorFleet region',
    title: 'TensorFleet: Select Region',
    matchOnDescription: true,
    matchOnDetail: true
  });

  if (!selected) {
    telemetry?.trackEvent('region.select', { phase: 'cancelled' });
    return;
  }

  const regionId = regions.getRegionIdFromQuickPick(selected.label);
  if (!regionId) {
    telemetry?.trackEvent('region.select', { phase: 'error', reason: 'invalid_selection' });
    return;
  }

  try {
    await regions.setSelectedRegion(regionId);

    const newRegion = regions.getSelectedRegion();
    telemetry?.trackEvent('region.select', { phase: 'success', region: regionId });

    // Refresh VM Manager status automatically
    if (vmManagerIntegration) {
      vmManagerIntegration.refreshStatus(false);
    }

    scheduleEnvRefresh(_context, 'region-change');
    vscode.window.showInformationMessage(
      `Region changed to ${newRegion.name}. API endpoints updated.`
    );

  } catch (error) {
    telemetry?.captureError(error, { source: 'selectRegion' });
    telemetry?.trackEvent('region.select', { phase: 'error' });
    vscode.window.showErrorMessage(
      `Failed to change region: ${error instanceof Error ? error.message : String(error)}`
    );
  }
}

// ============================================================================
// Authentication Functions
// ============================================================================

/**
 * Update context for auth status
 */
async function updateAuthenticatedContext(context: vscode.ExtensionContext) {
  const isAuth = await auth.isAuthenticated(context);

  await vscode.commands.executeCommand('setContext', 'tensorfleet.is_authenticated', isAuth);
}


/**
 * Handle login command
 */
async function handleLogin(context: vscode.ExtensionContext) {
  try {
    console.log('[TensorFleet] Starting login process...');
    if (unifiedStatusCoordinator) {
      unifiedStatusCoordinator.updateAuth('checking');
    }
    await auth.authenticate(context);

    console.log('[TensorFleet] Authentication completed, updating status...');
    await updateUnifiedAuthStatus(context);
    await updateAuthenticatedContext(context);

    // Force immediate status update
    const isAuth = await auth.isAuthenticated(context);
    console.log('[TensorFleet] Auth status after login:', isAuth);

    if (unifiedStatusCoordinator) {
      unifiedStatusCoordinator.updateAuth(isAuth ? 'authenticated' : 'not_authenticated');
    }


    // Refresh account panel
    const accountPanel = uniquePanelRegistry.get('tensorfleet-account');
    if (accountPanel) {
      await accountPanel.refresh();
    }

    // Trigger VM Manager refresh after login
    if (vmManagerIntegration) {
      console.log('[TensorFleet] Refreshing VM Manager status...');
      vmManagerIntegration.refreshStatus(false);
    }

    scheduleEnvRefresh(context, 'login');
    console.log('[TensorFleet] Login process completed');
  } catch (error) {
    console.error('[TensorFleet] Login error:', error);
    await updateUnifiedAuthStatus(context);
    void vscode.window.showErrorMessage(
      `Login failed: ${error instanceof Error ? error.message : String(error)}`
    );
  }
}

/**
 * Handle logout command
 */
async function handleLogout(context: vscode.ExtensionContext) {
  await auth.clearToken(context);
  await updateUnifiedAuthStatus(context);
  await updateAuthenticatedContext(context);
  void vscode.window.showInformationMessage('Logged out successfully');

  await vscode.commands.executeCommand('setContext', 'tensorfleet.current_panel', "");

  // Reset VM Manager state after logout
  if (vmManagerIntegration) {
    vmManagerIntegration.refreshStatus(true);
  }

  // Refresh account panel to show logged out state
  const accountPanel = uniquePanelRegistry.get('tensorfleet-account');
  if (accountPanel) {
    await accountPanel.refresh();
  }

  scheduleEnvRefresh(context, 'logout');
}

/**
 * Show authentication status (redirects to unified menu)
 * @internal - Used by command registration
 */
export async function showAuthStatus(context: vscode.ExtensionContext) {
  await showUnifiedMenu(context);
}

// ============================================================================
// VM Configuration Management
// ============================================================================

/**
 * Show VM configuration selection and start VM with selected config
 */
async function chooseVMConfiguration(context: vscode.ExtensionContext) {
  if (!vmManagerIntegration) {
    return;
  }

  const telemetry = getTelemetry();
  telemetry?.trackEvent('vm.config.choose', { phase: 'start' });

  const configs = Object.values(VMManagerIntegration.VM_CONFIGS);
  const currentConfig = vmManagerIntegration.getLastUsedConfig();

  const items: (vscode.QuickPickItem & { config: VMConfig })[] = configs.map(config => ({
    label: config.name,
    description: config.description,
    detail: config.id === currentConfig.id ? '$(check) Currently selected' : '',
    config
  }));

  const selected = await vscode.window.showQuickPick(items, {
    placeHolder: 'Select VM configuration to start',
    title: 'TensorFleet: Choose VM Configuration',
    matchOnDescription: true
  });

  if (!selected) {
    telemetry?.trackEvent('vm.config.choose', { phase: 'cancelled' });
    return;
  }

  telemetry?.trackEvent('vm.config.choose', {
    phase: 'selected',
    configId: selected.config.id
  });

  try {
    // Set the selected configuration without starting VM
    vmManagerIntegration.setLastUsedConfig(selected.config.id);
    telemetry?.trackEvent('vm.config.choose', {
      phase: 'success',
      configId: selected.config.id
    });
    vscode.window.showInformationMessage(`VM configuration changed to "${selected.config.name}". Use "Start VM" to launch with this configuration.`);
  } catch (error) {
    telemetry?.captureError(error, { source: 'chooseVMConfiguration', configId: selected.config.id });
    telemetry?.trackEvent('vm.config.choose', {
      phase: 'error',
      configId: selected.config.id
    });
    throw error;
  }
}

// ============================================================================
// Development-Only Functions
// ============================================================================

/**
 * Show debug information (dev mode only)
 */
async function showDebugInfo(context: vscode.ExtensionContext) {
  if (!isDev()) return;

  const state = unifiedStatusCoordinator?.getState();
  const currentRegion = regions.getSelectedRegion();

  const info = {
    mode: getMode(),
    region: currentRegion.id,
    regionName: currentRegion.name,
    backendUrl: regions.getBackendUrl(),
    vmManagerUrl: regions.getVmManagerUrl(),
    authState: state?.auth,
    vmState: state?.vmState,
    connectionState: state?.connection,
    ipAddress: state?.ipAddress,
    extensionPath: context.extensionPath,
  };

  env.log('Debug info:', info);

  const document = await vscode.workspace.openTextDocument({
    content: JSON.stringify(info, null, 2),
    language: 'json'
  });

  await vscode.window.showTextDocument(document);
}
