import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';
import { spawn, ChildProcess } from 'child_process';
import { MCPBridge } from './mcp-bridge';
import { ROS2Bridge } from './ros2-bridge';
import { ROS2WebSocketBridge } from './ros2-websocket-bridge';
import { FoxgloveBridge } from './foxglove-bridge';

type PanelKind = 'standard' | 'terminalTabs';

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

const DRONE_VIEWS: DroneViewport[] = [
  {
    id: 'tensorfleet-qgroundcontrol',
    title: 'QGroundControl Command Deck',
    description:
      'Monitor manual flight controls, telemetry, and mission scripts aligned with QGroundControl workflows.',
    image: 'connected_vehicle.tasoHGVc.jpg',
    command: 'tensorfleet.openQGroundControlPanel',
    actionLabel: 'Launch QGroundControl Workspace'
  },
  {
    id: 'tensorfleet-gazebo',
    title: 'Gazebo Simulation',
    description: 'Review Gazebo scenes, sensor overlays, and simulation states for the current drone world.',
    image: 'gazebo-placeholder.svg',
    command: 'tensorfleet.openGazeboPanel',
    actionLabel: 'Open Gazebo Viewer',
    htmlTemplate: 'visualization.html'
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
    image: 'tensorfleet-icon.svg',
    command: 'tensorfleet.openMapPanel',
    actionLabel: 'Open Map Panel',
    panelKind: 'standard',
    htmlTemplate: 'map-standalone'
  }
];

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
let ros2Bridge: ROS2Bridge | null = null;
let ros2WebSocketBridge: ROS2WebSocketBridge | null = null;
let foxgloveBridge: FoxgloveBridge | null = null;

// Active ROS2 connection mode: 'native' | 'websocket' | 'foxglove'
let ros2ConnectionMode: 'native' | 'websocket' | 'foxglove' | null = null;
let preferredConnectionMode: 'auto' | 'native' | 'rosbridge' | 'foxglove' = 'rosbridge';

// Status bar items for TensorFleet projects
let rosVersionStatusBar: vscode.StatusBarItem | null = null;
let droneStatusBar: vscode.StatusBarItem | null = null;
let projectWatcher: vscode.FileSystemWatcher | null = null;

export function activate(context: vscode.ExtensionContext) {
  // Start MCP bridge for communication between MCP server and VS Code
  mcpBridge = new MCPBridge(context);
  mcpBridge.start().then(() => {
    console.log('TensorFleet MCP Bridge started');
  }).catch((error) => {
    console.error('Failed to start MCP Bridge:', error);
  });

  // Initialize ROS2 bridge for real-time drone communication (native DDS mode)
  ros2Bridge = new ROS2Bridge(context);
  
  // Listen for ROS2 connection events
  ros2Bridge.on('connected', () => {
    vscode.window.showInformationMessage('ROS2 (Native DDS) connected successfully!');
    console.log('TensorFleet ROS2 Bridge (Native) connected');
    ros2ConnectionMode = 'native';
  });
  
  ros2Bridge.on('disconnected', () => {
    vscode.window.showWarningMessage('ROS2 (Native DDS) disconnected');
    console.log('TensorFleet ROS2 Bridge (Native) disconnected');
    if (ros2ConnectionMode === 'native') {
      ros2ConnectionMode = null;
    }
  });
  
  ros2Bridge.on('error', (error) => {
    console.error('ROS2 Bridge (Native) error:', error);
    // Don't show error immediately - ROS2 might not be running yet
  });

  // Initialize WebSocket bridge (for remote ROS2 instances like Firecracker VM)
  // Get WebSocket URL from configuration
  const config = vscode.workspace.getConfiguration('tensorfleet');
  const wsUrl = config.get<string>('ros2.websocketUrl', 'ws://172.16.0.2:9091');
  
  ros2WebSocketBridge = new ROS2WebSocketBridge(wsUrl);
  
  ros2WebSocketBridge.on('connected', () => {
    vscode.window.showInformationMessage(`ROS2 (WebSocket) connected to ${wsUrl}`);
    console.log('TensorFleet ROS2 WebSocket Bridge connected');
    ros2ConnectionMode = 'websocket';
  });
  
  ros2WebSocketBridge.on('disconnected', () => {
    vscode.window.showWarningMessage('ROS2 (WebSocket) disconnected');
    console.log('TensorFleet ROS2 WebSocket Bridge disconnected');
    if (ros2ConnectionMode === 'websocket') {
      ros2ConnectionMode = null;
    }
  });

  // Initialize Foxglove Bridge (high-performance C++ bridge)
  const foxgloveUrl = config.get<string>('ros2.foxgloveUrl', 'ws://172.16.0.2:8765');
  
  foxgloveBridge = new FoxgloveBridge(foxgloveUrl);
  
  foxgloveBridge.on('connected', () => {
    vscode.window.showInformationMessage(`ROS2 (Foxglove) connected to ${foxgloveUrl}`);
    console.log('TensorFleet Foxglove Bridge connected');
    ros2ConnectionMode = 'foxglove';
  });
  
  foxgloveBridge.on('disconnected', () => {
    vscode.window.showWarningMessage('ROS2 (Foxglove) disconnected');
    console.log('TensorFleet Foxglove Bridge disconnected');
    if (ros2ConnectionMode === 'foxglove') {
      ros2ConnectionMode = null;
    }
  });
  
  ros2WebSocketBridge.on('error', (error) => {
    console.error('ROS2 WebSocket Bridge error:', error);
  });
  
  foxgloveBridge.on('error', (error) => {
    console.error('Foxglove Bridge error:', error);
  });

  // Initialize status bar items for TensorFleet projects
  initializeStatusBarItems(context).catch((error) => {
    console.error('[TensorFleet] Failed to initialize status bars:', error);
  });
  DRONE_VIEWS.forEach((view) => {
    const provider = new DashboardViewProvider(view, context);
    context.subscriptions.push(
      vscode.window.registerWebviewViewProvider(view.id, provider, {
        webviewOptions: { retainContextWhenHidden: true }
      })
    );

    context.subscriptions.push(
      vscode.commands.registerCommand(view.command, () => openDedicatedPanel(view, context))
    );
  });

  const toolingProvider = new ToolingViewProvider(context);
  context.subscriptions.push(
    vscode.window.registerWebviewViewProvider('tensorfleet-tooling-view', toolingProvider, {
      webviewOptions: { retainContextWhenHidden: true }
    })
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.installTools', () => installBundledTools(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.createNewProject', () => createNewProject(context))
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

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.selectRosVersion', () => selectRosVersion())
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.showDroneStatus', () => showDroneStatus())
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.connectROS2', () => connectToROS2(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.connectROS2WebSocket', () => connectToROS2WebSocket(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.disconnectROS2WebSocket', () => disconnectROS2WebSocket())
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.configureROS2WebSocket', () => configureROS2WebSocket())
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.connectFoxgloveBridge', () => connectToFoxgloveBridge(context))
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.disconnectFoxgloveBridge', () => disconnectFoxgloveBridge())
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.configureFoxgloveBridge', () => configureFoxgloveBridge())
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.disconnectROS2', () => disconnectFromROS2())
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('tensorfleet.startPX4Telemetry', () => startPX4TelemetryMonitor(context))
  );

  context.subscriptions.push(
    vscode.window.onDidCloseTerminal((closedTerminal) => {
      for (const [key, terminal] of terminalRegistry.entries()) {
        if (terminal === closedTerminal) {
          terminalRegistry.delete(key);
          break;
        }
      }
    })
  );
}

export function deactivate() {
  // Clean up ROS2 bridge
  if (ros2Bridge) {
    ros2Bridge.shutdown().catch(console.error);
    ros2Bridge = null;
  }

  if (ros2WebSocketBridge) {
    ros2WebSocketBridge.shutdown().catch(console.error);
    ros2WebSocketBridge = null;
  }

  // Clean up MCP bridge
  if (mcpBridge) {
    mcpBridge.stop().catch(console.error);
    mcpBridge = null;
  }
  
  // Clean up MCP server if running
  if (mcpServerProcess) {
    mcpServerProcess.kill();
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
}

class DashboardViewProvider implements vscode.WebviewViewProvider {
  constructor(private readonly config: DroneViewport, private readonly context: vscode.ExtensionContext) {}

  resolveWebviewView(webviewView: vscode.WebviewView) {
    webviewView.webview.options = {
      enableScripts: true,
      localResourceRoots: [vscode.Uri.joinPath(this.context.extensionUri, 'media')]
    };

    webviewView.webview.html = this.renderHtml(webviewView.webview);

    webviewView.webview.onDidReceiveMessage((message) => {
      if (message?.command === 'openPanel') {
        vscode.commands.executeCommand(this.config.command).then(undefined, (error) => {
          vscode.window.showErrorMessage(`Failed to open panel: ${error instanceof Error ? error.message : error}`);
        });
      } else if (message?.command === 'openAllPanels') {
        vscode.commands.executeCommand('tensorfleet.openAllPanels').then(undefined, (error) => {
          vscode.window.showErrorMessage(
            `Failed to open all dashboards: ${error instanceof Error ? error.message : error}`
          );
        });
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

class ToolingViewProvider implements vscode.WebviewViewProvider {
  constructor(private readonly context: vscode.ExtensionContext) {}

  resolveWebviewView(webviewView: vscode.WebviewView) {
    webviewView.webview.options = {
      enableScripts: true,
      localResourceRoots: [vscode.Uri.joinPath(this.context.extensionUri, 'media')]
    };

    webviewView.webview.html = this.renderHtml(webviewView.webview);
    webviewView.webview.onDidReceiveMessage((message) => {
      if (message?.command === 'newProject') {
        vscode.commands.executeCommand('tensorfleet.createNewProject');
      } else if (message?.command === 'installTools') {
        vscode.commands.executeCommand('tensorfleet.installTools');
      } else if (message?.command === 'openAllPanels') {
        vscode.commands.executeCommand('tensorfleet.openAllPanels');
      }
    });
  }

  private renderHtml(webview: vscode.Webview): string {
    const styles = getBaseStyles();
    const cspSource = webview.cspSource;
    
    return loadTemplate('tooling-view.html', {
      cspSource,
      styles
    });
  }
}

async function openDedicatedPanel(
  view: DroneViewport,
  context: vscode.ExtensionContext,
  viewColumn: vscode.ViewColumn = vscode.ViewColumn.Active,
  preserveFocus = false
) {
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

    if (view.htmlTemplate == 'map-standalone') {
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

  const webview = panel.webview;
  const imageUri = webview.asWebviewUri(vscode.Uri.joinPath(context.extensionUri, 'media', view.image)).toString();
  const cspSource = webview.cspSource;

  panel.webview.onDidReceiveMessage((message) => {
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
    return panel;
  }

  // Check if view has a custom HTML template
  if (view.htmlTemplate) {
    panel.webview.html = getCustomPanelHtml(view, panel.webview, context, cspSource);
    return panel;
  }

  panel.webview.html = getStandardPanelHtml(view, imageUri, cspSource);
  return panel;
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

function getCustomPanelHtml(view: DroneViewport, webview: vscode.Webview, context: vscode.ExtensionContext, cspSource: string): string {
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

function getStandalonePanelHtml(
  panelName: 'teleops' | 'image' | 'mission_control',
  webview: vscode.Webview,
  context: vscode.ExtensionContext,
  cspSource: string
): string {
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

  const cspMeta = `<meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src ${cspSource} 'unsafe-inline'; script-src ${cspSource} 'unsafe-inline' 'unsafe-eval'; img-src ${cspSource} data: https:; font-src ${cspSource} data:; connect-src ${cspSource} https: http: ws: wss:;">`;
  if (html.includes('Content-Security-Policy')) {
    html = html.replace(/<meta[^>]+Content-Security-Policy[^>]+>/i, cspMeta);
  } else {
    html = html.replace('<head>', `<head>\n  ${cspMeta}`);
  }

  return html;
}

async function handleOption3Message(panel: vscode.WebviewPanel, message: any, context: vscode.ExtensionContext) {
  if (!message || !message.command) {
    console.warn('[TensorFleet] Invalid message received from webview');
    return;
  }

  switch (message.command) {
    case 'setConnectionMode':
      preferredConnectionMode = message.mode || 'rosbridge';
      console.log(`[TensorFleet] Connection mode set to: ${preferredConnectionMode}`);
      break;
    
    case 'subscribeToTopic':
      // Use connection mode from message or fallback to preferred
      const connMode = message.connectionMode || preferredConnectionMode;
      if (connMode !== preferredConnectionMode) {
        preferredConnectionMode = connMode;
      }
      await handleImageTopicSubscription(panel, message.topic);
      break;
    
    case 'publishTwist':
      await handleTwistPublication(message.topic, message.data);
      break;
    
    case 'connectROS':
      await handleROS2Connection(panel, context);
      break;
    
    case 'disconnectROS':
      await handleROS2Disconnection(panel);
      break;
  }
}

/**
 * Get the active ROS2 bridge (native DDS, WebSocket, or Foxglove)
 */
function getActiveROS2Bridge(): ROS2Bridge | ROS2WebSocketBridge | FoxgloveBridge | null {
  if (ros2ConnectionMode === 'foxglove' && foxgloveBridge?.isROS2Connected()) {
    return foxgloveBridge;
  }
  if (ros2ConnectionMode === 'websocket' && ros2WebSocketBridge?.isROS2Connected()) {
    return ros2WebSocketBridge;
  }
  if (ros2ConnectionMode === 'native' && ros2Bridge?.isROS2Connected()) {
    return ros2Bridge;
  }
  // Try Foxglove first, then WebSocket, then native
  if (foxgloveBridge?.isROS2Connected()) {
    return foxgloveBridge;
  }
  if (ros2WebSocketBridge?.isROS2Connected()) {
    return ros2WebSocketBridge;
  }
  if (ros2Bridge?.isROS2Connected()) {
    return ros2Bridge;
  }
  return null;
}

// Handle image topic subscription
async function handleImageTopicSubscription(panel: vscode.WebviewPanel, topic: string) {
  if (!topic || typeof topic !== 'string') {
    console.error('[TensorFleet] Invalid topic:', topic);
    return;
  }

  try {
    let bridge = getActiveROS2Bridge();
    
    // If no bridge is connected, try to connect based on preferred mode
    if (!bridge) {
      console.log(`[TensorFleet] No ROS2 connection active, attempting to connect (mode: ${preferredConnectionMode})...`);
      
      // Connect based on preferred mode
      if (preferredConnectionMode === 'foxglove' && foxgloveBridge) {
        try {
          await foxgloveBridge.connect();
          bridge = foxgloveBridge;
          ros2ConnectionMode = 'foxglove';
          console.log('[TensorFleet] Connected via Foxglove Bridge');
        } catch (error) {
          console.error('[TensorFleet] Foxglove connection failed:', error);
        }
      } else if (preferredConnectionMode === 'rosbridge' && ros2WebSocketBridge) {
        try {
          await ros2WebSocketBridge.connect();
          bridge = ros2WebSocketBridge;
          ros2ConnectionMode = 'websocket';
          console.log('[TensorFleet] Connected via ROS Bridge');
        } catch (error) {
          console.error('[TensorFleet] ROS Bridge connection failed:', error);
        }
      } else if (preferredConnectionMode === 'native' && ros2Bridge) {
        try {
          await ros2Bridge.initialize();
          bridge = ros2Bridge;
          ros2ConnectionMode = 'native';
          console.log('[TensorFleet] Connected via Native ROS2');
        } catch (error) {
          console.error('[TensorFleet] Native ROS2 connection failed:', error);
        }
      } else {
        // Auto mode or fallback: try rosbridge first
        if (ros2WebSocketBridge) {
          try {
            await ros2WebSocketBridge.connect();
            bridge = ros2WebSocketBridge;
            ros2ConnectionMode = 'websocket';
            console.log('[TensorFleet] Connected via ROS Bridge (auto)');
          } catch (error) {
            console.warn('[TensorFleet] ROS Bridge connection failed:', error);
          }
        }
        
        if (!bridge && foxgloveBridge) {
          try {
            await foxgloveBridge.connect();
            bridge = foxgloveBridge;
            ros2ConnectionMode = 'foxglove';
            console.log('[TensorFleet] Connected via Foxglove (auto fallback)');
          } catch (error) {
            console.warn('[TensorFleet] Foxglove connection failed:', error);
          }
        }
        
        if (!bridge && ros2Bridge) {
          try {
            await ros2Bridge.initialize();
            bridge = ros2Bridge;
            ros2ConnectionMode = 'native';
            console.log('[TensorFleet] Connected via Native ROS2 (auto fallback)');
          } catch (error) {
            console.warn('[TensorFleet] Native ROS2 connection failed:', error);
          }
        }
      }
      
      // If still no connection, fall back to simulation
      if (!bridge) {
        console.warn('[TensorFleet] No ROS2 connection available, falling back to simulation');
        startImageStreamSimulation(panel, topic);
        return;
      }
    }

    console.log(`[TensorFleet] Subscribing to ROS2 topic: ${topic} (mode: ${ros2ConnectionMode})`);
    
    // Subscribe to real ROS2 topic
    await bridge.subscribeToImageTopic(topic, (imageData, metadata) => {
      // Send image data to webview
      panel.webview.postMessage({
        type: 'imageData',
        topic: metadata.topic,
        timestamp: metadata.timestamp,
        encoding: metadata.encoding,
        width: metadata.width,
        height: metadata.height,
        data: imageData
      });
    });

    vscode.window.showInformationMessage(`Subscribed to ${topic} via ${ros2ConnectionMode || 'ROS2'}`);
  } catch (error) {
    console.error('[TensorFleet] Failed to subscribe to topic:', error);
    vscode.window.showErrorMessage(`Failed to subscribe to ${topic}: ${error}`);
    
    // Fall back to simulation
    startImageStreamSimulation(panel, topic);
  }
}

// Handle twist message publication
async function handleTwistPublication(topic: string, data: any) {
  if (!topic || !data) {
    console.error('[TensorFleet] Invalid twist data:', { topic, data });
    return;
  }

  try {
    let bridge = getActiveROS2Bridge();
    
    // If no bridge is connected, try to connect (prefer WebSocket)
    if (!bridge) {
      if (ros2WebSocketBridge) {
        try {
          await ros2WebSocketBridge.connect();
          bridge = ros2WebSocketBridge;
        } catch (error) {
          // Try native
          if (ros2Bridge) {
            try {
              await ros2Bridge.initialize();
              bridge = ros2Bridge;
            } catch (nativeError) {
              console.warn('[TensorFleet] No ROS2 connection available, twist command only logged:', data);
              console.log('Twist (simulated):', topic, data);
              return;
            }
          }
        }
      } else if (ros2Bridge) {
        try {
          await ros2Bridge.initialize();
          bridge = ros2Bridge;
        } catch (error) {
          console.warn('[TensorFleet] ROS2 not available, twist command only logged:', data);
          console.log('Twist (simulated):', topic, data);
          return;
        }
      }
    }

    if (!bridge) {
      console.warn('[TensorFleet] No ROS2 connection available, twist command only logged:', data);
      console.log('Twist (simulated):', topic, data);
      return;
    }

    // Publish to real ROS2 topic
    await bridge.publishTwist(topic, data);
    console.log(`[TensorFleet] Published twist to ${topic} via ${ros2ConnectionMode}:`, data);
    
  } catch (error) {
    console.error('[TensorFleet] Failed to publish twist:', error);
    // Don't show error to user for every publish failure (too noisy)
  }
}

// Handle ROS2 connection
async function handleROS2Connection(panel: vscode.WebviewPanel, context: vscode.ExtensionContext) {
  try {
    console.log(`[TensorFleet] Connecting to ROS2... (preferred mode: ${preferredConnectionMode})`);
    
    let connected = false;
    let bridge: ROS2Bridge | ROS2WebSocketBridge | FoxgloveBridge | null = null;
    
    // Try connection based on preferred mode
    if (preferredConnectionMode === 'foxglove') {
      // User explicitly requested Foxglove
      if (foxgloveBridge) {
        try {
          await foxgloveBridge.connect();
          bridge = foxgloveBridge;
          connected = true;
          ros2ConnectionMode = 'foxglove';
          console.log('[TensorFleet] Connected via Foxglove Bridge');
        } catch (error) {
          console.error('[TensorFleet] Foxglove connection failed:', error);
          vscode.window.showWarningMessage('Foxglove Bridge connection failed. Try ROS Bridge instead.');
        }
      }
    } else if (preferredConnectionMode === 'rosbridge') {
      // User explicitly requested rosbridge (or default)
      if (ros2WebSocketBridge) {
        try {
          await ros2WebSocketBridge.connect();
          bridge = ros2WebSocketBridge;
          connected = true;
          ros2ConnectionMode = 'websocket';
          console.log('[TensorFleet] Connected via ROS Bridge');
        } catch (error) {
          console.error('[TensorFleet] ROS Bridge connection failed:', error);
          vscode.window.showWarningMessage('ROS Bridge connection failed. Check if bridge is running on port 9091.');
        }
      }
    } else if (preferredConnectionMode === 'native') {
      // User explicitly requested native DDS
      if (ros2Bridge) {
        try {
          await ros2Bridge.initialize();
          bridge = ros2Bridge;
          connected = true;
          ros2ConnectionMode = 'native';
          console.log('[TensorFleet] Connected via native ROS2');
        } catch (error) {
          console.error('[TensorFleet] Native ROS2 connection failed:', error);
          vscode.window.showWarningMessage('Native ROS2 connection failed. Source ROS2 environment first.');
        }
      }
    } else {
      // Auto mode: Try rosbridge first, then Foxglove, then native
      if (!connected && ros2WebSocketBridge) {
        try {
          await ros2WebSocketBridge.connect();
          bridge = ros2WebSocketBridge;
          connected = true;
          ros2ConnectionMode = 'websocket';
          console.log('[TensorFleet] Connected via ROS Bridge (auto)');
        } catch (error) {
          console.warn('[TensorFleet] ROS Bridge connection failed:', error);
        }
      }
      
      if (!connected && foxgloveBridge) {
        try {
          await foxgloveBridge.connect();
          bridge = foxgloveBridge;
          connected = true;
          ros2ConnectionMode = 'foxglove';
          console.log('[TensorFleet] Connected via Foxglove Bridge (auto)');
        } catch (error) {
          console.warn('[TensorFleet] Foxglove connection failed:', error);
        }
      }
      
      if (!connected && ros2Bridge) {
        try {
          await ros2Bridge.initialize();
          bridge = ros2Bridge;
          connected = true;
          ros2ConnectionMode = 'native';
          console.log('[TensorFleet] Connected via native ROS2 (auto)');
        } catch (error) {
          console.warn('[TensorFleet] Native ROS2 connection failed:', error);
        }
      }
    }
    
    if (connected && bridge) {
      panel.webview.postMessage({ type: 'connectionStatus', connected: true });
      
      // Get list of available topics
      try {
        const topics = await bridge.getTopicList();
        const imageTopics = topics
          .filter(t => t.type.includes('Image'))
          .map(t => t.name);
        
        if (imageTopics.length > 0) {
          panel.webview.postMessage({ 
            type: 'availableTopics', 
            topics: imageTopics 
          });
        }
      } catch (error) {
        console.warn('[TensorFleet] Failed to get topic list:', error);
      }
      
      vscode.window.showInformationMessage(`Connected to ROS2 via ${ros2ConnectionMode}`);
    } else {
      throw new Error('All connection methods failed');
    }
  } catch (error) {
    console.error('[TensorFleet] Failed to connect to ROS2:', error);
    panel.webview.postMessage({ type: 'connectionStatus', connected: false });
    
    vscode.window.showErrorMessage(
      'Failed to connect to ROS2. Check that rosbridge is running at ws://172.16.0.2:9091',
      'View Setup Guide'
    ).then(action => {
      if (action === 'View Setup Guide') {
        const setupUri = vscode.Uri.file(path.join(context.extensionPath, 'ROS2_SETUP.md'));
        vscode.commands.executeCommand('markdown.showPreview', setupUri);
      }
    });
  }
}

// Handle ROS2 disconnection
async function handleROS2Disconnection(panel: vscode.WebviewPanel) {
  try {
    if (ros2ConnectionMode === 'websocket' && ros2WebSocketBridge) {
      await ros2WebSocketBridge.shutdown();
    } else if (ros2Bridge) {
      await ros2Bridge.shutdown();
    }
    panel.webview.postMessage({ type: 'connectionStatus', connected: false });
    vscode.window.showInformationMessage('Disconnected from ROS2');
  } catch (error) {
    console.error('[TensorFleet] Error during ROS2 disconnection:', error);
  }
}

// Fallback simulation for when ROS2 is not available
const imageStreamIntervals = new Map<vscode.WebviewPanel, NodeJS.Timeout>();

function startImageStreamSimulation(panel: vscode.WebviewPanel, topic: string) {
  console.log(`[TensorFleet] Starting simulated image stream for ${topic} (ROS2 not available)`);
  
  // Clear existing interval if any
  const existingInterval = imageStreamIntervals.get(panel);
  if (existingInterval) {
    clearInterval(existingInterval);
  }
  
  // Send simulated image data every 100ms
  const interval = setInterval(() => {
    if (!panel.visible) return; // Don't send when panel not visible
    
    try {
      const canvas = generateTestImage(640, 480);
      panel.webview.postMessage({
        type: 'imageData',
        topic: topic,
        timestamp: new Date().toISOString(),
        encoding: 'rgb8',
        width: 640,
        height: 480,
        data: canvas
      });
    } catch (error) {
      console.error('[TensorFleet] Error sending simulated image:', error);
      clearInterval(interval);
      imageStreamIntervals.delete(panel);
    }
  }, 100);
  
  imageStreamIntervals.set(panel, interval);
  
  // Clean up on panel disposal
  panel.onDidDispose(() => {
    const intervalToClean = imageStreamIntervals.get(panel);
    if (intervalToClean) {
      clearInterval(intervalToClean);
      imageStreamIntervals.delete(panel);
    }
  });
}

function generateTestImage(width: number, height: number): string {
  // Generate a simple gradient as base64 data URI
  // This is fallback simulation when ROS2 is not available
  return `data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${height}"><defs><linearGradient id="g" x1="0%" y1="0%" x2="100%" y2="100%"><stop offset="0%" style="stop-color:rgb(100,100,255);stop-opacity:1"/><stop offset="100%" style="stop-color:rgb(255,100,100);stop-opacity:1"/></linearGradient></defs><rect width="${width}" height="${height}" fill="url(%23g)"/><text x="50%" y="50%" text-anchor="middle" fill="white" font-size="24">SIMULATION - ${new Date().toLocaleTimeString()}</text><text x="50%" y="60%" text-anchor="middle" fill="yellow" font-size="14">ROS2 not connected</text></svg>`;
}

async function openAllPanels(context: vscode.ExtensionContext) {
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
}

function launchTerminalSession(target: string) {
  const config = TERMINAL_CONFIGS[target as keyof typeof TERMINAL_CONFIGS];
  if (!config) {
    vscode.window.showErrorMessage(`Unknown terminal target: ${target}`);
    return;
  }

  let terminal = terminalRegistry.get(config.id);
  if (!terminal) {
    terminal = vscode.window.createTerminal({ name: config.name });
    terminalRegistry.set(config.id, terminal);
    config.startupCommands?.forEach((command) => {
      terminal?.sendText(command);
    });
  }

  terminal.show();
}

async function createNewProject(context: vscode.ExtensionContext) {
  // Get project name from user
  const projectName = await vscode.window.showInputBox({
    prompt: 'Enter a name for your new drone project',
    placeHolder: 'my-drone-project',
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
    return;
  }

  const targetFolder = targetFolders[0];
  const projectFolder = vscode.Uri.joinPath(targetFolder, projectName);
  const templateFolder = vscode.Uri.joinPath(context.extensionUri, 'resources', 'project-templates');

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
        return;
      }
    } catch {
      // Folder doesn't exist, which is fine
    }

    await vscode.window.withProgress(
      {
        location: vscode.ProgressLocation.Notification,
        title: 'Creating TensorFleet Project',
        cancellable: false
      },
      async (progress) => {
        progress.report({ message: 'Setting up project structure…' });
        await copyDirectory(templateFolder, projectFolder);
        progress.report({ message: 'Project created successfully!' });
      }
    );

    const openProject = await vscode.window.showInformationMessage(
      `✨ Project "${projectName}" created successfully!`,
      'Open Project',
      'Open in New Window',
      'Close'
    );

    if (openProject === 'Open Project') {
      await vscode.commands.executeCommand('vscode.openFolder', projectFolder);
    } else if (openProject === 'Open in New Window') {
      await vscode.commands.executeCommand('vscode.openFolder', projectFolder, true);
    }
  } catch (error) {
    vscode.window.showErrorMessage(
      `Failed to create project: ${error instanceof Error ? error.message : String(error)}`
    );
  }
}

async function installBundledTools(context: vscode.ExtensionContext) {
  const targetFolders = await vscode.window.showOpenDialog({
    canSelectFiles: false,
    canSelectFolders: true,
    canSelectMany: false,
    openLabel: 'Select Install Location for TensorFleet Tools'
  });

  if (!targetFolders || targetFolders.length === 0) {
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

    vscode.window.showInformationMessage(`TensorFleet tools installed to ${installFolder.fsPath}`);
  } catch (error) {
    vscode.window.showErrorMessage(
      `Failed to install TensorFleet tools: ${error instanceof Error ? error.message : String(error)}`
    );
  }
}

async function copyDirectory(source: vscode.Uri, destination: vscode.Uri) {
  await vscode.workspace.fs.createDirectory(destination);
  const entries = await vscode.workspace.fs.readDirectory(source);

  for (const [name, fileType] of entries) {
    const sourceEntry = vscode.Uri.joinPath(source, name);
    const destinationEntry = vscode.Uri.joinPath(destination, name);

    if (fileType === vscode.FileType.Directory) {
      await copyDirectory(sourceEntry, destinationEntry);
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

function startMCPServer(context: vscode.ExtensionContext) {
  if (mcpServerProcess) {
    vscode.window.showInformationMessage('TensorFleet MCP Server is already running');
    return;
  }

  const mcpServerPath = path.join(context.extensionPath, 'out', 'mcp-server.js');
  
  if (!fs.existsSync(mcpServerPath)) {
    vscode.window.showErrorMessage(
      'MCP server not found. Please compile the extension first (run "bun run compile")'
    );
    return;
  }

  try {
    mcpServerProcess = spawn('node', [mcpServerPath], {
      stdio: ['pipe', 'pipe', 'pipe']
    });

    mcpServerProcess.stdout?.on('data', (data) => {
      console.log(`MCP Server: ${data}`);
    });

    mcpServerProcess.stderr?.on('data', (data) => {
      console.error(`MCP Server Error: ${data}`);
    });

    mcpServerProcess.on('exit', (code) => {
      console.log(`MCP Server exited with code ${code}`);
      mcpServerProcess = null;
    });

    vscode.window.showInformationMessage(
      'TensorFleet MCP Server started! Configure it in Cursor or Claude Desktop.',
      'Show Config'
    ).then((selection) => {
      if (selection === 'Show Config') {
        showMCPConfiguration(context);
      }
    });
  } catch (error) {
    vscode.window.showErrorMessage(
      `Failed to start MCP server: ${error instanceof Error ? error.message : String(error)}`
    );
  }
}

function stopMCPServer() {
  if (!mcpServerProcess) {
    vscode.window.showInformationMessage('TensorFleet MCP Server is not running');
    return;
  }

  mcpServerProcess.kill();
  mcpServerProcess = null;
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
  } catch {
    // Config not found or parse error, use default
  }

  // Update status bar
  if (rosVersionStatusBar) {
    rosVersionStatusBar.text = `$(archive) ${currentRosVersion.name}`;
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

    // Check if ROS2 is connected and get real telemetry
    let droneStatus: 'idle' | 'armed' | 'flying' | 'offline' = 'offline';
    let battery = 0;
    let mode = 'UNKNOWN';

    if (ros2Bridge && ros2Bridge.isROS2Connected()) {
      try {
        // Try to get topic list to check if PX4 is publishing
        const topics = await ros2Bridge.getTopicList();
        const px4Topics = topics.filter(t => 
          t.name.includes('mavros') || t.name.includes('fmu')
        );
        
        if (px4Topics.length > 0) {
          droneStatus = 'idle'; // ROS2 connected with PX4 topics
          battery = 100; // Would be updated from actual telemetry
          mode = 'MANUAL';
        }
      } catch (error) {
        console.error('[TensorFleet] Error checking PX4 topics:', error);
      }
    }

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
  } catch {
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

  if (selected) {
    currentRosVersion = selected.version;
    
    if (rosVersionStatusBar) {
      rosVersionStatusBar.text = `$(archive) ${currentRosVersion.name}`;
    }

    // Optionally update the config file
    const shouldUpdateConfig = await vscode.window.showInformationMessage(
      `Switched to ${currentRosVersion.name}. Update drone_config.yaml?`,
      'Yes',
      'No'
    );

    if (shouldUpdateConfig === 'Yes') {
      await updateConfigWithRosVersion(currentRosVersion);
    }

    vscode.window.showInformationMessage(
      `ROS version set to ${currentRosVersion.name}. Run: source ${currentRosVersion.path}/setup.bash`
    );
  }
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
    vscode.window.showInformationMessage('Updated drone_config.yaml with ROS version');
  } catch (error) {
    vscode.window.showErrorMessage(
      `Failed to update config: ${error instanceof Error ? error.message : String(error)}`
    );
  }
}

async function showDroneStatus() {
  if (drones.length === 0) {
    vscode.window.showInformationMessage('No drones detected. Start a simulation to see drone status.');
    return;
  }

  const items = drones.map((drone) => {
    const statusIcon =
      drone.status === 'flying' ? '$(rocket)' :
      drone.status === 'armed' ? '$(zap)' :
      drone.status === 'idle' ? '$(circle-outline)' :
      '$(circle-slash)';

    const batteryIcon =
      drone.battery > 75 ? '$(battery-full)' :
      drone.battery > 50 ? '$(battery)' :
      drone.battery > 25 ? '$(battery-charging)' :
      '$(battery-empty)';

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

  if (selected) {
    if (selected.label.includes('Refresh')) {
      await updateDroneStatus();
      vscode.window.showInformationMessage('Drone status refreshed');
    } else if (selected.label.includes('Start Simulation')) {
      vscode.commands.executeCommand('tensorfleet.openGazeboPanel');
    } else if (selected.drone) {
      // Show detailed drone info
      showDetailedDroneInfo(selected.drone);
    }
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
      vscode.commands.executeCommand('tensorfleet.openGazeboPanel');
    }
  });
}

// ============================================================================
// ROS2 Connection Management
// ============================================================================

async function connectToROS2(context: vscode.ExtensionContext) {
  if (!ros2Bridge) {
    vscode.window.showErrorMessage('ROS2 Bridge not initialized');
    return;
  }

  try {
    await vscode.window.withProgress(
      {
        location: vscode.ProgressLocation.Notification,
        title: 'Connecting to ROS2...',
        cancellable: false
      },
      async (progress) => {
        progress.report({ message: 'Initializing ROS2 node...' });
        await ros2Bridge!.initialize();
        
        progress.report({ message: 'Discovering topics...' });
        const topics = await ros2Bridge!.getTopicList();
        
        vscode.window.showInformationMessage(
          `Connected to ROS2! Found ${topics.length} active topics.`,
          'View Topics'
        ).then(action => {
          if (action === 'View Topics') {
            showROS2Topics(topics);
          }
        });
      }
    );
  } catch (error) {
    vscode.window.showErrorMessage(
      `Failed to connect to ROS2: ${error}`,
      'View Setup Guide'
    ).then(action => {
      if (action === 'View Setup Guide') {
        const setupUri = vscode.Uri.file(path.join(context.extensionPath, 'ROS2_SETUP.md'));
        vscode.commands.executeCommand('markdown.showPreview', setupUri);
      }
    });
  }
}

async function disconnectFromROS2() {
  if (!ros2Bridge) {
    return;
  }

  try {
    await ros2Bridge.shutdown();
    vscode.window.showInformationMessage('Disconnected from ROS2');
  } catch (error) {
    vscode.window.showErrorMessage(`Error disconnecting from ROS2: ${error}`);
  }
}

// ============================================================================
// ROS2 WebSocket Connection Management
// ============================================================================

async function connectToROS2WebSocket(context: vscode.ExtensionContext) {
  if (!ros2WebSocketBridge) {
    vscode.window.showErrorMessage('ROS2 WebSocket Bridge not initialized');
    return;
  }

  const config = vscode.workspace.getConfiguration('tensorfleet');
  const wsUrl = config.get<string>('ros2.websocketUrl', 'ws://172.16.0.2:9091');

  try {
    await vscode.window.withProgress(
      {
        location: vscode.ProgressLocation.Notification,
        title: `Connecting to ROS2 WebSocket at ${wsUrl}...`,
        cancellable: false
      },
      async (progress) => {
        progress.report({ message: 'Establishing WebSocket connection...' });
        await ros2WebSocketBridge!.connect();
        
        progress.report({ message: 'Discovering topics...' });
        const topics = await ros2WebSocketBridge!.getTopicList();
        
        vscode.window.showInformationMessage(
          `Connected to ROS2 via WebSocket! Found ${topics.length} active topics.`,
          'View Topics',
          'Configure URL'
        ).then(action => {
          if (action === 'View Topics') {
            showROS2Topics(topics);
          } else if (action === 'Configure URL') {
            configureROS2WebSocket();
          }
        });
      }
    );
  } catch (error) {
    vscode.window.showErrorMessage(
      `Failed to connect to ROS2 WebSocket: ${error}`,
      'Configure URL',
      'View Setup Guide'
    ).then(action => {
      if (action === 'Configure URL') {
        configureROS2WebSocket();
      } else if (action === 'View Setup Guide') {
        const setupUri = vscode.Uri.file(path.join(context.extensionPath, 'CONNECT_TO_REMOTE_VM.md'));
        vscode.commands.executeCommand('markdown.showPreview', setupUri);
      }
    });
  }
}

async function disconnectROS2WebSocket() {
  if (!ros2WebSocketBridge) {
    return;
  }

  try {
    await ros2WebSocketBridge.shutdown();
    vscode.window.showInformationMessage('Disconnected from ROS2 WebSocket');
  } catch (error) {
    vscode.window.showErrorMessage(`Error disconnecting from ROS2 WebSocket: ${error}`);
  }
}

async function configureROS2WebSocket() {
  const config = vscode.workspace.getConfiguration('tensorfleet');
  const currentUrl = config.get<string>('ros2.websocketUrl', 'ws://172.16.0.2:9091');

  const newUrl = await vscode.window.showInputBox({
    prompt: 'Enter ROS2 WebSocket URL (rosbridge_suite)',
    value: currentUrl,
    placeHolder: 'ws://172.16.0.2:9091',
    validateInput: (value) => {
      if (!value) {
        return 'URL cannot be empty';
      }
      if (!value.startsWith('ws://') && !value.startsWith('wss://')) {
        return 'URL must start with ws:// or wss://';
      }
      return null;
    }
  });

  if (newUrl && newUrl !== currentUrl) {
    try {
      await config.update('ros2.websocketUrl', newUrl, vscode.ConfigurationTarget.Global);
      
      // Update the bridge with new URL
      if (ros2WebSocketBridge) {
        // Disconnect if currently connected
        if (ros2WebSocketBridge.isROS2Connected()) {
          await ros2WebSocketBridge.shutdown();
        }
        ros2WebSocketBridge.setUrl(newUrl);
      }
      
      vscode.window.showInformationMessage(
        `ROS2 WebSocket URL updated to: ${newUrl}`,
        'Connect Now'
      ).then(action => {
        if (action === 'Connect Now') {
          vscode.commands.executeCommand('tensorfleet.connectROS2WebSocket');
        }
      });
    } catch (error) {
      vscode.window.showErrorMessage(`Failed to update WebSocket URL: ${error}`);
    }
  }
}

async function connectToFoxgloveBridge(context: vscode.ExtensionContext) {
  if (!foxgloveBridge) {
    vscode.window.showErrorMessage('Foxglove Bridge not initialized');
    return;
  }

  const config = vscode.workspace.getConfiguration('tensorfleet');
  const foxgloveUrl = config.get<string>('ros2.foxgloveUrl', 'ws://172.16.0.2:8765');

  try {
    await vscode.window.withProgress(
      {
        location: vscode.ProgressLocation.Notification,
        title: `Connecting to Foxglove Bridge at ${foxgloveUrl}...`,
        cancellable: false
      },
      async (progress) => {
        progress.report({ message: 'Establishing connection...' });
        await foxgloveBridge!.connect();
        
        progress.report({ message: 'Discovering channels...' });
        const topics = await foxgloveBridge!.getTopicList();
        
        vscode.window.showInformationMessage(
          `Connected to Foxglove Bridge! Found ${topics.length} channels.`,
          'View Topics',
          'Configure URL'
        ).then(action => {
          if (action === 'View Topics') {
            showROS2Topics(topics);
          } else if (action === 'Configure URL') {
            vscode.commands.executeCommand('tensorfleet.configureFoxgloveBridge');
          }
        });
      }
    );
  } catch (error) {
    vscode.window.showErrorMessage(`Failed to connect to Foxglove Bridge: ${error}`);
    console.error('[TensorFleet] Foxglove Bridge connection error:', error);
  }
}

async function disconnectFoxgloveBridge() {
  if (!foxgloveBridge) {
    return;
  }

  try {
    await foxgloveBridge.shutdown();
    vscode.window.showInformationMessage('Disconnected from Foxglove Bridge');
  } catch (error) {
    vscode.window.showErrorMessage(`Error disconnecting from Foxglove Bridge: ${error}`);
  }
}

async function configureFoxgloveBridge() {
  const config = vscode.workspace.getConfiguration('tensorfleet');
  const currentUrl = config.get<string>('ros2.foxgloveUrl', 'ws://172.16.0.2:8765');

  const newUrl = await vscode.window.showInputBox({
    prompt: 'Enter Foxglove Bridge WebSocket URL',
    value: currentUrl,
    placeHolder: 'ws://172.16.0.2:8765',
    validateInput: (value) => {
      if (!value) {
        return 'URL cannot be empty';
      }
      if (!value.startsWith('ws://') && !value.startsWith('wss://')) {
        return 'URL must start with ws:// or wss://';
      }
      return null;
    }
  });

  if (newUrl && newUrl !== currentUrl) {
    try {
      await config.update('ros2.foxgloveUrl', newUrl, vscode.ConfigurationTarget.Global);
      
      // Update the bridge with new URL
      if (foxgloveBridge) {
        // Disconnect if currently connected
        if (foxgloveBridge.isROS2Connected()) {
          await foxgloveBridge.shutdown();
        }
        foxgloveBridge.setUrl(newUrl);
      }
      
      vscode.window.showInformationMessage(
        `Foxglove Bridge URL updated to: ${newUrl}`,
        'Connect Now'
      ).then(action => {
        if (action === 'Connect Now') {
          vscode.commands.executeCommand('tensorfleet.connectFoxgloveBridge');
        }
      });
    } catch (error) {
      vscode.window.showErrorMessage(`Failed to update Foxglove Bridge URL: ${error}`);
    }
  }
}

async function showROS2Topics(topics: Array<{ name: string; type: string }>) {
  const items = topics.map(topic => ({
    label: topic.name,
    description: topic.type,
    detail: `Topic: ${topic.name}`
  }));

  const selected = await vscode.window.showQuickPick(items, {
    placeHolder: 'ROS2 Active Topics',
    title: `${topics.length} Active ROS2 Topics`,
    matchOnDescription: true,
    matchOnDetail: true
  });

  if (selected) {
    const action = await vscode.window.showInformationMessage(
      `Topic: ${selected.label}\nType: ${selected.description}`,
      'Copy Topic Name',
      'Echo Topic'
    );

    if (action === 'Copy Topic Name') {
      vscode.env.clipboard.writeText(selected.label);
      vscode.window.showInformationMessage(`Copied: ${selected.label}`);
    } else if (action === 'Echo Topic') {
      // Open terminal and echo the topic
      const terminal = vscode.window.createTerminal('ROS2 Echo');
      terminal.sendText(`ros2 topic echo ${selected.label}`);
      terminal.show();
    }
  }
}

// PX4 Telemetry Monitor
let telemetryOutputChannel: vscode.OutputChannel | null = null;

async function startPX4TelemetryMonitor(context: vscode.ExtensionContext) {
  if (!ros2Bridge) {
    vscode.window.showErrorMessage('ROS2 Bridge not initialized');
    return;
  }

  try {
    // Initialize ROS2 if not connected
    if (!ros2Bridge.isROS2Connected()) {
      await ros2Bridge.initialize();
    }

    // Create output channel for telemetry
    if (!telemetryOutputChannel) {
      telemetryOutputChannel = vscode.window.createOutputChannel('PX4 Telemetry');
      context.subscriptions.push(telemetryOutputChannel);
    }
    
    telemetryOutputChannel.clear();
    telemetryOutputChannel.show();
    telemetryOutputChannel.appendLine('='.repeat(60));
    telemetryOutputChannel.appendLine('PX4 Telemetry Monitor Started');
    telemetryOutputChannel.appendLine('='.repeat(60));
    telemetryOutputChannel.appendLine('');

    // Subscribe to PX4 telemetry
    await ros2Bridge.subscribeToPX4Telemetry((telemetry: any) => {
      if (!telemetryOutputChannel) return;

      const timestamp = new Date().toLocaleTimeString();
      telemetryOutputChannel.appendLine(`[${timestamp}] Telemetry Update:`);
      
      if (telemetry.pose) {
        telemetryOutputChannel.appendLine(`  Position: x=${telemetry.pose.position.x.toFixed(2)}, y=${telemetry.pose.position.y.toFixed(2)}, z=${telemetry.pose.position.z.toFixed(2)}`);
      }
      
      if (telemetry.battery) {
        telemetryOutputChannel.appendLine(`  Battery: ${telemetry.battery.percentage.toFixed(1)}% (${telemetry.battery.voltage.toFixed(2)}V)`);
      }
      
      if (telemetry.state) {
        telemetryOutputChannel.appendLine(`  State: ${telemetry.state.mode} | Armed: ${telemetry.state.armed} | Connected: ${telemetry.state.connected}`);
      }
      
      telemetryOutputChannel.appendLine('');
    });

    vscode.window.showInformationMessage('PX4 Telemetry monitoring started');
    
  } catch (error) {
    vscode.window.showErrorMessage(`Failed to start PX4 telemetry: ${error}`);
  }
}

// ============================================================================
// MCP Configuration
// ============================================================================

async function showMCPConfiguration(context: vscode.ExtensionContext) {
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
  
  vscode.window.showInformationMessage(
    'MCP Configuration copied! Add this to your Cursor or Claude Desktop config.',
    'Open Setup Guide'
  ).then((selection) => {
    if (selection === 'Open Setup Guide') {
      const setupPath = vscode.Uri.file(path.join(context.extensionPath, 'MCP_SETUP.md'));
      vscode.commands.executeCommand('markdown.showPreview', setupPath);
    }
  });
}
