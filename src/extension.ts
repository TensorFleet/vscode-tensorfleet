import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';
import { spawn, ChildProcess } from 'child_process';
import { MCPBridge } from './mcp-bridge';
import { VMManagerIntegration } from './vm-manager';

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
let vmManagerIntegration: VMManagerIntegration | null = null;


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

  // Panels handle ROS2 networking internally via panels-standalone Foxglove client.

  // Initialize status bar items for TensorFleet projects
  initializeStatusBarItems(context).catch((error) => {
    console.error('[TensorFleet] Failed to initialize status bars:', error);
  });

  vmManagerIntegration = new VMManagerIntegration(context);
  vmManagerIntegration.initialize();
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
    vscode.commands.registerCommand('tensorfleet.createNewRoboticProject', () => createNewRoboticProject(context))
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

  if (vmManagerIntegration) {
    context.subscriptions.push(
      vscode.commands.registerCommand('tensorfleet.showVMManagerMenu', () => vmManagerIntegration?.showVmActions())
    );
  }

  // ROS bridge commands removed; panels use embedded Foxglove networking.

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

  if (vmManagerIntegration) {
    vmManagerIntegration.dispose();
    vmManagerIntegration = null;
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
      } else if (message?.command === 'newRoboticProject') {
        vscode.commands.executeCommand('tensorfleet.createNewRoboticProject');
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

    if (view.htmlTemplate == 'raw-messages-standalone') {
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

  if (view.htmlTemplate === 'raw-messages-standalone') {
    return getStandalonePanelHtml('raw_messages', webview, context, cspSource);
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
  panelName: 'teleops' | 'image' | 'mission_control' | 'raw_messages',
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

async function handleOption3Message(_panel: vscode.WebviewPanel, message: any, _context: vscode.ExtensionContext) {
  if (!message || !message.command) {
    console.warn('[TensorFleet] Invalid message received from webview');
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
  await createNewProjectInternal(context, {
    kindLabel: 'drone',
    defaultName: 'my-drone-project',
    commandLabel: 'TensorFleet Project'
  });
}

async function createNewRoboticProject(context: vscode.ExtensionContext) {
  await createNewProjectInternal(context, {
    kindLabel: 'robotic',
    defaultName: 'my-robotic-project',
    commandLabel: 'TensorFleet Robotic Project',
    templateSubdir: 'robotic-project-templates'
  });
}

type NewProjectOptions = {
  kindLabel: string;
  defaultName: string;
  commandLabel: string;
  templateSubdir?: string;
};

async function createNewProjectInternal(
  context: vscode.ExtensionContext,
  options: NewProjectOptions
) {
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
  const templateFolder = vscode.Uri.joinPath(
    context.extensionUri,
    'resources',
    options.templateSubdir ?? 'project-templates'
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
        progress.report({ message: 'Project created successfully!' });
      }
    );

    const openProject = await vscode.window.showInformationMessage(
      `✨ ${options.commandLabel} "${projectName}" created successfully!`,
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
