import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';

type PanelKind = 'standard' | 'terminalTabs';

type DroneViewport = {
  id: string;
  title: string;
  description: string;
  image: string;
  command: string;
  actionLabel: string;
  panelKind?: PanelKind;
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
    image: 'qgroundcontrol-placeholder.svg',
    command: 'tensorfleet.openQGroundControlPanel',
    actionLabel: 'Launch QGroundControl Workspace'
  },
  {
    id: 'tensorfleet-gazebo',
    title: 'Gazebo Simulation',
    description: 'Review Gazebo scenes, sensor overlays, and simulation states for the current drone world.',
    image: 'gazebo-placeholder.svg',
    command: 'tensorfleet.openGazeboPanel',
    actionLabel: 'Open Gazebo Viewer'
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

export function activate(context: vscode.ExtensionContext) {
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
    vscode.commands.registerCommand('tensorfleet.openAllPanels', () => openAllPanels(context))
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
  // No resources to dispose.
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
      if (message?.command === 'installTools') {
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
  const panel = vscode.window.createWebviewPanel(
    viewType,
    view.title,
    { viewColumn, preserveFocus },
    {
      enableScripts: true,
      retainContextWhenHidden: true,
      localResourceRoots: [vscode.Uri.joinPath(context.extensionUri, 'media')]
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
    }
  });

  if (view.panelKind === 'terminalTabs') {
    panel.webview.html = getTerminalPanelHtml(view, imageUri, cspSource);
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
