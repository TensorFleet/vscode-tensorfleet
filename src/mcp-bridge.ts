import * as vscode from 'vscode';
import * as net from 'net';
import * as fs from 'fs';
import * as os from 'os';
import * as path from 'path';

// Bridge between MCP server and VS Code extension
// Allows MCP tools to trigger actual VS Code commands

export class MCPBridge {
  private server: net.Server | null = null;
  private socketPath: string;

  constructor(_context: vscode.ExtensionContext) {
    this.socketPath = path.join(os.tmpdir(), 'tensorfleet-mcp-bridge.sock');
  }

  async start() {
    // Remove existing socket file if it exists
    if (fs.existsSync(this.socketPath)) {
      fs.unlinkSync(this.socketPath);
    }

    return new Promise<void>((resolve, reject) => {
      this.server = net.createServer((socket) => {
        socket.on('data', async (data) => {
          try {
            const message = JSON.parse(data.toString());
            const response = await this.handleMessage(message);
            socket.write(JSON.stringify(response) + '\n');
          } catch (error) {
            socket.write(JSON.stringify({ 
              success: false, 
              error: error instanceof Error ? error.message : String(error) 
            }) + '\n');
          }
        });
      });

      this.server.listen(this.socketPath, () => {
        console.log(`MCP Bridge listening on ${this.socketPath}`);
        resolve();
      });

      this.server.on('error', reject);
    });
  }

  async stop() {
    return new Promise<void>((resolve) => {
      if (this.server) {
        this.server.close(() => {
          if (fs.existsSync(this.socketPath)) {
            fs.unlinkSync(this.socketPath);
          }
          resolve();
        });
      } else {
        resolve();
      }
    });
  }

  private async handleMessage(message: any): Promise<any> {
    const { command, params } = message;

    switch (command) {
      case 'openGazeboPanel':
        await vscode.commands.executeCommand('tensorfleet.openGazeboPanel');
        return { success: true, message: 'Gazebo panel opened' };

      case 'openQGCPanel':
        await vscode.commands.executeCommand('tensorfleet.openQGroundControlPanel');
        return { success: true, message: 'QGroundControl panel opened' };

      case 'openAIPanel':
        await vscode.commands.executeCommand('tensorfleet.openAIPanel');
        return { success: true, message: 'AI Ops panel opened' };

      case 'openROS2Panel':
        await vscode.commands.executeCommand('tensorfleet.openROS2Panel');
        return { success: true, message: 'ROS 2 panel opened' };

      case 'openAllPanels':
        await vscode.commands.executeCommand('tensorfleet.openAllPanels');
        return { success: true, message: 'All panels opened' };

      case 'showMessage':
        vscode.window.showInformationMessage(params?.message || 'TensorFleet notification');
        return { success: true };

      case 'createTerminal':
        const terminal = vscode.window.createTerminal({
          name: params?.name || 'TensorFleet',
          cwd: params?.cwd
        });
        if (params?.command) {
          terminal.sendText(params.command);
        }
        terminal.show();
        return { success: true, message: 'Terminal created' };

      default:
        return { success: false, error: `Unknown command: ${command}` };
    }
  }

  getSocketPath(): string {
    return this.socketPath;
  }
}

