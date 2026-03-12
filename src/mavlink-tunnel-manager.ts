import * as fs from 'fs';
import * as path from 'path';
import { ChildProcess, spawn } from 'child_process';

export type TunnelStatusCallback = (message: string) => void;

export interface MavlinkTunnelConnectOptions {
  projectDir: string;
  serialPath: string;
  baudRate: number;
  vmManagerUrl: string;
  token: string;
  nodeId: string;
  targetPort: number;
}

export interface MavlinkTunnelStats {
  bytes_in_serial: number;
  bytes_out_serial: number;
  bytes_in_ws: number;
  bytes_out_ws: number;
}

export class MavlinkTunnelManager {
  private process: ChildProcess | null = null;
  private connectPromise: Promise<void> | null = null;
  private serialReady = false;
  private wsReady = false;
  private stats: MavlinkTunnelStats = {
    bytes_in_serial: 0,
    bytes_out_serial: 0,
    bytes_in_ws: 0,
    bytes_out_ws: 0
  };

  constructor(private readonly onStatus?: TunnelStatusCallback) {}

  isConnected(): boolean {
    return Boolean(this.process) && this.serialReady && this.wsReady;
  }

  isRunning(): boolean {
    return Boolean(this.process);
  }

  getStats(): MavlinkTunnelStats {
    return { ...this.stats };
  }

  async connect(options: MavlinkTunnelConnectOptions): Promise<void> {
    if (this.isConnected()) {
      this.log('Tunnel already connected.');
      return;
    }
    if (this.connectPromise) {
      return this.connectPromise;
    }

    const scriptPath = path.join(options.projectDir, 'scripts', 'mavlink_tunnel_probe.js');
    if (!fs.existsSync(scriptPath)) {
      throw new Error(`Tunnel script not found: ${scriptPath}`);
    }

    this.serialReady = false;
    this.wsReady = false;
    this.stats = {
      bytes_in_serial: 0,
      bytes_out_serial: 0,
      bytes_in_ws: 0,
      bytes_out_ws: 0
    };

    this.connectPromise = new Promise<void>((resolve, reject) => {
      const args = [
        scriptPath,
        '--serial-path',
        options.serialPath,
        '--baud-rate',
        String(options.baudRate),
        '--vm-manager-url',
        options.vmManagerUrl,
        '--token',
        options.token,
        '--node-id',
        options.nodeId,
        '--target-port',
        String(options.targetPort),
        '--duration-seconds',
        '0'
      ];

      const child = spawn(process.execPath, args, {
        cwd: options.projectDir,
        env: { ...process.env },
        stdio: ['ignore', 'pipe', 'pipe']
      });

      this.process = child;
      this.log(
        `Started tunnel process serial=${options.serialPath} baud=${options.baudRate} targetPort=${options.targetPort}`
      );

      const timeout = setTimeout(() => {
        if (!this.isConnected()) {
          this.log('Tunnel readiness timeout; stopping process.');
          void this.disconnect().finally(() => {
            reject(new Error('Timed out waiting for tunnel to become ready'));
          });
        }
      }, 15000);

      const onLine = (line: string, isErr: boolean) => {
        const text = line.trim();
        if (!text) return;
        this.log(`${isErr ? '[stderr] ' : ''}${text}`);
        this.parseStatus(text);
        if (this.serialReady && this.wsReady) {
          clearTimeout(timeout);
          resolve();
        }
      };

      let stdoutBuf = '';
      let stderrBuf = '';

      child.stdout?.on('data', (chunk: Buffer) => {
        stdoutBuf += chunk.toString('utf8');
        const lines = stdoutBuf.split('\n');
        stdoutBuf = lines.pop() || '';
        lines.forEach((line) => onLine(line, false));
      });

      child.stderr?.on('data', (chunk: Buffer) => {
        stderrBuf += chunk.toString('utf8');
        const lines = stderrBuf.split('\n');
        stderrBuf = lines.pop() || '';
        lines.forEach((line) => onLine(line, true));
      });

      child.on('error', (error) => {
        clearTimeout(timeout);
        reject(error);
      });

      child.on('close', (code, signal) => {
        clearTimeout(timeout);
        const wasReady = this.serialReady && this.wsReady;
        this.process = null;
        this.serialReady = false;
        this.wsReady = false;
        if (wasReady) {
          this.log(`Tunnel process exited code=${String(code)} signal=${String(signal || '')}`);
          return;
        }
        reject(
          new Error(
            `Tunnel process exited before ready (code=${String(code)}, signal=${String(signal || '')})`
          )
        );
      });
    }).finally(() => {
      this.connectPromise = null;
    });

    return this.connectPromise;
  }

  async disconnect(): Promise<void> {
    if (!this.process) {
      return;
    }

    const child = this.process;
    this.process = null;
    this.serialReady = false;
    this.wsReady = false;

    await new Promise<void>((resolve) => {
      let closed = false;
      const done = () => {
        if (closed) return;
        closed = true;
        resolve();
      };

      child.once('close', () => done());
      try {
        child.kill('SIGTERM');
      } catch {
        done();
      }

      setTimeout(() => {
        if (closed) return;
        try {
          child.kill('SIGKILL');
        } catch {
          // ignore
        }
      }, 3000);
    });

    this.log('Tunnel process stopped.');
  }

  private parseStatus(text: string): void {
    if (text.includes('Serial connected:')) {
      this.serialReady = true;
    }
    if (text.includes('Proxy connected:')) {
      this.wsReady = true;
    }

    // Example:
    // [MAVLINK][TOTAL] serial_in=123 serial_out=45 ws_in=67 ws_out=89
    const totals = text.match(/serial_in=(\d+)\s+serial_out=(\d+)\s+ws_in=(\d+)\s+ws_out=(\d+)/);
    if (totals) {
      this.stats = {
        bytes_in_serial: Number(totals[1]),
        bytes_out_serial: Number(totals[2]),
        bytes_in_ws: Number(totals[3]),
        bytes_out_ws: Number(totals[4])
      };
    }
  }

  private log(message: string): void {
    if (this.onStatus) {
      this.onStatus(message);
    }
  }
}

