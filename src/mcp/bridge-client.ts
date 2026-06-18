import * as net from "net";
import * as os from "os";
import * as path from "path";

export type BridgeResponse<T = unknown> =
  | ({ success: true } & T)
  | {
      success: false;
      error: string;
    };

export async function sendBridgeCommand<T = Record<string, unknown>>(
  command: string,
  params?: Record<string, unknown>,
): Promise<BridgeResponse<T>> {
  const socketPath = path.join(os.tmpdir(), "tensorfleet-mcp-bridge.sock");

  return new Promise((resolve) => {
    const client = net.createConnection(socketPath, () => {
      client.write(JSON.stringify({ command, params }));
    });

    client.on("data", (data) => {
      try {
        const response = JSON.parse(data.toString()) as BridgeResponse<T>;
        client.end();
        resolve(response);
      } catch (error) {
        client.end();
        resolve({
          success: false,
          error: error instanceof Error ? error.message : String(error),
        });
      }
    });

    client.on("error", () => {
      resolve({ success: false, error: "VS Code extension bridge not available" });
    });

    client.setTimeout(1000, () => {
      client.end();
      resolve({ success: false, error: "Bridge connection timeout" });
    });
  });
}

