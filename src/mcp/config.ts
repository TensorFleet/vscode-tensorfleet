import { sendBridgeCommand } from "./bridge-client";

export type McpRuntimeConfig = {
  vmManagerUrl?: string;
  token?: string;
  tokenAvailable: boolean;
  selectedRegion?: string;
  selectedBackend?: string;
  vmState?: string;
  source: "env" | "bridge" | "env+bridge" | "missing";
};

export type RuntimeConfigBridgePayload = {
  vmManagerUrl?: string;
  token?: string;
  tokenAvailable?: boolean;
  selectedRegion?: string;
  selectedBackend?: string;
  vmState?: string;
};

function trimmed(value: string | undefined): string | undefined {
  const next = value?.trim();
  return next ? next.replace(/\/+$/, "") : undefined;
}

function trimmedBackend(value: string | undefined): string | undefined {
  const next = value?.trim();
  return next || undefined;
}

export async function resolveMcpRuntimeConfig(
  env: NodeJS.ProcessEnv = process.env,
  bridgeResolver: () => Promise<RuntimeConfigBridgePayload | null> = getBridgeRuntimeConfig,
): Promise<McpRuntimeConfig> {
  const envVmManagerUrl = trimmed(env.TENSORFLEET_VM_MANAGER_URL);
  const envToken = trimmed(env.TENSORFLEET_JWT);
  const envSelectedBackend = trimmedBackend(env.TENSORFLEET_VACUUM_BACKEND);

  if (envVmManagerUrl && envToken && envSelectedBackend) {
    return {
      vmManagerUrl: envVmManagerUrl,
      token: envToken,
      tokenAvailable: true,
      selectedBackend: envSelectedBackend,
      source: "env",
    };
  }

  const bridgeConfig = await bridgeResolver();
  const bridgeVmManagerUrl = trimmed(bridgeConfig?.vmManagerUrl);
  const bridgeToken = trimmed(bridgeConfig?.token);
  const vmManagerUrl = envVmManagerUrl ?? bridgeVmManagerUrl;
  const token = envToken ?? bridgeToken;
  const source = envVmManagerUrl || envToken
    ? bridgeConfig
      ? "env+bridge"
      : "env"
    : bridgeConfig
      ? "bridge"
      : "missing";

  return {
    vmManagerUrl,
    token,
    tokenAvailable: Boolean(token) || bridgeConfig?.tokenAvailable === true,
    selectedRegion: bridgeConfig?.selectedRegion,
    selectedBackend: envSelectedBackend ?? bridgeConfig?.selectedBackend,
    vmState: bridgeConfig?.vmState,
    source,
  };
}

async function getBridgeRuntimeConfig(): Promise<RuntimeConfigBridgePayload | null> {
  const response = await sendBridgeCommand<RuntimeConfigBridgePayload>("getRuntimeConfig");
  if (!response.success) {
    return null;
  }
  const { success: _success, ...payload } = response;
  return payload;
}
