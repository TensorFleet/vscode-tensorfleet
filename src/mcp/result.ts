export const MCP_RESULT_STATUSES = [
  "success",
  "unsupported",
  "unavailable",
  "invalid_request",
  "invalid_state",
  "not_authenticated",
  "runtime_offline",
  "source_unreachable",
  "stale_source",
  "backend_error",
  "failed",
] as const;

export type TensorFleetMcpStatus = (typeof MCP_RESULT_STATUSES)[number];

export type TensorFleetMcpResult<T = unknown> = {
  ok: boolean;
  status: TensorFleetMcpStatus;
  message: string;
  data: T | null;
  reason?: string;
};

export type McpToolResponse = {
  content: Array<{ type: "text"; text: string }>;
};

export function mcpSuccess<T>(message: string, data: T): TensorFleetMcpResult<T> {
  return {
    ok: true,
    status: "success",
    message,
    data,
  };
}

export function mcpFailure<T = null>(
  status: Exclude<TensorFleetMcpStatus, "success">,
  reason: string,
  message: string,
  data: T | null = null,
): TensorFleetMcpResult<T> {
  return {
    ok: false,
    status,
    reason,
    message,
    data,
  };
}

export function asToolResponse(result: TensorFleetMcpResult): McpToolResponse {
  return {
    content: [
      {
        type: "text",
        text: JSON.stringify(result, null, 2),
      },
    ],
  };
}

