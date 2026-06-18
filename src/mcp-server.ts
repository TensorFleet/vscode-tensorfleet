#!/usr/bin/env node

import { Server } from "@modelcontextprotocol/sdk/server/index.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import {
  CallToolRequestSchema,
  ListResourcesRequestSchema,
  ListToolsRequestSchema,
  ReadResourceRequestSchema,
} from "@modelcontextprotocol/sdk/types.js";
import { createVacuumTools } from "./mcp/vacuum-tools";
import { asToolResponse, mcpFailure, type McpToolResponse } from "./mcp/result";

class TensorFleetMCPServer {
  private readonly server: Server;
  private readonly tools = createVacuumTools();

  constructor() {
    this.server = new Server(
      {
        name: "tensorfleet",
        version: "0.1.0",
      },
      {
        capabilities: {
          tools: {},
          resources: {},
        },
      },
    );

    this.setupRequestHandlers();
  }

  private setupRequestHandlers() {
    this.server.setRequestHandler(ListToolsRequestSchema, async () => ({
      tools: [...this.tools.values()].map(({ execute: _execute, ...tool }) => tool),
    }));

    this.server.setRequestHandler(CallToolRequestSchema, async (request): Promise<McpToolResponse> => {
      const toolName = request.params.name;
      const args = request.params.arguments ?? {};
      const tool = this.tools.get(toolName);

      if (!tool) {
        return asToolResponse(
          mcpFailure("unsupported", "unknown_tool", `Unknown TensorFleet MCP tool: ${toolName}.`),
        );
      }

      try {
        return asToolResponse(await tool.execute(args as Record<string, unknown>));
      } catch (error) {
        return asToolResponse(
          mcpFailure(
            "failed",
            "tool_execution_failed",
            error instanceof Error ? error.message : String(error),
          ),
        );
      }
    });

    this.server.setRequestHandler(ListResourcesRequestSchema, async () => ({
      resources: [],
    }));

    this.server.setRequestHandler(ReadResourceRequestSchema, async () => ({
      contents: [],
    }));
  }

  async run() {
    const transport = new StdioServerTransport();
    await this.server.connect(transport);
    console.error("TensorFleet MCP Server running on stdio");
  }
}

const server = new TensorFleetMCPServer();
server.run().catch((error) => {
  console.error(error);
  process.exit(1);
});
