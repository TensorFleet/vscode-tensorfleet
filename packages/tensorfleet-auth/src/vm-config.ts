// VM Configuration types
export interface VMConfig {
  id: string;
  name: string;
  description: string;
  sim_config: {
    config_version: string;
    world_components?: string;
    [key: string]: any;
  };
}

export interface VMConfigOption {
  id: string;
  label: string;
  description: string;
  config: VMConfig;
}

export const VM_CONFIGS: Record<string, VMConfig> = {
  'px4': {
    id: 'px4',
    name: 'PX4 Autopilot',
    description: 'PX4 flight stack with Gazebo simulation',
    sim_config: {
      config_version: "0.0.1",
      world_components: "static_obstacles_01;static_ground",
      gazebo_px4_enabled: "true"
    }
  },
  'ardupilot': {
    id: 'ardupilot',
    name: 'ArduPilot',
    description: 'ArduPilot flight stack with Gazebo simulation',
    sim_config: {
      config_version: "0.0.1",
      world_components: "static_obstacles_01;static_ground",
      gazebo_ardupilot_enabled: "true"
    }
  },
  'simple_robot': {
    id: 'simple_robot',
    name: 'Simple Robot',
    description: 'Basic ground robot with Gazebo simulation',
    sim_config: {
      config_version: "0.0.1",
      world_components: "static_obstacles_01;static_ground",
      simple_robot_enabled: "true"
    }
  },
  'lerobot': {
    id: 'lerobot',
    name: 'Lerobot arm',
    description: 'Basic robotics arm simulation',
    sim_config: {
      config_version: "0.0.1",
      world_components: "lerobot/lerobot_world_01;static_ground",
      gazebo_lerobot_enabled: "true"
    }
  },
  'turtlebot4': {
    id: 'turtlebot4',
    name: 'TurtleBot4',
    description: 'TurtleBot4 Nav2 simulation',
    sim_config: {
      config_version: "0.0.1",
      world_components: "static_obstacles_01;static_ground",
      gazebo_turtlebot4_enabled: "true"
    }
  },
};

// Default config ID
export const DEFAULT_CONFIG_ID = 'simple_robot';

// Template to config ID mapping
export const TEMPLATE_TO_CONFIG_ID: Record<string, string> = {
  'drone-js': 'px4',
  'robotic-js': 'simple_robot',
  'robotic': 'simple_robot',
  'lerobot-arm': 'lerobot'
};

/**
 * Detect VM configuration from workspace .tensorfleet file
 * @param workspaceFolders - Array of workspace folders to check
 * @returns Promise resolving to detected config or null if not found
 */
export async function detectConfigFromWorkspace(
  workspaceFolders: readonly { uri: { fsPath: string } }[],
  readFileContent: (filePath: string) => Promise<string | null>
): Promise<{ configId: string; template: string } | null> {
  if (!workspaceFolders || workspaceFolders.length === 0) {
    return null;
  }

  for (const folder of workspaceFolders) {
    const markerPath = `${folder.uri.fsPath}/.tensorfleet`;
    try {
      const content = await readFileContent(markerPath);
      if (!content) {
        continue;
      }

      const metadata = JSON.parse(content) as { template?: string };
      const template = typeof metadata.template === 'string' ? metadata.template : '';
      const configId = TEMPLATE_TO_CONFIG_ID[template];
      if (configId) {
        return { configId, template };
      }
    } catch (error) {
      // File not found or parse error, continue to next folder
      continue;
    }
  }

  return null;
}

/**
 * Get the default VM configuration
 */
export function getDefaultConfig(): VMConfig {
  const config = VM_CONFIGS[DEFAULT_CONFIG_ID];
  if (!config) {
    // Fallback to first available config if default is missing
    const firstConfigId = Object.keys(VM_CONFIGS)[0];
    const fallbackConfig = VM_CONFIGS[firstConfigId];
    if (fallbackConfig) {
      return fallbackConfig;
    }
    throw new Error('No VM configurations available');
  }
  return config;
}

/**
 * Get a specific VM configuration by ID
 * @param configId - The configuration ID
 * @returns The VM configuration or undefined if not found
 */
export function getConfigById(configId: string): VMConfig | undefined {
  return VM_CONFIGS[configId];
}

/**
 * Get all available VM configurations as options
 */
export function getAllConfigOptions(): VMConfigOption[] {
  return Object.entries(VM_CONFIGS).map(([id, config]) => ({
    id,
    label: config.name,
    description: config.description,
    config
  }));
}

/**
 * Check if a configuration ID is valid
 */
export function isValidConfigId(configId?: string): configId is string {
  return Boolean(configId && VM_CONFIGS[configId]);
}

export function isValidVmConfigId(configId?: string): configId is string {
  return Boolean(configId && VM_CONFIGS[configId]);
}

/**
 * Get the configuration ID for a given template
 */
export function getConfigIdForTemplate(template: string): string | undefined {
  return TEMPLATE_TO_CONFIG_ID[template];
}

export function getVmConfig(configId?: string): VMConfig {
  const selected = configId && VM_CONFIGS[configId] ? VM_CONFIGS[configId] : VM_CONFIGS[DEFAULT_CONFIG_ID];
  if (selected) {
    return selected;
  }
  const firstConfig = Object.values(VM_CONFIGS)[0];
  if (!firstConfig) {
    throw new Error("No VM configurations available");
  }
  return firstConfig;
}

export function getVmConfigIdForTemplate(template?: string): string | undefined {
  return template ? TEMPLATE_TO_CONFIG_ID[template] : undefined;
}
