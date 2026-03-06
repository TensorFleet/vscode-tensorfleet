/**
 * Utility functions for ROS bridge operations.
 */

import type { ROS2BridgeApi } from "./ros-bridge-api";

const trimIfNonEmpty = (value: unknown): string | undefined => {
  if (typeof value !== "string") return undefined;
  const trimmed = value.trim();
  return trimmed.length > 0 ? trimmed : undefined;
};

const toEntityParams = (value: unknown): Record<string, unknown> | null => {
  if (typeof value !== "string") return null;
  try {
    const parsed = JSON.parse(value);
    if (!parsed || typeof parsed !== "object" || Array.isArray(parsed)) {
      return null;
    }
    return parsed as Record<string, unknown>;
  } catch {
    return null;
  }
};

const getModelNames = (params: Record<string, unknown>): string[] => {
  const modelNames = params.model_names;
  if (!Array.isArray(modelNames)) return [];
  return modelNames
    .map((name) => String(name).trim())
    .filter((name) => name.length > 0);
};

const resolveDisplayName = (params: Record<string, unknown>, fallbacks: Array<unknown>): string => {
  return (
    trimIfNonEmpty(params.display_name) ??
    trimIfNonEmpty(params.preferred_name) ??
    fallbacks.map(trimIfNonEmpty).find(Boolean) ??
    "unknown_entity"
  );
};

const resolvePrimaryTarget = (params: Record<string, unknown>, fallback: unknown): string => {
  const canonicalGazeboEntity = trimIfNonEmpty(params.gazebo_entity);
  if (canonicalGazeboEntity) {
    return canonicalGazeboEntity;
  }
  const modelNames = getModelNames(params);
  return modelNames[0] ?? trimIfNonEmpty(fallback) ?? "unknown_entity";
};

/**
 * Data type representing an entity card with predefined callbacks
 */
export interface EntityData {
  name: string;
  type: string;
  target: string;
  params: Record<string, unknown>;
  getModelNames(): string[];
  onCardClick(): void;
  onInfoClick(): void;
}

/**
 * Data type representing a featured entity card.
 */
export interface FeaturedEntityData extends EntityData {}

/**
 * EntityCardData implementation with built-in callback methods
 */
export class EntityCardDataImpl implements EntityData {
  constructor(
    public name: string,
    public type: string,
    public target: string,
    public params: Record<string, unknown>
  ) {}

  getModelNames(): string[] {
    const modelNames = this.params.model_names;
    if (Array.isArray(modelNames)) {
      return modelNames
        .map((name) => String(name).trim())
        .filter((name) => name.length > 0);
    }

    return [];
  }

  onCardClick(): void {
    const message = {
      type: 'ENTITY_CLICK',
      payload: {
        entity: this,
        timestamp: Date.now(),
      },
    };
    
    // Send message to parent window for external module communication
    window.parent.postMessage(message, '*');
    
    // Log for debugging
    console.log(`EntityCardDataImpl: Card clicked - ${this.name}`, message);
  }

  onInfoClick(): void {
    const popupData = {
      ...this,
      timestamp: Date.now(),
    };
    
    // Send message to parent window to open the popup
    const message = {
      type: 'ENTITY_INFO_POPUP_OPEN',
      payload: popupData,
    };
    
    window.parent.postMessage(message, '*');
    
    // Also send card info click message
    const clickMessage = {
      type: 'ENTITY_INFO_CLICK',
      payload: {
        entity: this,
        timestamp: Date.now(),
      },
    };
    
    window.parent.postMessage(clickMessage, '*');
    console.log(`EntityCardDataImpl: Info button clicked - ${this.name}`, message);
  }
}

/**
 * Fetch featured entities from ROS parameters.
 *
 * Featured entities can be identified in two ways:
 *
 * 1. Proxy featured nodes (when we can't modify the node's implementation):
 *    - `proxy_featured`: boolean flag to mark the entity
 *    - `proxy_target`: fallback display name of the entity
 *    - `params`: JSON string containing entity parameters
 *      - preferred fields: `display_name`, `type`, `model_names`
 *
 * 2. Direct featured nodes (when the node itself has the flag):
 *    - `featured`: boolean flag on the node
 *    - node name is used as fallback label when `display_name` is absent
 *    - `params`: optional JSON string containing entity parameters
 *
 * @param bridge - ROS2BridgeApi instance to fetch parameters from
 * @returns Promise<FeaturedEntityData[]> Array of featured entity data
 */
export async function fetchFeaturedEntities(bridge: ROS2BridgeApi): Promise<FeaturedEntityData[]> {
  const allParams = await bridge.getAllROSParameters();
  const featuredFlagKeys = Object.entries(allParams)
    .filter(([key, value]) => (key.endsWith('.proxy_featured') || key.endsWith('.featured')) && value === true)
    .map(([key]) => key)
    .sort();
  console.log('[fetchFeaturedEntities] Featured flags from current ROS parameter snapshot', featuredFlagKeys);
  
  const featured: FeaturedEntityData[] = [];
  
  // Check for proxy featured nodes first
  for (const [key, value] of Object.entries(allParams)) {
    if (key.endsWith('.proxy_featured') && value === true) {
      const nodeName = key.replace('.proxy_featured', '');
      const fallbackLabel = allParams[`${nodeName}.proxy_target`];
      const paramsStr = allParams[`${nodeName}.params`];
      const params = toEntityParams(paramsStr);
      if (!params) {
        console.warn(`Failed to parse params for ${nodeName}: expected JSON object string`);
        continue;
      }

      const nodeLabel = nodeName.startsWith('/') ? nodeName.slice(1) : nodeName;
      const name = resolveDisplayName(params, [fallbackLabel, nodeLabel]);
      const target = resolvePrimaryTarget(params, nodeLabel);
      const type = trimIfNonEmpty(params.type) ?? 'unknown';
      featured.push(new EntityCardDataImpl(name, type, target, params));
    }
  }
  
  // Check for direct featured nodes
  for (const [key, value] of Object.entries(allParams)) {
    if (key.endsWith('.featured') && value === true) {
      const nodeName = key.replace('.featured', '');
      const fallbackName = nodeName.startsWith('/') ? nodeName.slice(1) : nodeName;
      const paramsStr = allParams[`${nodeName}.params`];
      const params = toEntityParams(paramsStr);

      if (params) {
        const name = resolveDisplayName(params, [fallbackName]);
        const target = resolvePrimaryTarget(params, fallbackName);
        const type = trimIfNonEmpty(params.type) ?? 'unknown';
        featured.push(new EntityCardDataImpl(name, type, target, params));
        continue;
      }

      if (paramsStr) {
        console.warn(`Failed to parse params for ${nodeName}: expected JSON object string`);
      }

      // No params, just add with basic info
      featured.push(new EntityCardDataImpl(
        fallbackName,
        'unknown',
        fallbackName,
        {}
      ));
    }
  }
  
  return featured;
}
