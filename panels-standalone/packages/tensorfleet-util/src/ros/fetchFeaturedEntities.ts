/**
 * Utility functions for ROS bridge operations.
 */

import type { ROS2BridgeApi } from "./ros-bridge-api";

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
      return modelNames.map(name => String(name));
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
 *    - `proxy_target`: the display name of the entity
 *    - `params`: JSON string containing entity parameters (including `type`)
 * 
 * 2. Direct featured nodes (when the node itself has the flag):
 *    - `featured`: boolean flag on the node
 *    - name/target: the node name itself (strips leading `/`)
 *    - `params`: optional JSON string containing entity parameters
 * 
 * @param bridge - ROS2BridgeApi instance to fetch parameters from
 * @returns Promise<FeaturedEntityData[]> Array of featured entity data
 */
export async function fetchFeaturedEntities(bridge: ROS2BridgeApi): Promise<FeaturedEntityData[]> {
  const allParams = await bridge.getAllROSParameters();
  
  const featured: FeaturedEntityData[] = [];
  
  // Check for proxy featured nodes first
  for (const [key, value] of Object.entries(allParams)) {
    if (key.endsWith('.proxy_featured') && value === true) {
      const nodeName = key.replace('.proxy_featured', '');
      const target = allParams[`${nodeName}.proxy_target`] as string;
      const paramsStr = allParams[`${nodeName}.params`] as string;
      
      if (target && paramsStr) {
        try {
          const params = JSON.parse(paramsStr);
          featured.push(new EntityCardDataImpl(
            target,
            params.type || 'unknown',
            target,
            params
          ));
        } catch (parseError) {
          console.warn(`Failed to parse params for ${nodeName}:`, parseError);
        }
      }
    }
  }
  
  // Check for direct featured nodes
  for (const [key, value] of Object.entries(allParams)) {
    if (key.endsWith('.featured') && value === true) {
      const nodeName = key.replace('.featured', '');
      // Use node name (strip leading /) as both name and target
      const displayName = nodeName.startsWith('/') ? nodeName.slice(1) : nodeName;
      const paramsStr = allParams[`${nodeName}.params`] as string;
      
      if (paramsStr) {
        try {
          const params = JSON.parse(paramsStr);
          featured.push(new EntityCardDataImpl(
            displayName,
            params.type || 'unknown',
            displayName,
            params
          ));
        } catch (parseError) {
          console.warn(`Failed to parse params for ${nodeName}:`, parseError);
        }
      } else {
        // No params, just add with basic info
        featured.push(new EntityCardDataImpl(
          displayName,
          'unknown',
          displayName,
          {}
        ));
      }
    }
  }
  
  return featured;
}