/**
 * Utility functions for ROS bridge operations.
 */

import type { ROS2BridgeApi } from "./ros-bridge-api";

/**
 * Data type representing a featured entity card.
 */
export interface FeaturedEntityData {
  name: string;
  type: string;
  target: string;
  params: Record<string, unknown>;
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
          featured.push({
            name: target,
            type: params.type || 'unknown',
            target,
            params,
          });
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
          featured.push({
            name: displayName,
            type: params.type || 'unknown',
            target: displayName,
            params,
          });
        } catch (parseError) {
          console.warn(`Failed to parse params for ${nodeName}:`, parseError);
        }
      } else {
        // No params, just add with basic info
        featured.push({
          name: displayName,
          type: 'unknown',
          target: displayName,
          params: {},
        });
      }
    }
  }
  
  return featured;
}
