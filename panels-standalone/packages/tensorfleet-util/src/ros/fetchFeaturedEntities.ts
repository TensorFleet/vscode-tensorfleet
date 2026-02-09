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
 * Featured entities are identified by nodes with `proxy_featured=true` parameter.
 * Each featured entity has:
 * - `proxy_featured`: boolean flag to mark the entity
 * - `proxy_target`: the display name of the entity
 * - `params`: JSON string containing entity parameters (including `type`)
 * 
 * @param bridge - ROS2BridgeApi instance to fetch parameters from
 * @returns Promise<FeaturedEntityData[]> Array of featured entity data
 */
export async function fetchFeaturedEntities(bridge: ROS2BridgeApi): Promise<FeaturedEntityData[]> {
  const allParams = await bridge.getAllROSParameters();
  
  const featured: FeaturedEntityData[] = [];
  
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
  
  return featured;
}
