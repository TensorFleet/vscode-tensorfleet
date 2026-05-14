// ============================================================================
// Entity Card Data Types
// ============================================================================

import { EntityData, EntityCardDataImpl } from 'tensorfleet-util/ros/fetch-featured-entities';

export interface EntityCardData extends EntityData {}

// Re-export the implementation for convenience
export { EntityCardDataImpl };

export interface EntityClickMessage {
  type: 'ENTITY_CLICK' | 'ENTITY_INFO_CLICK';
  payload: {
    entity: EntityCardData;
    timestamp: number;
  };
}

export interface EntityHoverMessage {
  type: 'ENTITY_HOVER_START' | 'ENTITY_HOVER_END';
  payload: {
    entityName: string;
    timestamp: number;
  };
}

// Message type constants for card actions
export const CARD_MESSAGES = {
  CLICK: 'ENTITY_CLICK' as const,
  INFO_CLICK: 'ENTITY_INFO_CLICK' as const,
  HOVER_START: 'ENTITY_HOVER_START' as const,
  HOVER_END: 'ENTITY_HOVER_END' as const,
};
