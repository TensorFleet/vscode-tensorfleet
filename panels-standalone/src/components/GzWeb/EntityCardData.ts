// ============================================================================
// Entity Card Data Types
// ============================================================================

import { EntityData, EntityCardDataImpl } from 'tensorfleet-util/ros/fetchFeaturedEntities';

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

export interface EntitySelectMessage {
  type: 'ENTITY_SELECT';
  payload: {
    entity: EntityCardData;
    timestamp: number;
  };
}

export interface EntityNudgeMessage {
  type: 'ENTITY_NUDGE';
  payload: {
    entity: EntityCardData;
    requestId?: string;
    delta: {
      x: number;
      y: number;
      z: number;
    };
    step: number;
    timestamp: number;
  };
}

export type EntityNudgeStatusState = 'pending' | 'success' | 'error';

export interface EntityNudgeStatusMessage {
  type: 'ENTITY_NUDGE_STATUS';
  payload: {
    requestId?: string;
    entity: string;
    state: EntityNudgeStatusState;
    message: string;
    attempt?: number;
    maxAttempts?: number;
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

export const ENTITY_CONTROL_MESSAGES = {
  SELECT: 'ENTITY_SELECT' as const,
  NUDGE: 'ENTITY_NUDGE' as const,
  NUDGE_STATUS: 'ENTITY_NUDGE_STATUS' as const,
};
