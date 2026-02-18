// ============================================================================
// Entity Card Data Types
// ============================================================================

export interface EntityCardData {
  name: string;
  type: string;
  target: string;
  params: Record<string, unknown>;
}

export interface EntityClickMessage {
  type: 'ENTITY_CLICK' | 'ENTITY_INFO_CLICK';
  payload: {
    entity: EntityCardData;
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
};

export const ENTITY_CONTROL_MESSAGES = {
  SELECT: 'ENTITY_SELECT' as const,
  NUDGE: 'ENTITY_NUDGE' as const,
  NUDGE_STATUS: 'ENTITY_NUDGE_STATUS' as const,
};
