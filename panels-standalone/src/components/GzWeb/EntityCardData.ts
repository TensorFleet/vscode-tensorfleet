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

// Message type constants for card actions
export const CARD_MESSAGES = {
  CLICK: 'ENTITY_CLICK' as const,
  INFO_CLICK: 'ENTITY_INFO_CLICK' as const,
};
