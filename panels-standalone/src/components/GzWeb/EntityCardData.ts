// ============================================================================
// Entity Card Data Types
// ============================================================================

export interface EntityCardData {
  name: string;
  type: string;
  target: string;
  params: Record<string, unknown>;
  getModelNames(): string[];
}

export class EntityCardDataImpl implements EntityCardData {
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
}

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
