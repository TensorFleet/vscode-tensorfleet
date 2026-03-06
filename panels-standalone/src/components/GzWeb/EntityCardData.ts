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

export type EntityNudgeStatusState = 'pending' | 'success' | 'error' | 'warning';

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

export interface EntityUndoMessage {
  type: 'ENTITY_UNDO_LAST_MOVE';
  payload: {
    entity?: EntityCardData;
    timestamp: number;
  };
}

export interface EntityResetPoseMessage {
  type: 'ENTITY_RESET_ENTITY_POSE';
  payload: {
    entity?: EntityCardData;
    timestamp: number;
  };
}

export interface EntityResetAllPosesMessage {
  type: 'ENTITY_RESET_ALL_POSES';
  payload: {
    entities?: EntityCardData[];
    timestamp: number;
  };
}

export interface ScenePresetSaveMessage {
  type: 'ENTITY_SCENE_PRESET_SAVE';
  payload: {
    name: string;
    timestamp: number;
  };
}

export interface ScenePresetLoadMessage {
  type: 'ENTITY_SCENE_PRESET_LOAD';
  payload: {
    name: string;
    timestamp: number;
  };
}

export interface ScenePresetListRequestMessage {
  type: 'ENTITY_SCENE_PRESET_LIST_REQUEST';
  payload: {
    timestamp: number;
  };
}

export interface ScenePresetListMessage {
  type: 'ENTITY_SCENE_PRESET_LIST';
  payload: {
    names: string[];
    timestamp: number;
  };
}

export interface SceneSetupTraceConfigMessage {
  type: 'ENTITY_SCENE_SETUP_TRACE_CONFIG';
  payload: {
    enabled: boolean;
    filter?: string;
    timestamp: number;
  };
}

export interface EntitySelectedPoseMessage {
  type: 'ENTITY_SELECTED_POSE';
  payload: {
    entity: string;
    poseName: string;
    world?: string;
    position: {
      x: number;
      y: number;
      z: number;
    };
    orientation: {
      x: number;
      y: number;
      z: number;
      w: number;
    };
    eulerDeg: {
      roll: number;
      pitch: number;
      yaw: number;
    };
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
  UNDO_LAST_MOVE: 'ENTITY_UNDO_LAST_MOVE' as const,
  RESET_ENTITY_POSE: 'ENTITY_RESET_ENTITY_POSE' as const,
  RESET_ALL_POSES: 'ENTITY_RESET_ALL_POSES' as const,
  SCENE_PRESET_SAVE: 'ENTITY_SCENE_PRESET_SAVE' as const,
  SCENE_PRESET_LOAD: 'ENTITY_SCENE_PRESET_LOAD' as const,
  SCENE_PRESET_LIST_REQUEST: 'ENTITY_SCENE_PRESET_LIST_REQUEST' as const,
  SCENE_PRESET_LIST: 'ENTITY_SCENE_PRESET_LIST' as const,
  SCENE_SETUP_TRACE_CONFIG: 'ENTITY_SCENE_SETUP_TRACE_CONFIG' as const,
  SELECTED_POSE: 'ENTITY_SELECTED_POSE' as const,
};
