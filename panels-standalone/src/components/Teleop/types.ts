// Ported from: lichtblick/packages/suite-base/src/panels/Teleop/types.ts
// Date: 2025-11-06
// Modifications: Removed framework-specific imports and types

export type TeleopConfig = {
  topic: undefined | string;
  publishRate: number;
  upButton: { field: string; value: number };
  downButton: { field: string; value: number };
  leftButton: { field: string; value: number };
  rightButton: { field: string; value: number };
};

export enum DirectionalPadAction {
  UP,
  DOWN,
  LEFT,
  RIGHT,
}

export type DirectionalPadProps = {
  disabled?: boolean;
  onAction?: (action?: DirectionalPadAction) => void;
};

