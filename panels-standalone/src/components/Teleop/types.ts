// Ported from: lichtblick/packages/suite-base/src/panels/Teleop/types.ts
// Date: 2025-11-06
// Modifications: Removed framework-specific imports and types

export type TeleopButtonKey =
  | 'upButton'
  | 'downButton'
  | 'leftButton'
  | 'rightButton'
  | 'secondaryUpButton'
  | 'secondaryDownButton'
  | 'secondaryLeftButton'
  | 'secondaryRightButton';

export type TeleopButtonBinding = { field: string; value: number };

export type TeleopConfig = {
  topic: undefined | string;
  publishRate: number;
  upButton: TeleopButtonBinding;
  downButton: TeleopButtonBinding;
  leftButton: TeleopButtonBinding;
  rightButton: TeleopButtonBinding;
  secondaryUpButton: TeleopButtonBinding;
  secondaryDownButton: TeleopButtonBinding;
  secondaryLeftButton: TeleopButtonBinding;
  secondaryRightButton: TeleopButtonBinding;
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
  activeAction?: DirectionalPadAction;
};
