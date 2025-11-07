// Ported from: lichtblick/packages/suite-base/src/panels/Teleop/DirectionalPad.tsx
// Date: 2025-11-06
// Modifications: Removed framework dependencies (Stack component, useStyles hook)
// Converted CSS-in-JS to traditional CSS, replaced cx() with manual class building

import React, { useCallback, useState } from 'react';
import { svgPathsDisabled, svgPathsEnabled } from './constants';
import { DirectionalPadAction, DirectionalPadProps } from './types';
import './DirectionalPad.css';

export function DirectionalPad(props: Readonly<DirectionalPadProps>): React.JSX.Element {
  const { onAction, disabled = false, activeAction } = props;

  const [currentAction, setCurrentAction] = useState<DirectionalPadAction | undefined>();

  const handleMouseDown = useCallback(
    (action: DirectionalPadAction) => {
      setCurrentAction(action);
      onAction?.(action);
    },
    [onAction],
  );

  const handleMouseUp = useCallback(() => {
    if (currentAction === undefined) {
      return;
    }
    setCurrentAction(undefined);
    onAction?.();
  }, [onAction, currentAction]);

  const makeMouseHandlers = (action: DirectionalPadAction) =>
    disabled
      ? {}
      : {
          onMouseDown: () => {
            handleMouseDown(action);
          },
          onMouseUp: () => {
            handleMouseUp();
          },
          onMouseLeave: () => {
            handleMouseUp();
          },
        };

  // Helper to build CSS class names (replaces cx() from tss-react)
  const getButtonClass = (action: DirectionalPadAction) => {
    const classes = ['directional-pad-button'];
    if (currentAction === action || activeAction === action) {
      classes.push('active');
    }
    if (disabled) {
      classes.push('disabled');
    }
    return classes.join(' ');
  };

  const getIconClass = () => {
    return disabled ? 'directional-pad-button-icon disabled' : 'directional-pad-button-icon';
  };

  return (
    <div className="directional-pad-container">
      <svg className="directional-pad-svg" viewBox="0 0 256 256">
        <g opacity={1}>
          {/* UP button */}
          <g {...makeMouseHandlers(DirectionalPadAction.UP)} role="button">
            <path
              className={getButtonClass(DirectionalPadAction.UP)}
              d={svgPathsEnabled.up}
            />
            <path className={getIconClass()} d={svgPathsDisabled.up} />
          </g>

          {/* DOWN button */}
          <g {...makeMouseHandlers(DirectionalPadAction.DOWN)} role="button">
            <path
              className={getButtonClass(DirectionalPadAction.DOWN)}
              d={svgPathsEnabled.down}
            />
            <path className={getIconClass()} d={svgPathsDisabled.down} />
          </g>
        </g>

        <g opacity={1}>
          {/* LEFT button */}
          <g {...makeMouseHandlers(DirectionalPadAction.LEFT)} role="button">
            <path
              className={getButtonClass(DirectionalPadAction.LEFT)}
              d={svgPathsEnabled.left}
            />
            <path className={getIconClass()} d={svgPathsDisabled.left} />
          </g>

          {/* RIGHT button */}
          <g {...makeMouseHandlers(DirectionalPadAction.RIGHT)} role="button">
            <path
              className={getButtonClass(DirectionalPadAction.RIGHT)}
              d={svgPathsEnabled.right}
            />
            <path className={getIconClass()} d={svgPathsDisabled.right} />
          </g>
        </g>
      </svg>
    </div>
  );
}
