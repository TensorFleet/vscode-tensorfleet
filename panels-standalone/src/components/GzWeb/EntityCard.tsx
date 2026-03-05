import React, { useCallback, useEffect, useMemo, useRef } from 'react';
import { EntityCardData, CARD_MESSAGES } from './EntityCardData';
import { ENTITY_INFO_POPUP_MESSAGES } from './EntityInfoPopup';

interface EntityCardProps {
  entity: EntityCardData;
  isActive: boolean;
}

type ReactAnyEvent =
  | React.SyntheticEvent
  | React.MouseEvent
  | React.PointerEvent
  | React.TouchEvent
  | React.KeyboardEvent;

function stopAll(e: ReactAnyEvent) {
  try {
    (e as any).preventDefault?.();
  } catch {}
  try {
    (e as any).stopPropagation?.();
  } catch {}
  const ne = (e as any).nativeEvent as any;
  try {
    ne?.stopImmediatePropagation?.();
  } catch {}
}

function safeTag(node: any) {
  const el = node as HTMLElement | null;
  if (!el) return undefined;
  return el.tagName;
}

function safeClass(node: any) {
  const el = node as HTMLElement | null;
  if (!el) return undefined;
  return el.className;
}

function nowMs() {
  return Date.now();
}

type ClickBlockReason = 'prop_disabled' | 'cooldown' | 'reentry_guard';

interface ClickShieldHandle {
  markIntent: (e: ReactAnyEvent) => void;
  shouldBlock: () => { block: boolean; reason?: ClickBlockReason; ageMs?: number };
  clearSoon: () => void;
  debugDump: () => any;
}

function useClickShield(opts: {
  name: string;
  cooldownMs?: number;
  logPrefix?: string;
  enabled?: boolean;
}): ClickShieldHandle {
  const { name, cooldownMs = 850, logPrefix = 'ClickShield', enabled = true } = opts;

  const pressedRef = useRef(false);
  const pressTsRef = useRef(0);
  const seqRef = useRef(0);
  const lastTypeRef = useRef<string>('');
  const lastTargetRef = useRef<string>('');

  const markIntent = useCallback(
    (e: ReactAnyEvent) => {
      if (!enabled) return;

      pressedRef.current = true;
      pressTsRef.current = nowMs();
      seqRef.current += 1;

      const t = (e as any).target as HTMLElement | null;
      lastTypeRef.current = (e as any).type || '';
      lastTargetRef.current = `${safeTag(t) ?? 'NA'}:${safeClass(t) ?? 'NA'}`;

      stopAll(e);

      console.log(`[${logPrefix}][MARK]`, {
        name,
        seq: seqRef.current,
        type: (e as any).type,
        time: pressTsRef.current,
        targetTag: safeTag(t),
        targetClass: safeClass(t),
      });
    },
    [enabled, logPrefix, name]
  );

  const shouldBlock = useCallback(() => {
    if (!enabled) return { block: false };

    const ts = pressTsRef.current;
    const age = ts ? nowMs() - ts : Number.POSITIVE_INFINITY;
    const pressed = pressedRef.current;

    if (pressed && age < cooldownMs) {
      return { block: true, reason: 'cooldown', ageMs: age };
    }
    return { block: false };
  }, [enabled, cooldownMs]);

  const clearSoon = useCallback(() => {
    if (!enabled) return;

    const seqAtSchedule = seqRef.current;
    window.setTimeout(() => {
      const before = pressedRef.current;
      pressedRef.current = false;
      const after = pressedRef.current;

      console.log(`[${logPrefix}][CLEAR]`, {
        name,
        seq: seqAtSchedule,
        before,
        after,
        lastType: lastTypeRef.current,
        lastTarget: lastTargetRef.current,
        time: nowMs(),
      });
    }, 0);
  }, [enabled, logPrefix, name]);

  const debugDump = useCallback(() => {
    return {
      name,
      enabled,
      pressed: pressedRef.current,
      pressTs: pressTsRef.current,
      ageMs: pressTsRef.current ? nowMs() - pressTsRef.current : null,
      seq: seqRef.current,
      lastType: lastTypeRef.current,
      lastTarget: lastTargetRef.current,
      cooldownMs,
    };
  }, [name, enabled, cooldownMs]);

  return { markIntent, shouldBlock, clearSoon, debugDump };
}

interface StableClickButtonProps
  extends Omit<React.ButtonHTMLAttributes<HTMLButtonElement>, 'onClick' | 'disabled'> {
  debugName: string;
  blockClicks?: boolean;
  onStableClick: (e: React.MouseEvent<HTMLButtonElement>) => void;
  shield: ClickShieldHandle;
}

const StableClickButton: React.FC<StableClickButtonProps> = ({
  debugName,
  blockClicks = false,
  onStableClick,
  shield,
  ...rest
}) => {
  const inClickRef = useRef(false);

  const logBase = useMemo(() => `StableClickButton:${debugName}`, [debugName]);

  const mark = useCallback(
    (e: ReactAnyEvent) => {
      shield.markIntent(e);
    },
    [shield]
  );

  const handleClick = useCallback(
    (e: React.MouseEvent<HTMLButtonElement>) => {
      const t = e.target as HTMLElement | null;
      const ct = e.currentTarget as HTMLElement | null;

      if (blockClicks) {
        stopAll(e);
        console.log(`[${logBase}][CLICK_BLOCKED]`, {
          reason: 'prop_disabled' as ClickBlockReason,
          time: nowMs(),
          targetTag: safeTag(t),
          targetClass: safeClass(t),
          currentTargetTag: safeTag(ct),
          currentTargetClass: safeClass(ct),
          shield: shield.debugDump(),
        });
        shield.clearSoon();
        return;
      }

      if (inClickRef.current) {
        stopAll(e);
        console.log(`[${logBase}][CLICK_BLOCKED]`, {
          reason: 'reentry_guard' as ClickBlockReason,
          time: nowMs(),
          targetTag: safeTag(t),
          targetClass: safeClass(t),
          currentTargetTag: safeTag(ct),
          currentTargetClass: safeClass(ct),
          shield: shield.debugDump(),
        });
        shield.clearSoon();
        return;
      }

      inClickRef.current = true;

      stopAll(e);

      console.log(`[${logBase}][CLICK_OK]`, {
        time: nowMs(),
        targetTag: safeTag(t),
        targetClass: safeClass(t),
        currentTargetTag: safeTag(ct),
        currentTargetClass: safeClass(ct),
        shield: shield.debugDump(),
      });

      try {
        onStableClick(e);
      } catch (err) {
        console.error(`[${logBase}][CLICK_ERROR]`, { err, shield: shield.debugDump() });
        throw err;
      } finally {
        inClickRef.current = false;
        shield.clearSoon();
      }
    },
    [blockClicks, logBase, onStableClick, shield]
  );

  return (
    <button
      {...rest}
      aria-disabled={blockClicks ? true : rest['aria-disabled']}
      onPointerDown={mark as any}
      onMouseDown={mark as any}
      onTouchStart={mark as any}
      onClick={handleClick}
    />
  );
};

export const EntityCard: React.FC<EntityCardProps> = ({
  entity,
  isActive,
}) => {
  const getEntityIcon = (type: string) => {
    switch (type.toLowerCase()) {
      case 'drone':
        return '🚁';
      case 'robot':
        return '🤖';
      default:
        return '📦';
    }
  };

  const infoShield = useClickShield({
    name: `EntityCard:${entity.name}:info`,
    cooldownMs: 900,
    logPrefix: 'EntityCardInfoShield',
    enabled: true,
  });

  useEffect(() => {
    return () => {
      console.log('[EntityCard][UNMOUNT]', { entity: entity.name, infoShield: infoShield.debugDump() });
    };
  }, []);

  const handleInfoClickStable = useCallback(
    (e: React.MouseEvent<HTMLButtonElement>) => {
      const t = e.target as HTMLElement | null;
      const ct = e.currentTarget as HTMLElement | null;

      console.log('[EntityCard][INFO_CLICK]', {
        entity: entity.name,
        type: e.type,
        time: nowMs(),
        targetTag: safeTag(t),
        targetClass: safeClass(t),
        currentTargetTag: safeTag(ct),
        currentTargetClass: safeClass(ct),
        shield: infoShield.debugDump(),
      });

      try {
        entity.onInfoClick?.();
      } catch (err) {
        console.error('[EntityCard][INFO_ONINFOCLICK_ERROR]', { entity: entity.name, err });
        throw err;
      }
    },
    [entity, infoShield]
  );

  const handleCardClick = useCallback(
    (e: React.MouseEvent<HTMLDivElement>) => {
      const t = e.target as HTMLElement | null;

      const block = infoShield.shouldBlock();

      console.log('[EntityCard][CARD_CLICK]', {
        entity: entity.name,
        time: nowMs(),
        defaultPrevented: e.defaultPrevented,
        targetClass: safeClass(t),
        shield: infoShield.debugDump(),
        block,
      });

      if (block.block) {
        console.warn('[EntityCard][CARD_CLICK_BLOCKED]', {
          entity: entity.name,
          reason: block.reason,
          ageMs: block.ageMs,
        });
        return;
      }

      if (e.defaultPrevented) {
        console.warn('[EntityCard][CARD_CLICK_BLOCKED]', {
          entity: entity.name,
          reason: 'event_defaultPrevented',
        });
        return;
      }

      try {
        entity.onCardClick?.();
      } catch (err) {
        console.error('[EntityCard][CARD_ONCLICK_ERROR]', { entity: entity.name, err });
        throw err;
      }
    },
    [entity, infoShield]
  );

  const handleMouseEnter = useCallback(() => {
    const message = {
      type: CARD_MESSAGES.HOVER_START,
      payload: {
        entityName: entity.name,
        modelNames: entity.getModelNames(),
        timestamp: nowMs(),
      },
    };

    try {
      window.parent.postMessage(message, '*');
    } catch (err) {
      console.error('[EntityCard][HOVER_START_POSTMESSAGE_ERROR]', { entity: entity.name, err });
    }
  }, [entity]);

  const handleMouseLeave = useCallback(() => {
    const message = {
      type: CARD_MESSAGES.HOVER_END,
      payload: {
        entityName: entity.name,
        modelNames: entity.getModelNames(),
        timestamp: nowMs(),
      },
    };

    try {
      window.parent.postMessage(message, '*');
    } catch (err) {
      console.error('[EntityCard][HOVER_END_POSTMESSAGE_ERROR]', { entity: entity.name, err });
    }
  }, [entity]);

  const handleFocusEntity = useCallback(() => {
    if (entity) {
      const modelNames = entity.getModelNames();

      const message = {
        type: 'FOCUS_ON_MODELS',
        payload: {
          modelNames: modelNames,
          entityName: entity.name,
          timestamp: nowMs(),
        },
      };

      window.parent.postMessage(message, '*');
    }
  }, [entity]);

  return (
    <div
      className={`entity-card ${isActive ? 'active' : ''}`}
      onClick={handleCardClick}
      onMouseEnter={handleMouseEnter}
      onMouseLeave={handleMouseLeave}
      role="button"
      tabIndex={0}
      onKeyDown={(e) => {
        const target = e.target as HTMLElement | null;
        if (target?.closest('.info-button')) return;
        if (e.key === 'Enter' || e.key === ' ') {
          e.preventDefault();
          try {
            entity.onCardClick?.();
          } catch (err) {
            console.error('[EntityCard][KEYDOWN_ONCLICK_ERROR]', { entity: entity.name, err });
            throw err;
          }
        }
      }}
    >
      <div className="entity-header">
        <span className="entity-icon">{getEntityIcon(entity.type)}</span>
        <span className="entity-name">{entity.name}</span>
        <span className="entity-status status-active">Active</span>

        <StableClickButton
          type="button"
          className="fentity-sub-button"
          debugName={`EntityCardInfo:${entity.name}`}
          blockClicks={false}
          shield={infoShield}
          onStableClick={handleInfoClickStable}
          aria-label={`View info for ${entity.name}`}
          title="View detailed information"
        >
          <span className="codicon-info"></span>
        </StableClickButton>
        <StableClickButton
          type="button"
          className="fentity-sub-button"
          debugName={`EntityCardFocus:${entity.name}`}
          blockClicks={false}
          shield={infoShield}
          onStableClick={handleFocusEntity}
          aria-label={`Focus on ${entity.name}`}
          title="Focus on entity"
        >
          <svg width="16" height="16" viewBox="0 0 16 16" xmlns="http://www.w3.org/2000/svg" fill="currentColor"><path d="M11 8C11 9.65685 9.65685 11 8 11C6.34315 11 5 9.65685 5 8C5 6.34315 6.34315 5 8 5C9.65685 5 11 6.34315 11 8ZM10 8C10 6.89543 9.10457 6 8 6C6.89543 6 6 6.89543 6 8C6 9.10457 6.89543 10 8 10C9.10457 10 10 9.10457 10 8ZM6.61803 2C6.04988 2 5.53048 2.321 5.27639 2.82918L4.69098 4H4C2.89543 4 2 4.89543 2 6V11C2 12.1046 2.89543 13 4 13H12C13.1046 13 14 12.1046 14 11V6C14 4.89543 13.1046 4 12 4H11.309L10.7236 2.82918C10.4695 2.321 9.95012 2 9.38197 2H6.61803ZM6.17082 3.27639C6.25552 3.107 6.42865 3 6.61803 3H9.38197C9.57135 3 9.74448 3.107 9.82918 3.27639L10.5528 4.72361C10.6375 4.893 10.8106 5 11 5H12C12.5523 5 13 5.44772 13 6V11C13 11.5523 12.5523 12 12 12H4C3.44772 12 3 11.5523 3 11V6C3 5.44772 3.44772 5 4 5H5C5.18939 5 5.36252 4.893 5.44721 4.72361L6.17082 3.27639Z"/></svg>
        </StableClickButton>
      </div>

      <div className="entity-details">
        <div className="entity-metric">
          <span className="metric-label">Type:</span>
          <span className="metric-value">{entity.type}</span>
        </div>
        <div className="entity-metric">
          <span className="metric-label">Target:</span>
          <span className="metric-value">{entity.target}</span>
        </div>
      </div>
    </div>
  );
};

export default EntityCard;
