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
  }, [entity.name, infoShield]);

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
        // Get model names from the entity
        const modelNames = entity.getModelNames();
        
        // Send focus message to parent window
        const message = {
          type: 'FOCUS_ON_MODELS',
          payload: {
            modelNames: modelNames,
            entityName: entity.name,
            timestamp: nowMs(),
          },
        };

        window.parent.postMessage(message, '*');
        
        // Call the original onCardClick if it exists
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
          className="info-button"
          debugName={`EntityCardInfo:${entity.name}`}
          blockClicks={false}
          shield={infoShield}
          onStableClick={handleInfoClickStable}
          aria-label={`View info for ${entity.name}`}
          title="View detailed information"
        >
          <span className="codicon-info"></span>
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