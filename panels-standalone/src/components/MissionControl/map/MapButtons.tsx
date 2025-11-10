import React from 'react';

// A lightweight, reusable map UI button stack + buttons.
// - Visually centered icon by drawing around (0,0) with a symmetric viewBox.
// - Bottom-right stack by default; can switch corners via prop.

export type Corner = 'top-left' | 'top-right' | 'bottom-left' | 'bottom-right';

export function MapButtonsStack({
                                    corner = 'bottom-right',
                                    gap = 8,
                                    children,
                                    className,
                                }: {
    corner?: Corner;
    gap?: number;
    className?: string;
    children: React.ReactNode;
}) {
    const pos: React.CSSProperties = { position: 'absolute' };
    if (corner.includes('bottom')) pos.bottom = 12;
    if (corner.includes('top')) pos.top = 12;
    if (corner.includes('right')) pos.right = 12;
    if (corner.includes('left')) pos.left = 12;

    return (
        <div
            className={className}
            style={{
                ...pos,
                display: 'flex',
                flexDirection: 'column',
                gap,
                pointerEvents: 'none', // allow map interactions except on buttons
                zIndex: 1000,
            }}
        >
            {React.Children.map(children, (child) => (
                <div style={{ pointerEvents: 'auto' }}>{child}</div>
            ))}
        </div>
    );
}

export function MapButton({
                              title,
                              pressed,
                              onClick,
                              children,
                              size = 48,
                          }: {
    title: string;
    pressed?: boolean;
    onClick?: () => void;
    children: React.ReactNode;
    size?: number;
}) {
    return (
        <button
            type="button"
            aria-pressed={pressed}
            title={title}
            onClick={onClick}
            style={{
                width: size,
                height: size,
                padding: 0,
                lineHeight: 0,
                borderRadius: 999,
                border: '1px solid rgba(0,0,0,0.15)',
                background: 'rgba(255,255,255,0.92)',
                boxShadow: '0 2px 8px rgba(0,0,0,0.15)',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                cursor: 'pointer',
                userSelect: 'none',
                backdropFilter: 'saturate(1.2) blur(2px)',
            }}
        >
            {children}
        </button>
    );
}

// Visually centered stretched navigation triangle icon.
// Drawn around (0,0) and centered with viewBox so it *really* sits in the middle.
export function NavArrowIcon({ filled = false, size = 24 }: { filled?: boolean; size?: number }) {
    return (
        <svg
            width={size}
            height={size}
            viewBox="0 0 24 24"
            role="img"
            aria-hidden="true"
            preserveAspectRatio="xMidYMid meet"
            style={{ display: 'block', overflow: 'visible' }}
        >
            <path
                d="M12 2 L20 22 L12 17 L4 22 Z"
                fill={filled ? '#111' : 'none'}
                stroke="#111"
                strokeWidth={1.6}
                strokeLinejoin="round"
                strokeLinecap="round"
                vectorEffect="non-scaling-stroke"
            />
        </svg>
    );
}

// Specialized follow/lock button using the nav arrow icon.
export function FollowLockButton({ locked, onToggle }: { locked: boolean; onToggle: () => void }) {
    return (
        <MapButton
            title={locked ? 'Unlock camera (stop following)' : 'Lock camera to drone'}
            pressed={locked}
            onClick={onToggle}
        >
            <NavArrowIcon filled={locked} />
        </MapButton>
    );
}
