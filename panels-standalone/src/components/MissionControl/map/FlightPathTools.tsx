import React, { useEffect, useMemo, useRef, useState } from 'react';
import type Map from 'ol/Map';
import VectorSource from 'ol/source/Vector';
import VectorLayer from 'ol/layer/Vector';
import Draw from 'ol/interaction/Draw';
import DoubleClickZoom from 'ol/interaction/DoubleClickZoom';
import LineString from 'ol/geom/LineString';
import MultiPoint from 'ol/geom/MultiPoint';
import Point from 'ol/geom/Point';
import Style from 'ol/style/Style';
import Stroke from 'ol/style/Stroke';
import Fill from 'ol/style/Fill';
import RegularShape from 'ol/style/RegularShape';

type TopRightButtonsStackProps = { children: React.ReactNode };

export const TopRightButtonsStack: React.FC<TopRightButtonsStackProps> = ({ children }) => (
    <div
        style={{
            position: 'absolute',
            top: 12,
            right: 12,
            display: 'flex',
            flexDirection: 'column',
            gap: 8,
            zIndex: 1000,
            pointerEvents: 'none',
        }}
    >
        {children}
    </div>
);

type IconButtonProps = {
    title?: string;
    active?: boolean;
    onClick?: () => void;
    icon: React.ReactNode;
    size?: number;
    iconScale?: number;
    iconNudge?: { x?: number; y?: number };
};

const IconButton: React.FC<IconButtonProps> = ({
                                                   title,
                                                   active,
                                                   onClick,
                                                   icon,
                                                   size = 44,
                                                   iconScale = 0.8,
                                                   iconNudge,
                                               }) => {
    const iconPx = Math.round(size * iconScale);
    const iconEl = React.isValidElement(icon)
        ? React.cloneElement(icon as React.ReactElement<any>, {
            style: { ...(icon as any).props?.style, width: iconPx, height: iconPx, display: 'block' },
            preserveAspectRatio: 'xMidYMid meet',
        })
        : icon;

    return (
        <button
            title={title}
            onClick={onClick}
            style={{
                pointerEvents: 'auto',
                width: size,
                height: size,
                display: 'grid',
                placeItems: 'center',
                borderRadius: Math.round(size * 0.27),
                background: 'rgba(255,255,255,0.96)',
                boxShadow: '0 2px 10px rgba(0,0,0,0.15)',
                border: active ? '2px solid #ff3b30' : '1px solid rgba(0,0,0,0.12)',
                backdropFilter: 'blur(6px)',
                WebkitBackdropFilter: 'blur(6px)',
                cursor: 'pointer',
                padding: 0,
                lineHeight: 0,
            }}
        >
      <span
          style={{
              display: 'block',
              transform: `translate(${iconNudge?.x ?? 0}px, ${iconNudge?.y ?? 0}px)`,
          }}
      >
        {iconEl}
      </span>
        </button>
    );
};

const SVGFlagPlusIcon: React.FC<{ style?: React.CSSProperties }> = ({ style }) => (
    <svg
        xmlns="http://www.w3.org/2000/svg"
        viewBox="0 0 24 24"
        style={style}
        preserveAspectRatio="xMidYMid meet"
        aria-hidden="true"
        focusable="false"
    >
        <rect
            x="4" y="4" width="16" height="16" rx="3" ry="3"
            fill="none" stroke="rgba(0,0,0,0.6)" strokeWidth="1.5" strokeDasharray="3 2"
            vectorEffect="non-scaling-stroke"
        />
        <line
            x1="7" y1="6.5" x2="7" y2="17.5"
            stroke="#333" strokeWidth="1.8" strokeLinecap="round"
            vectorEffect="non-scaling-stroke"
        />
        <path
            d="M7 7.2 C9.7 7.2, 9.7 6, 12.6 6 v4.4 c-2.9 0-2.9 1.2-5.6 1.2z"
            fill="#ff3b30" stroke="#c21f19" strokeWidth="1"
            vectorEffect="non-scaling-stroke"
        />
        <line x1="15.25" y1="12" x2="17.75" y2="12" stroke="#0a66ff" strokeWidth="2" strokeLinecap="round" vectorEffect="non-scaling-stroke" />
        <line x1="16.5"  y1="10.75" x2="16.5"  y2="13.25" stroke="#0a66ff" strokeWidth="2" strokeLinecap="round" vectorEffect="non-scaling-stroke" />
    </svg>
);

function makeFlightPathStyles(): (feature: any) => Style[] {
    const redStroke = new Stroke({
        color: 'rgba(255,0,0,0.95)',
        width: 2,
        lineCap: 'square',
        lineJoin: 'miter',
    });

    const lineStyle = new Style({ stroke: redStroke });

    const squareMarker = new Style({
        image: new RegularShape({
            points: 4,
            radius: 5,
            angle: 0,
            fill: new Fill({ color: '#ffffff' }),
            stroke: new Stroke({ color: 'rgba(255,0,0,0.95)', width: 2 }),
        }),
        geometry: (feature) => {
            const geom = feature.getGeometry();
            if (geom instanceof LineString) return new MultiPoint(geom.getCoordinates());
            return undefined;
        },
        zIndex: 10,
    });

    const arrowStyle = new Style({
        image: new RegularShape({
            points: 3,
            radius: 8,
            rotation: 0,
            fill: new Fill({ color: 'rgba(255,0,0,0.95)' }),
            stroke: new Stroke({ color: '#ffffff', width: 1 }),
        }),
        geometry: (feature) => {
            const geom = feature.getGeometry();
            if (!(geom instanceof LineString)) return undefined;
            const coords = geom.getCoordinates();
            if (coords.length < 2) return undefined;
            const end = coords[coords.length - 1];
            const prev = coords[coords.length - 2];
            const dx = end[0] - prev[0];
            const dy = end[1] - prev[1];
            const rotation = Math.atan2(dy, dx);
            (arrowStyle.getImage() as RegularShape).setRotation(rotation);
            return new Point(end);
        },
        zIndex: 11,
    });

    return () => [lineStyle, squareMarker, arrowStyle];
}

type FlightPathToolsProps = {
    map: Map | null | undefined;
    onPathChange?: (coords: [number, number][]) => void;
};

export const FlightPathTools: React.FC<FlightPathToolsProps> = ({ map, onPathChange }) => {
    const [isDrawing, setIsDrawing] = useState(false);

    const sourceRef = useRef<VectorSource | null>(null);
    const layerRef = useRef<VectorLayer<VectorSource> | null>(null);
    const drawRef = useRef<Draw | null>(null);
    const dblZoomRef = useRef<DoubleClickZoom | null>(null);

    const styleFn = useMemo(() => makeFlightPathStyles(), []);

    useEffect(() => {
        if (!map) return;
        const source = new VectorSource();
        const layer = new VectorLayer({ source, zIndex: 9, style: styleFn });
        map.addLayer(layer);
        sourceRef.current = source;
        layerRef.current = layer;

        return () => {
            if (drawRef.current) {
                map.removeInteraction(drawRef.current);
                drawRef.current = null;
            }
            map.removeLayer(layer);
            sourceRef.current = null;
            layerRef.current = null;
        };
    }, [map, styleFn]);

    const startNewPath = () => {
        if (!map || !sourceRef.current) return;

        sourceRef.current.clear();

        if (drawRef.current) {
            map.removeInteraction(drawRef.current);
            drawRef.current = null;
        }

        const dbl = map.getInteractions().getArray().find(i => i instanceof DoubleClickZoom) as DoubleClickZoom | undefined;
        if (dbl) {
            dblZoomRef.current = dbl;
            dbl.setActive(false);
        }

        const draw = new Draw({
            source: sourceRef.current,
            type: 'LineString',
            style: styleFn,
            stopClick: true,
        });

        draw.on('drawend', (evt) => {
            setIsDrawing(false);
            if (drawRef.current) map.removeInteraction(drawRef.current);
            drawRef.current = null;

            dblZoomRef.current?.setActive(true);

            const geom = evt.feature.getGeometry();
            if (geom instanceof LineString) {
                const coords = geom.getCoordinates() as [number, number][];
                onPathChange?.(coords);
            }
        });

        map.addInteraction(draw);
        drawRef.current = draw;
        setIsDrawing(true);
    };

    return (
        <TopRightButtonsStack>
            <IconButton
                title={isDrawing ? 'Drawing flight path… (double-click to finish)' : 'Start flight path'}
                active={isDrawing}
                onClick={startNewPath}
                size={44}
                iconScale={0.9}
                icon={<SVGFlagPlusIcon />}
                iconNudge={{ x: 0, y: 0 }}
            />
        </TopRightButtonsStack>
    );
};
