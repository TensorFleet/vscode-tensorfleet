import React, { useEffect, useMemo, useRef, useState } from 'react';
import Feature from 'ol/Feature';
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
import { MapButtonsStack } from './MapButtons';

const iconButtonBaseStyle: React.CSSProperties = {
    display: 'block',
};

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
            type="button"
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
        style={{ ...iconButtonBaseStyle, ...style }}
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
        <line x1="16.5" y1="10.75" x2="16.5" y2="13.25" stroke="#0a66ff" strokeWidth="2" strokeLinecap="round" vectorEffect="non-scaling-stroke" />
    </svg>
);

const DroneStatusIcon: React.FC<{ style?: React.CSSProperties }> = ({ style }) => (
    <span
        aria-hidden="true"
        style={{
            ...iconButtonBaseStyle,
            ...style,
            width: '100%',
            height: '100%',
            display: 'grid',
            placeItems: 'center',
            lineHeight: 1,
            fontSize: 20,
        }}
    >
        ✈️
    </span>
);

function makeFlightPathStyles(): (feature: Feature<LineString>) => Style[] {
    const lineStyle = new Style();
    const squareMarker = new Style();
    const arrowStyle = new Style();

    return (feature) => {
        const strokeColor = feature.get('isSelected') === true
            ? 'rgba(255,105,180,0.98)'
            : 'rgba(255,0,0,0.95)';

        lineStyle.setStroke(new Stroke({
            color: strokeColor,
            width: 3,
            lineCap: 'square',
            lineJoin: 'miter',
        }));

        squareMarker.setImage(new RegularShape({
            points: 4,
            radius: 5,
            angle: 0,
            fill: new Fill({ color: '#ffffff' }),
            stroke: new Stroke({ color: strokeColor, width: 2 }),
        }));
        squareMarker.setGeometry((innerFeature) => {
            const geom = innerFeature.getGeometry();
            if (geom instanceof LineString) return new MultiPoint(geom.getCoordinates());
            return undefined;
        });
        squareMarker.setZIndex(10);

        arrowStyle.setImage(new RegularShape({
            points: 3,
            radius: 8,
            rotation: 0,
            fill: new Fill({ color: strokeColor }),
            stroke: new Stroke({ color: '#ffffff', width: 1 }),
        }));
        arrowStyle.setGeometry((innerFeature) => {
            const geom = innerFeature.getGeometry();
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
        });
        arrowStyle.setZIndex(11);

        return [lineStyle, squareMarker, arrowStyle];
    };
}

type FlightPlanRecord = {
    id: string;
    name: string;
    path: [number, number][];
};

function syncFlightPlanFeatures(
    source: VectorSource,
    flightPlans: FlightPlanRecord[],
    selectedFlightPlanId: string | null,
) {
    source.clear();

    for (const flightPlan of flightPlans) {
        if (flightPlan.id !== selectedFlightPlanId) continue;
        if (flightPlan.path.length < 2) continue;

        const feature = new Feature({
            geometry: new LineString(flightPlan.path),
        });
        feature.setId(flightPlan.id);
        feature.set('isSelected', flightPlan.id === selectedFlightPlanId);
        source.addFeature(feature);
    }
}

type FlightPathToolsProps = {
    map: Map | null | undefined;
    onPathChange?: (coords: [number, number][]) => void;
    startRequestKey?: number;
    flightPlans?: FlightPlanRecord[];
    selectedFlightPlanId?: string | null;
    activePanel?: 'mission-planning' | 'drone-status';
    onSelectPanel?: (panel: 'mission-planning' | 'drone-status') => void;
};

export const FlightPathTools: React.FC<FlightPathToolsProps> = ({
    map,
    onPathChange,
    startRequestKey = 0,
    flightPlans = [],
    selectedFlightPlanId = null,
    activePanel = 'mission-planning',
    onSelectPanel,
}) => {
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

    useEffect(() => {
        if (!sourceRef.current) return;
        syncFlightPlanFeatures(sourceRef.current, flightPlans, selectedFlightPlanId);
    }, [flightPlans, selectedFlightPlanId]);

    const startNewPath = () => {
        if (!map || !sourceRef.current) return;

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

    useEffect(() => {
        if (startRequestKey > 0) {
            startNewPath();
        }
    }, [startRequestKey]);

    return (
        <MapButtonsStack corner="top-right">
            <IconButton
                title="Open mission planning"
                active={activePanel === 'mission-planning'}
                onClick={() => onSelectPanel?.('mission-planning')}
                size={44}
                iconScale={0.9}
                icon={<SVGFlagPlusIcon />}
                iconNudge={{ x: 0, y: 0 }}
            />
            <IconButton
                title="Open drone status"
                active={activePanel === 'drone-status'}
                onClick={() => onSelectPanel?.('drone-status')}
                size={44}
                iconScale={0.72}
                icon={<DroneStatusIcon />}
                iconNudge={{ x: 0, y: -1 }}
            />
        </MapButtonsStack>
    );
};
