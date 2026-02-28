import React, { useEffect, useMemo, useRef, useState } from 'react';
import 'ol/ol.css';
import Map from 'ol/Map';
import View from 'ol/View';
import TileLayer from 'ol/layer/Tile';
import VectorLayer from 'ol/layer/Vector';
import OSM from 'ol/source/OSM';
import VectorSource from 'ol/source/Vector';
import Feature from 'ol/Feature';
import Point from 'ol/geom/Point';
import LineString from 'ol/geom/LineString';
import { defaults as defaultInteractions } from 'ol/interaction/defaults';
import Style from 'ol/style/Style';
import CircleStyle from 'ol/style/Circle';
import Fill from 'ol/style/Fill';
import Stroke from 'ol/style/Stroke';
import { fromLonLat, toLonLat } from 'ol/proj';
import type { EventsKey } from 'ol/events';
import { unByKey } from 'ol/Observable';
import { useFlightPlans } from './FlightPlanContext';

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
      x="4"
      y="4"
      width="16"
      height="16"
      rx="3"
      ry="3"
      fill="none"
      stroke="rgba(0,0,0,0.6)"
      strokeWidth="1.5"
      strokeDasharray="3 2"
      vectorEffect="non-scaling-stroke"
    />
    <line
      x1="7"
      y1="6.5"
      x2="7"
      y2="17.5"
      stroke="#333"
      strokeWidth="1.8"
      strokeLinecap="round"
      vectorEffect="non-scaling-stroke"
    />
    <path
      d="M7 7.2 C9.7 7.2, 9.7 6, 12.6 6 v4.4 c-2.9 0-2.9 1.2-5.6 1.2z"
      fill="#ff3b30"
      stroke="#c21f19"
      strokeWidth="1"
      vectorEffect="non-scaling-stroke"
    />
    <line
      x1="15.25"
      y1="12"
      x2="17.75"
      y2="12"
      stroke="#0a66ff"
      strokeWidth="2"
      strokeLinecap="round"
      vectorEffect="non-scaling-stroke"
    />
    <line
      x1="16.5"
      y1="10.75"
      x2="16.5"
      y2="13.25"
      stroke="#0a66ff"
      strokeWidth="2"
      strokeLinecap="round"
      vectorEffect="non-scaling-stroke"
    />
  </svg>
);

const SVGTrashIcon: React.FC<{ style?: React.CSSProperties }> = ({ style }) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    viewBox="0 0 24 24"
    style={style}
    preserveAspectRatio="xMidYMid meet"
    aria-hidden="true"
    focusable="false"
  >
    <path
      d="M6 7h12v13a2 2 0 0 1-2 2H8a2 2 0 0 1-2-2V7zm2 0v13h8V7H8zm2-2h4v-1a2 2 0 0 0-2-2 2 2 0 0 0-2 2v1z"
      fill="#666"
      stroke="#333"
      strokeWidth="1"
      vectorEffect="non-scaling-stroke"
    />
    <line
      x1="8"
      y1="9"
      x2="16"
      y2="9"
      stroke="#333"
      strokeWidth="2"
      strokeLinecap="round"
      vectorEffect="non-scaling-stroke"
    />
    <line
      x1="8"
      y1="12"
      x2="16"
      y2="12"
      stroke="#333"
      strokeWidth="2"
      strokeLinecap="round"
      vectorEffect="non-scaling-stroke"
    />
    <line
      x1="8"
      y1="15"
      x2="16"
      y2="15"
      stroke="#333"
      strokeWidth="2"
      strokeLinecap="round"
      vectorEffect="non-scaling-stroke"
    />
  </svg>
);

export type FlightPlanLonLat = { lon: number; lat: number };
export type FlightPlan = { points: FlightPlanLonLat[]; closed: boolean };

type Props = {
  className?: string;
  initialCenterLonLat?: FlightPlanLonLat;
  initialZoom?: number;
  value?: FlightPlan;
  onChange?: (plan: FlightPlan) => void;
};

type Mode = 'idle' | 'drawing';

const waypointStyle = new Style({
  image: new CircleStyle({
    radius: 4,
    fill: new Fill({ color: 'rgba(255,255,255,1)' }),
    stroke: new Stroke({ color: 'rgba(255,59,48,1)', width: 2 }),
  }),
  zIndex: 3,
});

const pathStyle = new Style({
  stroke: new Stroke({ color: 'rgba(255,59,48,0.95)', width: 3 }),
  zIndex: 2,
});

const previewStyle = new Style({
  stroke: new Stroke({ color: 'rgba(255,59,48,0.45)', width: 2, lineDash: [6, 6] }),
  zIndex: 1,
});

function clampZoom(z: number): number {
  return Math.max(1, Math.min(21, z));
}

function toFlightPlan(source: VectorSource): FlightPlan {
  const features = source.getFeatures();
  const pts = features
    .filter((f) => f.get('kind') === 'waypoint')
    .sort((a, b) => (a.get('idx') ?? 0) - (b.get('idx') ?? 0))
    .map((f) => {
      const g = f.getGeometry() as Point;
      const c = g.getCoordinates();
      const [lon, lat] = toLonLat(c);
      return { lon, lat };
    });
  return { points: pts, closed: false };
}

function renderPlanToSource(source: VectorSource, plan: FlightPlan) {
  source.clear();

  const coords = plan.points.map((p) => fromLonLat([p.lon, p.lat]) as [number, number]);

  coords.forEach((c, i) => {
    const f = new Feature<Point>(new Point(c));
    f.set('kind', 'waypoint');
    f.set('idx', i);
    f.setStyle(waypointStyle);
    source.addFeature(f);
  });

  if (coords.length >= 2) {
    const line = new Feature<LineString>(new LineString(coords));
    line.set('kind', 'plan');
    line.setStyle(pathStyle);
    source.addFeature(line);
  }
}

export const FlightPlannerMap: React.FC<Props> = ({
  className,
  initialCenterLonLat = { lon: 0, lat: 0 },
  initialZoom = 2,
  value,
  onChange,
}) => {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const mapTargetRef = useRef<HTMLDivElement | null>(null);

  const mapRef = useRef<Map | null>(null);
  const sourceRef = useRef<VectorSource | null>(null);

  const planLineRef = useRef<Feature<LineString> | null>(null);
  const previewLineRef = useRef<Feature<LineString> | null>(null);

  const modeRef = useRef<Mode>('idle');
  const [mode, setMode] = useState<Mode>('idle');

  const drawingPtsRef = useRef<[number, number][]>([]);
  const pointerCoordRef = useRef<[number, number] | null>(null);

  const isPanningRef = useRef<boolean>(false);
  const downPixelRef = useRef<[number, number] | null>(null);
  const movedRef = useRef<boolean>(false);

  // Use the flight plans context
  const { addPlan, plans } = useFlightPlans();
  console.log('FlightPlannerMap: Context initialized, current plans count:', plans.length);

  const setModeSafe = (m: Mode) => {
    console.log('FlightPlannerMap: Setting mode to:', m);
    modeRef.current = m;
    setMode(m);
  };

  const emitChange = () => {
    if (!sourceRef.current || !onChange) return;
    onChange(toFlightPlan(sourceRef.current));
  };

  const clearPlan = () => {
    drawingPtsRef.current = [];
    pointerCoordRef.current = null;
    planLineRef.current = null;
    previewLineRef.current = null;
    sourceRef.current?.clear();
    emitChange();
  };

  const startDrawing = () => {
    clearPlan();
    setModeSafe('drawing');
  };

  const ensurePlanLine = (): Feature<LineString> | null => {
    if (!sourceRef.current) return null;
    if (planLineRef.current) return planLineRef.current;
    const f = new Feature<LineString>(new LineString([]));
    f.set('kind', 'plan');
    f.setStyle(pathStyle);
    sourceRef.current.addFeature(f);
    planLineRef.current = f;
    return f;
  };

  const ensurePreviewLine = (): Feature<LineString> | null => {
    if (!sourceRef.current) return null;
    if (previewLineRef.current) return previewLineRef.current;
    const f = new Feature<LineString>(new LineString([]));
    f.set('kind', 'preview');
    f.setStyle(previewStyle);
    sourceRef.current.addFeature(f);
    previewLineRef.current = f;
    return f;
  };

  const addWaypoint = (coord3857: [number, number]) => {
    if (!sourceRef.current) return;

    const idx = drawingPtsRef.current.length;
    drawingPtsRef.current.push(coord3857);

    const wp = new Feature<Point>(new Point(coord3857));
    wp.set('kind', 'waypoint');
    wp.set('idx', idx);
    wp.setStyle(waypointStyle);
    sourceRef.current.addFeature(wp);

    const line = ensurePlanLine();
    if (line) (line.getGeometry() as LineString).setCoordinates([...drawingPtsRef.current]);

    emitChange();
  };

  const updatePreview = () => {
    const pts = drawingPtsRef.current;
    const p = pointerCoordRef.current;
    const preview = ensurePreviewLine();
    if (!preview) return;

    if (modeRef.current !== 'drawing' || pts.length === 0 || !p) {
      (preview.getGeometry() as LineString).setCoordinates([]);
      return;
    }

    (preview.getGeometry() as LineString).setCoordinates([pts[pts.length - 1], p]);
  };

  const finishDrawing = () => {
    console.log('FlightPlannerMap: finishDrawing called, current mode:', modeRef.current);
    setModeSafe('idle');
    const preview = previewLineRef.current;
    if (preview) (preview.getGeometry() as LineString).setCoordinates([]);
    
    // Convert the current drawing to a flight plan and add it to context
    if (drawingPtsRef.current.length > 0) {
      console.log('FlightPlannerMap: Converting drawing to flight plan, points count:', drawingPtsRef.current.length);
      const plan: FlightPlan = toFlightPlan(sourceRef.current!);
      console.log('FlightPlannerMap: Converted plan:', plan);
      const planData = {
        name: `Flight Plan ${plans.length + 1}`,
        plan: plan
      };
      console.log('FlightPlannerMap: Calling addPlan with:', planData);
      addPlan(planData);
      console.log('FlightPlannerMap: addPlan completed');
    } else {
      console.log('FlightPlannerMap: No points to convert, skipping plan creation');
    }
    
    emitChange();
  };

  const setFromValue = (plan: FlightPlan) => {
    if (!sourceRef.current) return;
    drawingPtsRef.current = [];
    pointerCoordRef.current = null;
    planLineRef.current = null;
    previewLineRef.current = null;

    renderPlanToSource(sourceRef.current, plan);

    const pts3857 = plan.points.map((p) => fromLonLat([p.lon, p.lat]) as [number, number]);
    drawingPtsRef.current = pts3857;

    const line = sourceRef.current
      .getFeatures()
      .find((f) => f.get('kind') === 'plan') as Feature<LineString> | undefined;
    planLineRef.current = line ?? null;
  };

  const installHandlers = () => {
    const map = mapRef.current;
    if (!map) return () => undefined;

    const viewport = map.getViewport();

    const moveStartKey: EventsKey = map.on('movestart', () => {
      isPanningRef.current = true;
      movedRef.current = true;
    });

    const moveEndKey: EventsKey = map.on('moveend', () => {
      isPanningRef.current = false;
      downPixelRef.current = null;
    });

    const onPointerDown = (e: PointerEvent) => {
      if (modeRef.current !== 'drawing') return;
      if (e.button !== 0) return;
      const px = map.getEventPixel(e) as [number, number];
      downPixelRef.current = px;
      movedRef.current = false;
    };

    const onPointerMove = (e: PointerEvent) => {
      if (modeRef.current !== 'drawing') return;

      const px = map.getEventPixel(e) as [number, number];
      if (downPixelRef.current) {
        const dx = px[0] - downPixelRef.current[0];
        const dy = px[1] - downPixelRef.current[1];
        if (dx * dx + dy * dy >= 36) movedRef.current = true;
      }

      const coord = map.getCoordinateFromPixel(px) as [number, number];
      pointerCoordRef.current = coord;
      updatePreview();
    };

    const onClick = (e: MouseEvent) => {
      if (modeRef.current !== 'drawing') return;
      if (e.button !== 0) return;
      if (isPanningRef.current || movedRef.current) return;

      const px = map.getEventPixel(e) as [number, number];
      const coord = map.getCoordinateFromPixel(px) as [number, number];
      addWaypoint(coord);
    };

    const onDblClick = (e: MouseEvent) => {
      if (modeRef.current !== 'drawing') return;
      e.preventDefault();
      e.stopPropagation();
      finishDrawing();
    };

    const onContextMenu = (e: MouseEvent) => {
      if (modeRef.current !== 'drawing') return;
      e.preventDefault();
      e.stopPropagation();
      finishDrawing();
    };

    const onKeyDown = (e: KeyboardEvent) => {
      if (modeRef.current !== 'drawing') return;
      if (e.key === 'Escape') {
        e.preventDefault();
        finishDrawing();
      }
    };

    viewport.addEventListener('pointerdown', onPointerDown);
    viewport.addEventListener('pointermove', onPointerMove);
    viewport.addEventListener('click', onClick);
    viewport.addEventListener('dblclick', onDblClick);
    viewport.addEventListener('contextmenu', onContextMenu);
    window.addEventListener('keydown', onKeyDown, { capture: true });

    return () => {
      viewport.removeEventListener('pointerdown', onPointerDown);
      viewport.removeEventListener('pointermove', onPointerMove);
      viewport.removeEventListener('click', onClick);
      viewport.removeEventListener('dblclick', onDblClick);
      viewport.removeEventListener('contextmenu', onContextMenu);
      window.removeEventListener('keydown', onKeyDown, { capture: true } as any);
      unByKey(moveStartKey);
      unByKey(moveEndKey);
    };
  };

  const vectorLayer = useMemo(() => {
    const source = new VectorSource();
    sourceRef.current = source;
    return new VectorLayer({ source, zIndex: 10 });
  }, []);

  useEffect(() => {
    if (!mapTargetRef.current) return;

    const raster = new TileLayer({ source: new OSM() });
    const view = new View({
      center: fromLonLat([initialCenterLonLat.lon, initialCenterLonLat.lat]),
      zoom: clampZoom(initialZoom),
    });

    const map = new Map({
      target: mapTargetRef.current,
      layers: [raster, vectorLayer],
      view,
      interactions: defaultInteractions({ doubleClickZoom: false }),
    });

    mapRef.current = map;

    const cleanup = installHandlers();

    return () => {
      cleanup();
      map.setTarget(undefined);
      mapRef.current = null;
      sourceRef.current = null;
      planLineRef.current = null;
      previewLineRef.current = null;
      drawingPtsRef.current = [];
      pointerCoordRef.current = null;
      isPanningRef.current = false;
      downPixelRef.current = null;
      movedRef.current = false;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useEffect(() => {
    if (!value) return;
    setFromValue(value);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [value?.points?.length]);

  return (
    <div
      ref={containerRef}
      className={className}
      style={{
        position: 'relative',
        width: '100%',
        height: '100%',
        minHeight: '320px',
        borderRadius: 12,
        overflow: 'hidden',
      }}
    >
      <div ref={mapTargetRef} style={{ position: 'absolute', inset: 0 }} />

      <TopRightButtonsStack>
        <IconButton
          title={mode === 'drawing' ? 'Finish plan' : 'New plan'}
          active={mode === 'drawing'}
          onClick={() => (modeRef.current === 'drawing' ? finishDrawing() : startDrawing())}
          icon={<SVGFlagPlusIcon />}
          iconScale={0.82}
        />
        <IconButton title="Clear" onClick={clearPlan} icon={<SVGTrashIcon />} iconScale={0.78} />
      </TopRightButtonsStack>
    </div>
  );
};