import React, { useEffect, useRef, useState } from 'react';
import 'ol/ol.css';
import Map from 'ol/Map';
import View from 'ol/View';
import TileLayer from 'ol/layer/Tile';
import VectorLayer from 'ol/layer/Vector';
import OSM from 'ol/source/OSM';
import VectorSource from 'ol/source/Vector';
import Feature from 'ol/Feature';
import Point from 'ol/geom/Point';
import Style from 'ol/style/Style';
import Icon from 'ol/style/Icon';
import CircleStyle from 'ol/style/Circle';
import Fill from 'ol/style/Fill';
import Stroke from 'ol/style/Stroke';
import { fromLonLat } from 'ol/proj';
import { unByKey } from 'ol/Observable';
import type { EventsKey } from 'ol/events';

import type { DroneStateModel, DroneState } from '../DroneStateModel';
import { MapButtonsStack, FollowLockButton } from './MapButtons';
import { FlightPathTools } from './FlightPathTools';

type Props = {
  model: DroneStateModel;
  follow?: boolean;
  className?: string;
};

function toDegreesMaybeScaled(value: number | undefined): number | undefined {
  if (value == null) return undefined;
  if (Math.abs(value) > 180) return value / 1e7;
  return value;
}


// Bigger + fully compatible SVG (explicit width/height; rgb + fill-opacity)
const NAV_ICON_SRC =
  'data:image/svg+xml;utf8,' +
  encodeURIComponent(
    `<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="32" height="32" viewBox="0 0 32 32">
  <path d="M16 4 L22 28 L16 22 L10 28 Z" fill="rgb(255,0,0)" fill-opacity="0.7"/>
</svg>`
  );

// Tweak these to control the arrow size
const NAV_ICON_SIZE: [number, number] = [32, 32];
const NAV_ICON_SCALE = 1.5; // <— make the arrow larger/smaller here

function makeDroneStyles(rotationRad: number): Style[] {
  // Zero radians is north facing.
  const rot = -rotationRad;

  const dot = new Style({
    image: new CircleStyle({
      radius: 3,
      fill: new Fill({ color: '#ffffff' }),
      stroke: new Stroke({ color: '#ff0000', width: 2 }),
    }),
    zIndex: 2,
  });

  const arrow = new Style({
    image: new Icon({
        src: NAV_ICON_SRC,
        size: NAV_ICON_SIZE,
        rotation: rot,
        rotateWithView: true,
        anchor: [0.5, 0.5625], // center of the arrow shape
        anchorXUnits: 'fraction',
        anchorYUnits: 'fraction',
        scale: NAV_ICON_SCALE,
    }),
    zIndex: 1,
    });

  return [arrow, dot];
}

const TAU_POS_MS = 200;
const TAU_ROT_MS = 150;
function expSmoothingAlpha(dtMs: number, tauMs: number): number {
  if (tauMs <= 0) return 1;
  return 1 - Math.exp(-dtMs / tauMs);
}
function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}
function wrapAngleRad(angle: number): number {
  const twoPi = Math.PI * 2;
  return ((angle + Math.PI) % twoPi + twoPi) % twoPi - Math.PI;
}
function lerpAngleRad(a: number, b: number, t: number): number {
  const delta = wrapAngleRad(b - a);
  return a + delta * t;
}

const CAM_EASE_DURATION_MS = 450;
const easeOutCubic = (t: number) => 1 - Math.pow(1 - t, 3);

export const DroneMap: React.FC<Props> = ({ model, follow = true, className }) => {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<HTMLDivElement | null>(null);

  const mapObjRef = useRef<Map | null>(null);
  const vectorSourceRef = useRef<VectorSource | null>(null);
  const droneFeatureRef = useRef<Feature<Point> | null>(null);
  const hasCenteredRef = useRef<boolean>(false);

  const [isLocked, setIsLocked] = useState<boolean>(!!follow);
  const followLockedRef = useRef<boolean>(isLocked);
  useEffect(() => {
    followLockedRef.current = isLocked;
  }, [isLocked]);
  useEffect(() => {
    setIsLocked(!!follow);
  }, [follow]);

  const currentCoordRef = useRef<number[]>(fromLonLat([0, 0]));
  const targetCoordRef = useRef<number[]>(fromLonLat([0, 0]));
  const currentHeadingRef = useRef<number>(0);
  const targetHeadingRef = useRef<number>(0);
  const rafIdRef = useRef<number | null>(null);
  const lastTsRef = useRef<number | null>(null);

  const camEaseActiveRef = useRef<boolean>(false);
  const camEaseElapsedRef = useRef<number>(0);
  const camEaseStartRef = useRef<[number, number]>([0, 0]);
  const camEaseEndRef = useRef<[number, number]>([0, 0]);

  const panListenerKeyRef = useRef<EventsKey | null>(null);

  // IMPORTANT: expose map via state so children see a non-null map
  const [olMap, setOlMap] = useState<Map | null>(null);

  useEffect(() => {
    if (!mapRef.current) return;

    const raster = new TileLayer({ source: new OSM() });
    const vectorSource = new VectorSource();
    const vectorLayer = new VectorLayer({ source: vectorSource, zIndex: 10 });

    const initialCenter = fromLonLat([0, 0]);
    const view = new View({ center: initialCenter, zoom: 2 });

    const map = new Map({ target: mapRef.current, layers: [raster, vectorLayer], view });

    const droneFeature = new Feature<Point>(new Point(initialCenter));
    droneFeature.setStyle(makeDroneStyles(0));
    vectorSource.addFeature(droneFeature);

    mapObjRef.current = map;
    vectorSourceRef.current = vectorSource;
    droneFeatureRef.current = droneFeature;

    setOlMap(map);

    panListenerKeyRef.current = map.on('pointerdrag', () => {
      if (followLockedRef.current) {
        camEaseActiveRef.current = false;
        setIsLocked(false);
      }
    });

    const tick = (ts: number) => {
      if (!mapObjRef.current || !droneFeatureRef.current) return;

      const last = lastTsRef.current ?? ts;
      const dt = Math.max(0, ts - last);
      lastTsRef.current = ts;

      const aPos = expSmoothingAlpha(dt, TAU_POS_MS);
      const curr = currentCoordRef.current;
      const targ = targetCoordRef.current;
      const nextX = lerp(curr[0], targ[0], aPos);
      const nextY = lerp(curr[1], targ[1], aPos);
      const nextCoord: [number, number] = [nextX, nextY];

      const geom = droneFeatureRef.current.getGeometry();
      if (geom) geom.setCoordinates(nextCoord);

      const aRot = expSmoothingAlpha(dt, TAU_ROT_MS);
      const nextHeading = lerpAngleRad(currentHeadingRef.current, targetHeadingRef.current, aRot);
      droneFeatureRef.current.setStyle(makeDroneStyles(nextHeading));

      if (followLockedRef.current) {
        const v = mapObjRef.current.getView();
        if (camEaseActiveRef.current) {
          camEaseElapsedRef.current += dt;
          const tNorm = Math.min(1, camEaseElapsedRef.current / CAM_EASE_DURATION_MS);
          const k = easeOutCubic(tNorm);
          const s = camEaseStartRef.current;
          const e = camEaseEndRef.current;
          const cx = lerp(s[0], e[0], k);
          const cy = lerp(s[1], e[1], k);
          v.setCenter([cx, cy]);
          if (tNorm >= 1) camEaseActiveRef.current = false;
        } else {
          v.setCenter(nextCoord);
        }
      }

      currentCoordRef.current = nextCoord;
      currentHeadingRef.current = nextHeading;

      rafIdRef.current = requestAnimationFrame(tick);
    };

    rafIdRef.current = requestAnimationFrame(tick);

    return () => {
      if (rafIdRef.current != null) cancelAnimationFrame(rafIdRef.current);
      if (panListenerKeyRef.current) unByKey(panListenerKeyRef.current);
      map.setTarget(undefined);
      mapObjRef.current = null;
      vectorSourceRef.current = null;
      droneFeatureRef.current = null;
      hasCenteredRef.current = false;
      lastTsRef.current = null;
      setOlMap(null);
    };
  }, []);

  useEffect(() => {
    if (!mapObjRef.current || !droneFeatureRef.current) return;

    const unsubscribe = model.onUpdate((state) => {
      const gp = state.global_position_int;
      if (!gp) return;

      const lat = toDegreesMaybeScaled(gp.lat);
      const lon = toDegreesMaybeScaled(gp.lon);
      if (lat == null || lon == null) return;

      const coord = fromLonLat([lon, lat]);
      targetCoordRef.current = coord;

      const headingRad = state.yaw ?? 0.0;
      targetHeadingRef.current = headingRad;

      if (!hasCenteredRef.current) {
        currentCoordRef.current = coord;
        const geom = droneFeatureRef.current!.getGeometry();
        if (geom) geom.setCoordinates(coord);
        currentHeadingRef.current = headingRad;
        droneFeatureRef.current!.setStyle(makeDroneStyles(headingRad));

        if (followLockedRef.current) {
          const v = mapObjRef.current!.getView();
          v.setCenter(coord);
          if ((v.getZoom() ?? 0) < 16) v.setZoom(18);
        }
        hasCenteredRef.current = true;
      }
    });

    return () => {
      unsubscribe();
    };
  }, [model]);

  useEffect(() => {
    if (!mapObjRef.current) return;
    const v = mapObjRef.current.getView();

    if (isLocked) {
      const vc = (v.getCenter() as [number, number] | undefined) || currentCoordRef.current;
      camEaseStartRef.current = [vc[0], vc[1]];
      camEaseEndRef.current = [currentCoordRef.current[0], currentCoordRef.current[1]];
      camEaseElapsedRef.current = 0;
      camEaseActiveRef.current = true;
    } else {
      camEaseActiveRef.current = false;
    }
  }, [isLocked]);

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
      <div ref={mapRef} style={{ position: 'absolute', inset: 0 }} />

      <FlightPathTools
        map={olMap}
        onPathChange={(coords) => {
          // coords are in EPSG:3857
        }}
      />

      <MapButtonsStack corner="bottom-right">
        <FollowLockButton locked={isLocked} onToggle={() => setIsLocked((v) => !v)} />
      </MapButtonsStack>
    </div>
  );
};
