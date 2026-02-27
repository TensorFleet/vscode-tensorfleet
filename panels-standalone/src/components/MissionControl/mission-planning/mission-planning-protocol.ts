/**
 * Mission Planning Communication Protocol
 * 
 * This file defines the communication protocol for mission planning operations
 * using CustomEvent-based messaging between UI components and mission controllers.
 */

// ---------- Mission Planning Actions ----------
export type MissionPlanningAction = 
  | "create_mission"
  | "save_mission" 
  | "load_mission"
  | "delete_mission"
  | "cancel_mission"
  | "finalize_mission"
  | "execute_mission"
  | "list_missions";

// ---------- Mission Data Structures ----------
export interface MissionWaypoint {
  id: string;
  latitude: number;
  longitude: number;
  altitude: number;
  yaw?: number;
  speed?: number;
  holdTime?: number;
  action?: string;
}

export interface MissionPlan {
  id: string;
  name: string;
  description?: string;
  createdAt: number;
  updatedAt: number;
  waypoints: MissionWaypoint[];
  status: "draft" | "saved" | "executing" | "completed" | "cancelled";
  metadata?: {
    totalDistance?: number;
    estimatedDuration?: number;
    maxAltitude?: number;
    minAltitude?: number;
  };
}

export interface MissionListResponse {
  missions: MissionPlan[];
  total: number;
}

// ---------- Event Contract ----------
export interface MissionPlanningEventDetail<T = any> {
  category: "mission_planning";
  action: MissionPlanningAction;
  payload?: T;
  requestId: string;
  timestamp: number;
}

const EVENT_NAME = "app:request";

// ---------- Event Dispatch Functions ----------
export function postMissionPlanningRequest<T = any>(
  action: MissionPlanningAction, 
  payload?: T
): MissionPlanningEventDetail<T> {
  const requestId = crypto.randomUUID();
  const detail: MissionPlanningEventDetail<T> = {
    category: "mission_planning",
    action,
    payload,
    requestId,
    timestamp: Date.now(),
  };
  
  window.dispatchEvent(
    new CustomEvent<MissionPlanningEventDetail<T>>(EVENT_NAME, { detail })
  );
  
  return detail;
}

// ---------- Specific Mission Operations ----------

export function createMission(name: string, description?: string): MissionPlanningEventDetail<{
  name: string;
  description?: string;
}> {
  return postMissionPlanningRequest("create_mission", { name, description });
}

export function saveMission(mission: MissionPlan): MissionPlanningEventDetail<{
  mission: MissionPlan;
}> {
  return postMissionPlanningRequest("save_mission", { mission });
}

export function loadMission(missionId: string): MissionPlanningEventDetail<{
  missionId: string;
}> {
  return postMissionPlanningRequest("load_mission", { missionId });
}

export function deleteMission(missionId: string): MissionPlanningEventDetail<{
  missionId: string;
}> {
  return postMissionPlanningRequest("delete_mission", { missionId });
}

export function cancelMission(missionId: string): MissionPlanningEventDetail<{
  missionId: string;
}> {
  return postMissionPlanningRequest("cancel_mission", { missionId });
}

export function finalizeMission(missionId: string): MissionPlanningEventDetail<{
  missionId: string;
}> {
  return postMissionPlanningRequest("finalize_mission", { missionId });
}

export function executeMission(missionId: string): MissionPlanningEventDetail<{
  missionId: string;
}> {
  return postMissionPlanningRequest("execute_mission", { missionId });
}

export function listMissions(): MissionPlanningEventDetail {
  return postMissionPlanningRequest("list_missions");
}

// ---------- Mission Validation ----------
export class MissionValidationError extends Error {
  constructor(message: string, public field?: string) {
    super(message);
    this.name = "MissionValidationError";
  }
}

export function validateMission(mission: MissionPlan): MissionValidationError[] {
  const errors: MissionValidationError[] = [];
  
  if (!mission.name || mission.name.trim().length === 0) {
    errors.push(new MissionValidationError("Mission name is required", "name"));
  }
  
  if (mission.waypoints.length < 2) {
    errors.push(new MissionValidationError("Mission must have at least 2 waypoints", "waypoints"));
  }
  
  mission.waypoints.forEach((wp, index) => {
    if (!Number.isFinite(wp.latitude) || Math.abs(wp.latitude) > 90) {
      errors.push(new MissionValidationError(
        `Invalid latitude for waypoint ${index + 1}`, 
        `waypoints[${index}].latitude`
      ));
    }
    
    if (!Number.isFinite(wp.longitude) || Math.abs(wp.longitude) > 180) {
      errors.push(new MissionValidationError(
        `Invalid longitude for waypoint ${index + 1}`, 
        `waypoints[${index}].longitude`
      ));
    }
    
    if (!Number.isFinite(wp.altitude) || wp.altitude < 0) {
      errors.push(new MissionValidationError(
        `Invalid altitude for waypoint ${index + 1}`, 
        `waypoints[${index}].altitude`
      ));
    }
  });
  
  return errors;
}

// ---------- Mission Utilities ----------
export function calculateMissionStats(mission: MissionPlan): {
  totalDistance: number;
  estimatedDuration: number;
  maxAltitude: number;
  minAltitude: number;
} {
  let totalDistance = 0;
  let maxAltitude = -Infinity;
  let minAltitude = Infinity;
  
  for (let i = 1; i < mission.waypoints.length; i++) {
    const prev = mission.waypoints[i - 1];
    const curr = mission.waypoints[i];
    
    // Simple distance calculation (Haversine would be more accurate for long distances)
    const latDiff = (curr.latitude - prev.latitude) * 111000; // meters
    const lonDiff = (curr.longitude - prev.longitude) * 111000 * Math.cos(prev.latitude * Math.PI / 180);
    const altDiff = curr.altitude - prev.altitude;
    
    const segmentDistance = Math.sqrt(latDiff * latDiff + lonDiff * lonDiff + altDiff * altDiff);
    totalDistance += segmentDistance;
    
    maxAltitude = Math.max(maxAltitude, curr.altitude);
    minAltitude = Math.min(minAltitude, curr.altitude);
  }
  
  // Simple duration estimate (assuming 10 m/s speed)
  const estimatedDuration = totalDistance / 10;
  
  return {
    totalDistance,
    estimatedDuration,
    maxAltitude: maxAltitude === -Infinity ? 0 : maxAltitude,
    minAltitude: minAltitude === Infinity ? 0 : minAltitude,
  };
}

export function generateWaypointId(): string {
  return `wp_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}

export function createEmptyMission(name: string, description?: string): MissionPlan {
  return {
    id: `mission_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
    name,
    description,
    createdAt: Date.now(),
    updatedAt: Date.now(),
    waypoints: [],
    status: "draft",
    metadata: {
      totalDistance: 0,
      estimatedDuration: 0,
      maxAltitude: 0,
      minAltitude: 0,
    },
  };
}