import React, { useState, useEffect, useRef } from 'react';
import { ros2Bridge } from 'tensorfleet-ros';
import { DroneStateModel } from 'tensorfleet-util/drone/drone-state-model';
import { MavrosMissionCommand, MavrosMissionWaypoint, type MavrosMsgsWaypoint } from 'tensorfleet-util/ros/ros-types/mavros-msgs-waypoint';
import { fromLonLat, toLonLat } from 'ol/proj';
import { DroneMap } from './map/DroneMap';
import './MissionControl.css';
import { DroneStatusPanel } from './drone/DroneStatusPanel';
import { MissionPlanningPanel } from './drone/MissionPlanningPanel';
import { DroneController } from 'tensorfleet-util/drone/mission-control/drone-controller';
import MissionControlBridge from './drone/MissionControlBridge';
import SimulationControlBridge from '../SimulationControl/SimulationControlBridge';
import { SimulationController } from '@/simulation/simulation_controller';
import { ConnectionSettingsProvider, ConnectionSettingsTrigger } from '../ConnectionSettingsProvider';

const droneState = new DroneStateModel();
const droneController = new DroneController(droneState, ros2Bridge);
const simulationController = new SimulationController();

type FlightPlanRecord = {
    id: string;
    name: string;
    path: [number, number][];
};

const DEFAULT_MISSION_ALTITUDE_METERS = 3;

function loadPersistedFlightPlans(): FlightPlanRecord[] {
    return [];
}

function persistFlightPlans(_flightPlans: FlightPlanRecord[]): void {
    // Placeholder for future persistence wiring.
}

function getNextFlightPlanNumber(flightPlans: FlightPlanRecord[]): number {
    let maxMissionNumber = 0;

    for (const flightPlan of flightPlans) {
        const match = /^Mission (\d+)$/.exec(flightPlan.name);
        if (!match) continue;

        const missionNumber = Number(match[1]);
        if (Number.isFinite(missionNumber)) {
            maxMissionNumber = Math.max(maxMissionNumber, missionNumber);
        }
    }

    return maxMissionNumber + 1;
}

function buildMissionFromFlightPlan(flightPlan: FlightPlanRecord): MavrosMsgsWaypoint[] {
    return flightPlan.path.map(([x, y], index) => {
        const [longitude, latitude] = toLonLat([x, y]);

        return new MavrosMissionWaypoint({
            command: MavrosMissionCommand.GO_TO,
            latitude,
            longitude,
            altitude: DEFAULT_MISSION_ALTITUDE_METERS,
            isCurrent: index === 0,
        });
    });
}

function getOngoingMissionPath(waypoints: MavrosMsgsWaypoint[] | undefined): [number, number][] {
    if (!waypoints || waypoints.length === 0) {
        return [];
    }

    const path: [number, number][] = [];

    for (const waypoint of waypoints) {
        if (!Number.isFinite(waypoint.x_lat) || !Number.isFinite(waypoint.y_long)) {
            continue;
        }
        if (Math.abs(waypoint.x_lat) > 90 || Math.abs(waypoint.y_long) > 180) {
            continue;
        }
        if (waypoint.x_lat === 0 && waypoint.y_long === 0) {
            continue;
        }

        path.push(fromLonLat([waypoint.y_long, waypoint.x_lat]) as [number, number]);
    }

    return path;
}

function getOngoingMissionLabel(waypoints: MavrosMsgsWaypoint[] | undefined, currentSeq: number | undefined): string {
    if (!waypoints || waypoints.length === 0) {
        return '';
    }

    const itemNumber = typeof currentSeq === 'number' && Number.isFinite(currentSeq)
        ? currentSeq + 1
        : 1;

    return `Ongoing mission ${itemNumber}/${waypoints.length}`;
}

export const MissionControlPanel: React.FC = () => {
    const [, setConnectionStatus] = useState<'connected' | 'connecting' | 'disconnected'>('connecting');
    const [activePanel, setActivePanel] = useState<'mission-planning' | 'drone-status'>('mission-planning');
    const [missionPlanningRequestKey, setMissionPlanningRequestKey] = useState(0);
    const [flightPlans, setFlightPlans] = useState<FlightPlanRecord[]>(() => loadPersistedFlightPlans());
    const [selectedFlightPlanId, setSelectedFlightPlanId] = useState<string | null>(null);
    const [ongoingMissionPath, setOngoingMissionPath] = useState<[number, number][]>([]);
    const [ongoingMissionLabel, setOngoingMissionLabel] = useState('');
    const nextFlightPlanNumberRef = useRef<number>(getNextFlightPlanNumber(flightPlans));

    useEffect(() => {
        // Ensure connection to rosbridge (single supported mode)
        console.log("Initializing mission control panel")
        // ros2Bridge.connect('rosbridge');

        droneState.connect(ros2Bridge);
    
        // Check connection status periodically
        const statusInterval = setInterval(() => {
          const isConnected = ros2Bridge.isConnected();
          setConnectionStatus(isConnected ? 'connected' : 'disconnected');
        }, 1000);
    
        return () => {
          droneState.disconnect();
        //   ros2Bridge.disconnect();
          clearInterval(statusInterval);
        };
      }, []);

    useEffect(() => {
        persistFlightPlans(flightPlans);
    }, [flightPlans]);

    useEffect(() => {
        const unsubscribe = droneState.onUpdate(() => {
            const state = droneState.getCurrentState();
            const missionWaypoints =
                state.vehicle?.mode === 'AUTO.MISSION' && !state.mission?.completed
                    ? state.mission?.waypoints
                    : undefined;
            setOngoingMissionPath(getOngoingMissionPath(missionWaypoints));
            setOngoingMissionLabel(getOngoingMissionLabel(missionWaypoints, state.mission?.current_seq));
        });

        return () => {
            unsubscribe();
        };
    }, []);

    const handleStartNewPlan = () => {
        const nextPlanNumber = nextFlightPlanNumberRef.current;
        nextFlightPlanNumberRef.current += 1;
        const newFlightPlan: FlightPlanRecord = {
            id: `flight-plan-${Date.now()}`,
            name: `Mission ${nextPlanNumber}`,
            path: [],
        };

        setFlightPlans((current) => [...current, newFlightPlan]);
        setSelectedFlightPlanId(newFlightPlan.id);
        setMissionPlanningRequestKey((key) => key + 1);
        setActivePanel('mission-planning');
    };

    const handleSelectFlightPlan = (flightPlanId: string) => {
        setSelectedFlightPlanId(flightPlanId);
        setActivePanel('mission-planning');
    };

    const handleDeleteFlightPlan = (flightPlanId: string) => {
        setFlightPlans((current) => {
            const nextFlightPlans = current.filter((flightPlan) => flightPlan.id !== flightPlanId);
            if (selectedFlightPlanId === flightPlanId) {
                setSelectedFlightPlanId(nextFlightPlans[0]?.id ?? null);
            }
            return nextFlightPlans;
        });
    };

    const handleFlightPlanPathChange = (flightPlanId: string, path: [number, number][]) => {
        setFlightPlans((current) =>
            current.map((flightPlan) =>
                flightPlan.id === flightPlanId ? { ...flightPlan, path } : flightPlan
            )
        );
    };

    const handleSendFlightPlan = (flightPlanId: string) => {
        const flightPlan = flightPlans.find((plan) => plan.id === flightPlanId);
        if (!flightPlan || flightPlan.path.length === 0) {
            return;
        }

        const mission = buildMissionFromFlightPlan(flightPlan);
        void droneController.sendMissionRequest(mission)
            .catch((error) => {
                console.error(`Failed to send mission "${flightPlan.name}"`, error);
            });
    };

    const handleStopOngoingMission = () => {
        const state = droneState.getCurrentState();
        const currentAltitude =
            state.altitude?.relative ??
            state.altitude?.agl ??
            state.global_position_int?.relative_alt ??
            Math.abs(state.local?.position?.z ?? 0) ??
            DEFAULT_MISSION_ALTITUDE_METERS;

        void droneController.requestAutoState({
            kind: 'airborne',
            altMeters: currentAltitude > 0 ? currentAltitude : DEFAULT_MISSION_ALTITUDE_METERS,
        }).then(() => {
        }).catch((error) => {
            console.error('Failed to stop ongoing mission', error);
        });
    };

    return (
      <ConnectionSettingsProvider onSettingsChange={(settings) => {
        // Handle connection settings changes - could trigger reconnection
        console.log('Connection settings changed:', settings);
        // TODO: Implement reconnection logic if needed
      }}>
        <div className="mission-control-panel">
          <ConnectionSettingsTrigger className="mission-control-settings-trigger" />
          <MissionControlBridge controller={droneController} />
          <SimulationControlBridge controller={simulationController} />
          <DroneMap
            model={droneState}
            missionPlanningRequestKey={missionPlanningRequestKey}
            flightPlans={flightPlans}
            selectedFlightPlanId={selectedFlightPlanId}
            ongoingMissionPath={ongoingMissionPath}
            activePanel={activePanel}
            onSelectPanel={setActivePanel}
            onFlightPlanPathChange={handleFlightPlanPathChange}
          />
          {activePanel === 'mission-planning' ? (
            <MissionPlanningPanel
              flightPlans={flightPlans}
              selectedFlightPlanId={selectedFlightPlanId}
              ongoingMissionLabel={ongoingMissionLabel}
              hasOngoingMission={ongoingMissionPath.length >= 2}
              onStartNewPlan={handleStartNewPlan}
              onSelectFlightPlan={handleSelectFlightPlan}
              onSendFlightPlan={handleSendFlightPlan}
              onDeleteFlightPlan={handleDeleteFlightPlan}
              onStopOngoingMission={handleStopOngoingMission}
            />
          ) : (
            <DroneStatusPanel model={droneState} />
          )}
        </div>
      </ConnectionSettingsProvider>
    );
}
