import React, { useState, useEffect, useRef } from 'react';
import { ros2Bridge } from 'tensorfleet-ros';
import { DroneStateModel } from 'tensorfleet-util/drone/drone-state-model';
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

export const MissionControlPanel: React.FC = () => {
    const [, setConnectionStatus] = useState<'connected' | 'connecting' | 'disconnected'>('connecting');
    const [activePanel, setActivePanel] = useState<'mission-planning' | 'drone-status'>('mission-planning');
    const [missionPlanningRequestKey, setMissionPlanningRequestKey] = useState(0);
    const [flightPlans, setFlightPlans] = useState<FlightPlanRecord[]>(() => loadPersistedFlightPlans());
    const [selectedFlightPlanId, setSelectedFlightPlanId] = useState<string | null>(null);
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
            activePanel={activePanel}
            onSelectPanel={setActivePanel}
            onFlightPlanPathChange={handleFlightPlanPathChange}
          />
          {activePanel === 'mission-planning' ? (
            <MissionPlanningPanel
              flightPlans={flightPlans}
              selectedFlightPlanId={selectedFlightPlanId}
              onStartNewPlan={handleStartNewPlan}
              onSelectFlightPlan={handleSelectFlightPlan}
              onDeleteFlightPlan={handleDeleteFlightPlan}
            />
          ) : (
            <DroneStatusPanel model={droneState} />
          )}
        </div>
      </ConnectionSettingsProvider>
    );
}
