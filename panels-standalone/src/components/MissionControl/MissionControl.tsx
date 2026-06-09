import React, { useState, useEffect } from 'react';
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

export const MissionControlPanel: React.FC = () => {
    const [, setConnectionStatus] = useState<'connected' | 'connecting' | 'disconnected'>('connecting');
    const [activePanel, setActivePanel] = useState<'mission-planning' | 'drone-status'>('mission-planning');
    const [missionPlanningRequestKey, setMissionPlanningRequestKey] = useState(0);

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
            activePanel={activePanel}
            onSelectPanel={setActivePanel}
          />
          {activePanel === 'mission-planning' ? (
            <MissionPlanningPanel onStartNewPlan={() => setMissionPlanningRequestKey((key) => key + 1)} />
          ) : (
            <DroneStatusPanel model={droneState} />
          )}
        </div>
      </ConnectionSettingsProvider>
    );
}
