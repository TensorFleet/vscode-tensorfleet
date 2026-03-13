import React, { useState, useEffect, useRef } from 'react';
import { ros2Bridge } from 'tensorfleet-ros';
import { DroneStateModel } from '../../../packages/tensorfleet-util/src/drone/drone-state-model';
import { DroneMap } from './map/DroneMap';
import './MissionControl.css';
import { DroneStatusPanel } from './drone/DroneStatusPanel';
import { DroneController } from 'tensorfleet-util/drone/mission-control/drone-controller';
import MissionControlBridge from './drone/MissionControlBridge';
import SimulationControlBridge from '../SimulationControl/SimulationControlBridge';
import { SimulationController } from '@/simulation/simulation_controller';
import { ConnectionSettingsProvider, ConnectionSettingsTrigger } from '../ConnectionSettingsProvider';

const droneState = new DroneStateModel();
const droneController = new DroneController(droneState, ros2Bridge);
const simulationController = new SimulationController();

export const MissionControlPanel: React.FC = () => {
    const [connectionStatus, setConnectionStatus] = useState<'connected' | 'connecting' | 'disconnected'>('connecting');

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
        <div className="mission-control-header">
          <ConnectionSettingsTrigger />
        </div>
        <MissionControlBridge controller={droneController} />
        <SimulationControlBridge controller={simulationController} />
        <DroneMap
            model = {droneState}
            >
        </DroneMap>
        <DroneStatusPanel
          model = {droneState}>
        </DroneStatusPanel>
      </div>
    </ConnectionSettingsProvider>);
}
