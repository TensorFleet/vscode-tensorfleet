import React, { useState, useEffect, useRef } from 'react';
import { ros2Bridge } from '../../ros2-bridge';
import { DroneStateModel } from '../../mission-control/drone-state-model';
import { DroneMap } from './map/DroneMap';
import './MissionControl.css';
import { DroneStatusPanel } from './drone/DroneStatusPanel';
import { DroneController } from '@/mission-control/drone-controller';
import MissionControlBridge from './drone/MissionControlBridge';
import SimulationControlBridge from '../SimulationControl/SimulationControlBridge';
import { SimulationController } from '@/simulation/simulation_controller';

const droneState = new DroneStateModel();
const droneController = new DroneController(droneState);
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
    <div className="mission-control-panel">
        <MissionControlBridge controller={droneController} />
        <SimulationControlBridge controller={simulationController} />
        <DroneMap
            model = {droneState}
            >
        </DroneMap>
        <DroneStatusPanel
          model = {droneState}>
        </DroneStatusPanel>
    </div>);
}