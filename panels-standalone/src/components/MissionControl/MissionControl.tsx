import React, { useState, useEffect, useRef } from 'react';
import { ros2Bridge } from '../../ros2-bridge';
import { DroneStateModel } from './DroneStateModel';
import { DroneMap } from './map/DroneMap';
import './MissionControl.css';


export const MissionControlPanel: React.FC = () => {
    const [droneState, setDroneState ] = useState<DroneStateModel>(new DroneStateModel());
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
        <DroneMap
            model = {droneState}
            >
        </DroneMap>
    </div>);
}