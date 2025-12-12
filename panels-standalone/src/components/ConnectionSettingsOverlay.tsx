import React, { useState, useEffect } from 'react';
import './ConnectionSettingsOverlay.css';

interface ConnectionSettings {
  proxyUrl: string;
  vmManagerUrl: string;
  nodeId: string;
  token: string;
  targetPort: number;
}

interface ConnectionSettingsOverlayProps {
  isOpen: boolean;
  onClose: () => void;
  onSettingsChange: (settings: ConnectionSettings) => void;
}

export const ConnectionSettingsOverlay: React.FC<ConnectionSettingsOverlayProps> = ({
  isOpen,
  onClose,
  onSettingsChange,
}) => {
  const [settings, setSettings] = useState<ConnectionSettings>({
    proxyUrl: (window as any).TENSORFLEET_PROXY_URL || '',
    vmManagerUrl: (window as any).TENSORFLEET_VM_MANAGER_URL || '',
    nodeId: (window as any).TENSORFLEET_NODE_ID || '',
    token: (window as any).TENSORFLEET_JWT || '',
    targetPort: 8765, // Default Foxglove Bridge port
  });

  const handleInputChange = (field: keyof ConnectionSettings, value: string | number) => {
    setSettings(prev => ({ ...prev, [field]: value }));
  };

  const handleApply = () => {
    // Update window globals
    (window as any).TENSORFLEET_PROXY_URL = settings.proxyUrl;
    (window as any).TENSORFLEET_VM_MANAGER_URL = settings.vmManagerUrl;
    (window as any).TENSORFLEET_NODE_ID = settings.nodeId;
    (window as any).TENSORFLEET_JWT = settings.token;

    // Notify parent via postMessage
    window.parent.postMessage({
      type: 'CONNECTION_SETTINGS_CHANGED',
      settings: settings,
    }, '*');

    onSettingsChange(settings);
    onClose();
  };

  const handleReset = () => {
    const defaultSettings: ConnectionSettings = {
      proxyUrl: '',
      vmManagerUrl: '',
      nodeId: '',
      token: '',
      targetPort: 8765,
    };
    setSettings(defaultSettings);
  };

  if (!isOpen) return null;

  return (
    <>
      <div className="connection-overlay-backdrop" onClick={onClose} />
      <div className="connection-settings-overlay">
        <div className="connection-settings-caret" />
        <div className="connection-settings-content">
          <h3>Connection Settings</h3>

          <div className="setting-group">
            <label htmlFor="proxyUrl">Proxy URL:</label>
            <input
              id="proxyUrl"
              type="text"
              value={settings.proxyUrl}
              onChange={(e) => handleInputChange('proxyUrl', e.target.value)}
              placeholder="ws://proxy.example.com"
            />
          </div>

          <div className="setting-group">
            <label htmlFor="vmManagerUrl">VM Manager URL:</label>
            <input
              id="vmManagerUrl"
              type="text"
              value={settings.vmManagerUrl}
              onChange={(e) => handleInputChange('vmManagerUrl', e.target.value)}
              placeholder="ws://vm-manager.example.com"
            />
          </div>

          <div className="setting-group">
            <label htmlFor="nodeId">Node ID:</label>
            <input
              id="nodeId"
              type="text"
              value={settings.nodeId}
              onChange={(e) => handleInputChange('nodeId', e.target.value)}
              placeholder="node-123"
            />
          </div>

          <div className="setting-group">
            <label htmlFor="token">JWT Token:</label>
            <input
              id="token"
              type="password"
              value={settings.token}
              onChange={(e) => handleInputChange('token', e.target.value)}
              placeholder="jwt-token-here"
            />
          </div>

          <div className="setting-group">
            <label htmlFor="targetPort">Target Port:</label>
            <input
              id="targetPort"
              type="number"
              value={settings.targetPort}
              onChange={(e) => handleInputChange('targetPort', parseInt(e.target.value) || 8765)}
              placeholder="8765"
            />
          </div>

          <div className="connection-settings-actions">
            <button onClick={handleReset} className="reset-btn">
              Reset
            </button>
            <div className="spacer" />
            <button onClick={onClose} className="cancel-btn">
              Cancel
            </button>
            <button onClick={handleApply} className="apply-btn">
              Apply
            </button>
          </div>
        </div>
      </div>
    </>
  );
};
