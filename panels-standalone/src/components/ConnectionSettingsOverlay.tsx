import React, { useState, useEffect } from 'react';
import './ConnectionSettingsOverlay.css';

interface ConnectionSettings {
  proxyUrl: string;
  vmManagerUrl: string;
  nodeId: string;
  token: string;
  targetPort: number;
  useProxy: boolean;
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
    useProxy: (window as any).TENSORFLEET_USE_PROXY !== undefined ? (window as any).TENSORFLEET_USE_PROXY : true,
  });

  const handleInputChange = (field: keyof ConnectionSettings, value: string | number | boolean) => {
    setSettings(prev => ({ ...prev, [field]: value }));
  };

  const handleApply = () => {
    // Update window globals
    (window as any).TENSORFLEET_PROXY_URL = settings.proxyUrl;
    (window as any).TENSORFLEET_VM_MANAGER_URL = settings.vmManagerUrl;
    (window as any).TENSORFLEET_NODE_ID = settings.nodeId;
    (window as any).TENSORFLEET_JWT = settings.token;
    (window as any).TENSORFLEET_USE_PROXY = settings.useProxy;

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
      useProxy: true,
    };
    setSettings(defaultSettings);
  };

  const handleResetFirst = () => {
    const defaultSettings: ConnectionSettings = {
      proxyUrl: '',
      vmManagerUrl: 'http://localhost:8080',
      nodeId: '[copy your vm id here]',
      token: 'eyJhbGciOiJSUzI1NiIsImNhdCI6ImNsX0I3ZDRQRDIyMkFBQSIsImtpZCI6Imluc18zNW1INUFPYU5ydmxqZFNRUVFibXZTZ2xJcG0iLCJ0eXAiOiJKV1QifQ.eyJlbWFpbCI6ImRhcnRobGV2aWVsbGlzQGdtYWlsLmNvbSIsImV4cCI6MTc2NTg0MDEwNSwiaWF0IjoxNzY1NzUzNzA1LCJpc3MiOiJodHRwczovL2NsZXJrLnRlbnNvcmZsZWV0Lm5ldCIsImp0aSI6IjQ5ZDkxZDUxZjM0YzY2M2UxMTZhIiwibmFtZSI6ImxldmkgZWxsaXMiLCJuYmYiOjE3NjU3NTM3MDAsInBpY3R1cmUiOiJodHRwczovL2ltZy5jbGVyay5jb20vZXlKMGVYQmxJam9pY0hKdmVIa2lMQ0p6Y21NaU9pSm9kSFJ3Y3pvdkwybHRZV2RsY3k1amJHVnlheTVrWlhZdmIyRjFkR2hmWjI5dloyeGxMMmx0WjE4ek5qVXdWa1pWV2pac2FsWlpTRmxYZVZWRFdEaE9aelpRYUVRaWZRIiwic3ViIjoidXNlcl8zNjUwVkNWS1JRZ0E4RDQ1Q1c4anFyUDRqYzYiLCJ1c2VySWQiOiJ1c2VyXzM2NTBWQ1ZLUlFnQThENDVDVzhqcXJQNGpjNiJ9.JFPgqhQWGCIhahuwCAqpiagHixAa55arPFOpZ2r3edMbxb_OFBxdW3J-nLTBJc_Vdig0qnIsRT3y8RYiyJayhW52eeh8DKGV-2J0c_xKpfMRN7WS3hCUDNDkDMFIQXWGf9NKXy6p36BWAYNEzrQi0ye4bRxZS873p73kZ1Nsf4kGBtVRH9AbChqLy2DZ6ZFxfW4hvUgS0R4y4mIDqSPZ7CNF50L4oCATucibyhWV8RbJsUlkhSU6HdhQHSY-eyBIX32Qg1J8naxSMFjyqKOpkgcCtAMaindjyeL0SL_T9EX7wcM4Gs5qFlx0SIo2WYPf1m3m5thYQpmzX9dHPZmQxQ&state=bcb6404412c1bfeaa1fe9e0c890bb2ddd7cb868bba392bce6f3cb7283d4a110a',
      targetPort: 8765,
      useProxy: true,
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

          <div className="setting-group">
            <label htmlFor="useProxy">Use Proxy:</label>
            <button
              id="useProxy"
              type="button"
              className={`toggle-btn ${settings.useProxy ? 'active' : ''}`}
              onClick={() => handleInputChange('useProxy', !settings.useProxy)}
            >
              {settings.useProxy ? 'Enabled' : 'Disabled'}
            </button>
          </div>

          <div className="connection-settings-actions">
            <button onClick={handleReset} className="reset-btn">
              Reset
            </button>
            <button onClick={handleResetFirst} className="reset-btn">
              Reset to first VM
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
