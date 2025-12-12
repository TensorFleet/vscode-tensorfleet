import React, { createContext, useContext, useState, ReactNode } from 'react';
import { ConnectionSettingsOverlay } from './ConnectionSettingsOverlay';
import { ConnectionSettingsButton } from './ConnectionSettingsButton';

interface ConnectionSettings {
  proxyUrl: string;
  vmManagerUrl: string;
  nodeId: string;
  token: string;
  targetPort: number;
}

interface ConnectionSettingsContextType {
  isOpen: boolean;
  openOverlay: () => void;
  closeOverlay: () => void;
  onSettingsChange: (settings: ConnectionSettings) => void;
}

const ConnectionSettingsContext = createContext<ConnectionSettingsContextType | undefined>(undefined);

export const useConnectionSettings = () => {
  const context = useContext(ConnectionSettingsContext);
  if (context === undefined) {
    throw new Error('useConnectionSettings must be used within a ConnectionSettingsProvider');
  }
  return context;
};

interface ConnectionSettingsProviderProps {
  children: ReactNode;
  onSettingsChange?: (settings: ConnectionSettings) => void;
}

export const ConnectionSettingsProvider: React.FC<ConnectionSettingsProviderProps> = ({
  children,
  onSettingsChange,
}) => {
  const [isOpen, setIsOpen] = useState(false);

  const openOverlay = () => setIsOpen(true);
  const closeOverlay = () => setIsOpen(false);

  const handleSettingsChange = (settings: ConnectionSettings) => {
    onSettingsChange?.(settings);
  };

  const contextValue: ConnectionSettingsContextType = {
    isOpen,
    openOverlay,
    closeOverlay,
    onSettingsChange: handleSettingsChange,
  };

  return (
    <ConnectionSettingsContext.Provider value={contextValue}>
      {children}
      <ConnectionSettingsOverlay
        isOpen={isOpen}
        onClose={closeOverlay}
        onSettingsChange={handleSettingsChange}
      />
    </ConnectionSettingsContext.Provider>
  );
};

export const ConnectionSettingsTrigger: React.FC<{ className?: string }> = ({ className }) => {
  const { openOverlay } = useConnectionSettings();

  return (
    <ConnectionSettingsButton onClick={openOverlay} className={className} />
  );
};
