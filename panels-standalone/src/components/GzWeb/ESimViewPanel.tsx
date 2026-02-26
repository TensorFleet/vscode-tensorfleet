import React, { useEffect, useRef, useState } from 'react';
import { GzWebPanel } from './GzWebPanel';
import { FeaturedEntitiesPanel } from './FeaturedEntitiesPanel';
import { EntityInfoPopup } from './EntityInfoPopup';
import SidePanel from './SidePanel';
import './ESimViewPanel.css';
import './SidePanel.css';

interface ViewSubRect {
  fullWidth: number;
  fullHeight: number;
  offsetX: number;
  offsetY: number;
  width: number;
  height: number;
}

interface ViewportDetectorProps {
  sidePanelWidth: number;
  sidePanelOpen: boolean;
}

const ViewportDetector: React.FC = () => {
  const gzWebPanelRef = useRef<HTMLDivElement>(null);
  const sidePanelRef = useRef<HTMLDivElement>(null);
  const animationFrameRef = useRef<number>();
  const lastViewportRef = useRef<ViewSubRect | null>(null);

  useEffect(() => {
    const updateViewportSize = () => {
      if (!gzWebPanelRef.current) return;

      const gzWebRect = gzWebPanelRef.current.getBoundingClientRect();
      const fullWidth = gzWebRect.width;
      const fullHeight = gzWebRect.height;

      // Get actual side panel width (including during animation)
      let sidePanelWidth = 0;
      if (sidePanelRef.current) {
        const sidePanelRect = sidePanelRef.current.getBoundingClientRect();
        // For right-side panel, calculate width based on position
        sidePanelWidth = Math.max(0, gzWebRect.right - sidePanelRect.left);
      }

      const offsetX = sidePanelWidth;
      const offsetY = 0;
      const width = fullWidth - offsetX;
      const height = fullHeight;

      const newViewport: ViewSubRect = {
        fullWidth,
        fullHeight,
        offsetX,
        offsetY,
        width,
        height
      };

      // Only send update if viewport has actually changed
      if (!lastViewportRef.current || 
          JSON.stringify(lastViewportRef.current) !== JSON.stringify(newViewport)) {
        lastViewportRef.current = newViewport;
        
        window.postMessage({
          type: 'VIEWPORT_SIZE_UPDATE',
          id: 'esim-viewport-detector',
          payload: newViewport
        }, '*');
      }

      // Continue animation loop for real-time updates during animations
      animationFrameRef.current = requestAnimationFrame(updateViewportSize);
    };

    // Start the animation loop
    animationFrameRef.current = requestAnimationFrame(updateViewportSize);

    // Initial update
    updateViewportSize();

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, []);

  return (
    <>
      <div 
        ref={gzWebPanelRef}
        style={{ 
          position: 'absolute', 
          top: 0, 
          left: 0, 
          right: 0, 
          bottom: 0,
          pointerEvents: 'none' // Don't interfere with GzWebPanel interactions
        }}
      />
      <div 
        ref={sidePanelRef}
        style={{ 
          position: 'absolute', 
          top: 0, 
          right: 0, 
          bottom: 0,
          pointerEvents: 'none' // Don't interfere with SidePanel interactions
        }}
      />
    </>
  );
};

export const ESimViewPanel: React.FC = () => {
  const [sidePanelWidth, setSidePanelWidth] = useState(320);
  const [sidePanelOpen, setSidePanelOpen] = useState(true);

  // Monitor side panel state changes
  useEffect(() => {
    // Listen for side panel toggle events
    const handleSidePanelToggle = (event: CustomEvent) => {
      setSidePanelOpen(event.detail.open);
    };

    window.addEventListener('sidepanel-toggle' as any, handleSidePanelToggle as EventListener);

    return () => {
      window.removeEventListener('sidepanel-toggle' as any, handleSidePanelToggle as EventListener);
    };
  }, []);

  return (
    <div className="esim-wrapper">
      <div className="esim-main-view">
        <GzWebPanel />
        <ViewportDetector />
      </div>
      <SidePanel side="right">
        <FeaturedEntitiesPanel />
      </SidePanel>
      {/* Self-contained EntityInfoPopup - renders when window message is received */}
      <EntityInfoPopup />
    </div>
  );
};
