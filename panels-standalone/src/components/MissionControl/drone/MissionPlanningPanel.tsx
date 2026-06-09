import React from 'react';

type MissionPlanningPanelProps = {
  onStartNewPlan: () => void;
};

export function MissionPlanningPanel({ onStartNewPlan }: MissionPlanningPanelProps) {
  return (
    <div className="mission-side-panel">
      <div className="mission-side-panel__header">
        <div className="mission-side-panel__title">Mission Planning</div>
      </div>

      <div className="mission-side-panel__body">
        <button
          type="button"
          className="mission-side-panel__action"
          onClick={onStartNewPlan}
        >
          New plan +
        </button>
      </div>
    </div>
  );
}
