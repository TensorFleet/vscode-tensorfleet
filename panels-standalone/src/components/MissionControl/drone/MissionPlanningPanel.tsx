import React from 'react';

type FlightPlanRecord = {
  id: string;
  name: string;
  path: [number, number][];
};

type MissionPlanningPanelProps = {
  flightPlans: FlightPlanRecord[];
  selectedFlightPlanId: string | null;
  onStartNewPlan: () => void;
  onSelectFlightPlan: (flightPlanId: string) => void;
  onDeleteFlightPlan: (flightPlanId: string) => void;
};

function TrashIcon() {
  return (
    <svg
      className="mission-side-panel__delete-icon"
      xmlns="http://www.w3.org/2000/svg"
      viewBox="0 0 24 24"
      fill="none"
      aria-hidden="true"
      focusable="false"
    >
      <path d="M5 7h14" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" />
      <path d="M9 7V5.75C9 5.34 9.34 5 9.75 5h4.5c.41 0 .75.34.75.75V7" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" />
      <path d="M8 9.5v7.25c0 .69.56 1.25 1.25 1.25h5.5c.69 0 1.25-.56 1.25-1.25V9.5" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" />
      <path d="M10.5 11v4.5" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" />
      <path d="M13.5 11v4.5" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" />
    </svg>
  );
}

export function MissionPlanningPanel({
  flightPlans,
  selectedFlightPlanId,
  onStartNewPlan,
  onSelectFlightPlan,
  onDeleteFlightPlan,
}: MissionPlanningPanelProps) {
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

        <div className="mission-side-panel__missions">
          {flightPlans.map((flightPlan) => {
            const isSelected = flightPlan.id === selectedFlightPlanId;
            return (
              <div
                key={flightPlan.id}
                className={`mission-side-panel__mission-row${isSelected ? ' mission-side-panel__mission-row--selected' : ''}`}
              >
                <div
                  role="button"
                  tabIndex={0}
                  className={`mission-side-panel__mission-button${isSelected ? ' mission-side-panel__mission-button--selected' : ''}`}
                  onClick={() => onSelectFlightPlan(flightPlan.id)}
                  onKeyDown={(event) => {
                    if (event.key === 'Enter' || event.key === ' ') {
                      event.preventDefault();
                      onSelectFlightPlan(flightPlan.id);
                    }
                  }}
                >
                  <span className="mission-side-panel__mission-name">{flightPlan.name}</span>
                </div>
                <button
                  type="button"
                  className="mission-side-panel__delete-button"
                  onClick={(event) => {
                    event.stopPropagation();
                    onDeleteFlightPlan(flightPlan.id);
                  }}
                  aria-label={`Delete ${flightPlan.name}`}
                >
                  <TrashIcon />
                </button>
              </div>
            );
          })}
        </div>
      </div>
    </div>
  );
}
