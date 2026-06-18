import React from 'react';

type FlightPlanRecord = {
  id: string;
  name: string;
  path: [number, number][];
};

type MissionPlanningPanelProps = {
  flightPlans: FlightPlanRecord[];
  selectedFlightPlanId: string | null;
  ongoingMissionLabel: string;
  hasOngoingMission: boolean;
  onStartNewPlan: () => void;
  onSelectFlightPlan: (flightPlanId: string) => void;
  onSendFlightPlan: (flightPlanId: string) => void;
  onDeleteFlightPlan: (flightPlanId: string) => void;
  onStopOngoingMission: () => void;
};

function PlayIcon() {
  return (
    <svg
      className="mission-side-panel__action-icon"
      xmlns="http://www.w3.org/2000/svg"
      viewBox="0 0 24 24"
      fill="none"
      aria-hidden="true"
      focusable="false"
    >
      <path d="M8 6.75v10.5a.75.75 0 0 0 1.15.64l8.12-5.25a.75.75 0 0 0 0-1.28L9.15 6.11A.75.75 0 0 0 8 6.75Z" fill="currentColor" />
    </svg>
  );
}

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

function StopIcon() {
  return (
    <svg
      className="mission-side-panel__action-icon"
      xmlns="http://www.w3.org/2000/svg"
      viewBox="0 0 24 24"
      fill="none"
      aria-hidden="true"
      focusable="false"
    >
      <rect x="7" y="7" width="10" height="10" rx="1.5" fill="currentColor" />
    </svg>
  );
}

export function MissionPlanningPanel({
  flightPlans,
  selectedFlightPlanId,
  ongoingMissionLabel,
  hasOngoingMission,
  onStartNewPlan,
  onSelectFlightPlan,
  onSendFlightPlan,
  onDeleteFlightPlan,
  onStopOngoingMission,
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
          <div className="mission-side-panel__mission-row">
            <div className="mission-side-panel__mission-button">
              <span className="mission-side-panel__mission-name">
                {ongoingMissionLabel}
              </span>
            </div>
            {hasOngoingMission ? (
              <div className="mission-side-panel__mission-actions">
                <button
                  type="button"
                  className="mission-side-panel__icon-button"
                  onClick={onStopOngoingMission}
                  aria-label="Stop ongoing mission"
                  title="Stop ongoing mission"
                >
                  <StopIcon />
                </button>
              </div>
            ) : null}
          </div>
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
                <div className="mission-side-panel__mission-actions">
                  <button
                    type="button"
                    className="mission-side-panel__icon-button"
                    onClick={(event) => {
                      event.stopPropagation();
                      onSendFlightPlan(flightPlan.id);
                    }}
                    aria-label={`Send ${flightPlan.name}`}
                    title={`Send ${flightPlan.name}`}
                    disabled={flightPlan.path.length === 0}
                  >
                    <PlayIcon />
                  </button>
                  <button
                    type="button"
                    className="mission-side-panel__icon-button"
                    onClick={(event) => {
                      event.stopPropagation();
                      onDeleteFlightPlan(flightPlan.id);
                    }}
                    aria-label={`Delete ${flightPlan.name}`}
                    title={`Delete ${flightPlan.name}`}
                  >
                    <TrashIcon />
                  </button>
                </div>
              </div>
            );
          })}
        </div>
      </div>
    </div>
  );
}
