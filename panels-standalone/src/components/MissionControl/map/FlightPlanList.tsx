import React from 'react';

interface FlightPlanData {
  id: string;
  name: string;
  plan: {
    points: { lon: number; lat: number }[];
    closed: boolean;
  };
  createdAt: Date;
}

const FlightPlanList: React.FC = () => {
  const flightPlans = (window as any).flightPlanner?.list() || [];

  const formatDateTime = (date: Date) => {
    return date.toLocaleString();
  };

  return (
    <div style={{ padding: '16px', height: '100%', overflow: 'auto' }}>
      <h3 style={{ margin: '0 0 16px 0', fontSize: '16px', fontWeight: '600' }}>Flight Plans</h3>
      {flightPlans.length === 0 ? (
        <div style={{ color: '#666', fontSize: '14px', textAlign: 'center', marginTop: '40px' }}>
          No flight plans yet
        </div>
      ) : (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '12px' }}>
          {flightPlans.map((plan: FlightPlanData) => (
            <div
              key={plan.id}
              style={{
                padding: '12px',
                border: '1px solid #e0e0e0',
                borderRadius: '8px',
                backgroundColor: '#fafafa',
                cursor: 'default'
              }}
            >
              <div style={{ fontWeight: '600', fontSize: '14px', marginBottom: '4px' }}>
                {plan.name}
              </div>
              <div style={{ fontSize: '12px', color: '#666', marginBottom: '8px' }}>
                Created: {formatDateTime(plan.createdAt)}
              </div>
              <div style={{ fontSize: '12px', color: '#666' }}>
                Points: {plan.plan.points.length}
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default FlightPlanList;