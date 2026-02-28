import React, { createContext, useContext, useEffect, useState, useCallback, ReactNode } from 'react';

export interface FlightPlanData {
  id: string;
  name: string;
  plan: {
    points: { lon: number; lat: number }[];
    closed: boolean;
  };
  createdAt: Date;
}

type FlightPlanEvent = 'planAdded' | 'planRemoved' | 'planUpdated' | 'listChanged';

interface FlightPlanContextType {
  plans: FlightPlanData[];
  addPlan: (planData: Omit<FlightPlanData, 'id' | 'createdAt'>) => FlightPlanData;
  removePlan: (id: string) => boolean;
  getPlan: (id: string) => FlightPlanData | undefined;
  subscribe: (event: FlightPlanEvent, callback: () => void) => () => void;
}

const FlightPlanContext = createContext<FlightPlanContextType | undefined>(undefined);

export const useFlightPlans = () => {
  const context = useContext(FlightPlanContext);
  if (!context) {
    throw new Error('useFlightPlans must be used within a FlightPlanProvider');
  }
  return context;
};

interface FlightPlanProviderProps {
  children: ReactNode;
}

export const FlightPlanProvider: React.FC<FlightPlanProviderProps> = ({ children }) => {
  const [plans, setPlans] = useState<FlightPlanData[]>([]);
  
  // Initialize with existing plans from global flightPlanner
  useEffect(() => {
    console.log('FlightPlanContext: Initializing with existing plans');
    
    // Ensure flightPlanner is available in browser environment
    if (!(window as any).flightPlanner) {
      console.log('FlightPlanContext: flightPlanner not found, initializing...');
      initializeFlightPlanner();
    }
    
    const initialPlans = (window as any).flightPlanner?.list() || [];
    console.log('FlightPlanContext: Initial plans from global flightPlanner:', initialPlans);
    setPlans(initialPlans);
  }, []);

  // Initialize flightPlanner in browser environment
  const initializeFlightPlanner = () => {
    console.log('FlightPlanContext: Initializing flightPlanner in browser');
    
    // Create the FlightPlanManager class inline since we can't import from Node.js
    class FlightPlanManager {
      private plans: FlightPlanData[] = [];
      private listeners: Map<string, Set<() => void>> = new Map();

      add(planData: Omit<FlightPlanData, 'id' | 'createdAt'>): FlightPlanData {
        const newPlan: FlightPlanData = {
          ...planData,
          id: this.generateId(),
          createdAt: new Date()
        };
        this.plans.push(newPlan);
        this.emit('planAdded', newPlan);
        this.emit('listChanged');
        return newPlan;
      }

      remove(id: string): boolean {
        const index = this.plans.findIndex(plan => plan.id === id);
        if (index !== -1) {
          const removedPlan = this.plans.splice(index, 1)[0];
          this.emit('planRemoved', removedPlan);
          this.emit('listChanged');
          return true;
        }
        return false;
      }

      list(): FlightPlanData[] {
        return [...this.plans];
      }

      get(id: string): FlightPlanData | undefined {
        return this.plans.find(plan => plan.id === id);
      }

      subscribe(event: string, callback: () => void): () => void {
        if (!this.listeners.has(event)) {
          this.listeners.set(event, new Set());
        }
        this.listeners.get(event)!.add(callback);
        
        return () => {
          this.listeners.get(event)?.delete(callback);
        };
      }

      private emit(event: string, data?: any): void {
        const callbacks = this.listeners.get(event);
        if (callbacks) {
          callbacks.forEach(callback => callback());
        }
      }

      private generateId(): string {
        return Math.random().toString(36).substr(2, 9);
      }
    }

    const manager = new FlightPlanManager();
    (window as any).flightPlanner = {
      add: (planData: Omit<FlightPlanData, 'id' | 'createdAt'>) => {
        console.log('FlightPlanContext: flightPlanner.add called with:', planData);
        const result = manager.add(planData);
        console.log('FlightPlanContext: flightPlanner.add returning:', result);
        return result;
      },
      remove: (id: string) => {
        return manager.remove(id);
      },
      list: () => {
        return manager.list();
      },
      subscribe: (event: string, callback: () => void) => {
        return manager.subscribe(event, callback);
      }
    };
  };

  const addPlan = useCallback((planData: Omit<FlightPlanData, 'id' | 'createdAt'>) => {
    console.log('FlightPlanContext: addPlan called with:', planData);
    const newPlan = (window as any).flightPlanner?.add(planData);
    console.log('FlightPlanContext: flightPlanner.add returned:', newPlan);
    if (newPlan) {
      console.log('FlightPlanContext: Adding new plan to state, previous plans count:', plans.length);
      setPlans(prev => {
        const newPlans = [...prev, newPlan];
        console.log('FlightPlanContext: Updated plans state:', newPlans);
        return newPlans;
      });
    } else {
      console.log('FlightPlanContext: flightPlanner.add returned falsy value');
    }
    return newPlan;
  }, [plans.length]);

  const removePlan = useCallback((id: string) => {
    const success = (window as any).flightPlanner?.remove(id);
    if (success) {
      setPlans(prev => prev.filter(plan => plan.id !== id));
    }
    return success;
  }, []);

  const getPlan = useCallback((id: string) => {
    return (window as any).flightPlanner?.get(id);
  }, []);

  const subscribe = useCallback((event: FlightPlanEvent, callback: () => void) => {
    return (window as any).flightPlanner?.subscribe(event, callback);
  }, []);

  const value = {
    plans,
    addPlan,
    removePlan,
    getPlan,
    subscribe
  };

  return (
    <FlightPlanContext.Provider value={value}>
      {children}
    </FlightPlanContext.Provider>
  );
};