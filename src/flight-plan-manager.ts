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

class FlightPlanManager {
  private plans: FlightPlanData[] = [];
  private listeners: Map<FlightPlanEvent, Set<() => void>> = new Map();

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

  subscribe(event: FlightPlanEvent, callback: () => void): () => void {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, new Set());
    }
    this.listeners.get(event)!.add(callback);
    
    // Return unsubscribe function
    return () => {
      this.listeners.get(event)?.delete(callback);
    };
  }

  private emit(event: FlightPlanEvent, data?: any): void {
    const callbacks = this.listeners.get(event);
    if (callbacks) {
      callbacks.forEach(callback => callback());
    }
  }

  private generateId(): string {
    return Math.random().toString(36).substr(2, 9);
  }
}

// Initialize window.flightPlanner if it doesn't exist
declare global {
  interface Window {
    flightPlanner?: {
      add: (planData: Omit<FlightPlanData, 'id' | 'createdAt'>) => FlightPlanData;
      remove: (id: string) => boolean;
      list: () => FlightPlanData[];
      subscribe: (event: FlightPlanEvent, callback: () => void) => () => void;
    };
  }
}

const globalWindow = globalThis as unknown as Window;

if (!globalWindow.flightPlanner) {
  const manager = new FlightPlanManager();
  globalWindow.flightPlanner = {
    add: (planData: Omit<FlightPlanData, 'id' | 'createdAt'>) => {
      return manager.add(planData);
    },
    remove: (id: string) => {
      return manager.remove(id);
    },
    list: () => {
      return manager.list();
    },
    subscribe: (event: FlightPlanEvent, callback: () => void) => {
      return manager.subscribe(event, callback);
    }
  };
}
