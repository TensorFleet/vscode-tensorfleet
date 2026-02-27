export interface FlightPlanData {
  id: string;
  name: string;
  plan: {
    points: { lon: number; lat: number }[];
    closed: boolean;
  };
  createdAt: Date;
}

class FlightPlanManager {
  private plans: FlightPlanData[] = [];

  add(planData: Omit<FlightPlanData, 'id' | 'createdAt'>): FlightPlanData {
    const newPlan: FlightPlanData = {
      ...planData,
      id: this.generateId(),
      createdAt: new Date()
    };
    this.plans.push(newPlan);
    return newPlan;
  }

  remove(id: string): boolean {
    const index = this.plans.findIndex(plan => plan.id === id);
    if (index !== -1) {
      this.plans.splice(index, 1);
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
    };
  }
}

const globalWindow = globalThis as unknown as Window;

if (!globalWindow.flightPlanner) {
  globalWindow.flightPlanner = {
    add: (planData: Omit<FlightPlanData, 'id' | 'createdAt'>) => {
      const manager = new FlightPlanManager();
      return manager.add(planData);
    },
    remove: (id: string) => {
      const manager = new FlightPlanManager();
      return manager.remove(id);
    },
    list: () => {
      const manager = new FlightPlanManager();
      return manager.list();
    }
  };
}
