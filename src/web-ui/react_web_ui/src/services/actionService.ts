// API service for ROS2 actions
export interface ActionGoalField {
    name: string;
    type: string;
    description: string;
}

export interface ActionInfo {
    name: string;
    type: string;
    description: string;
    goal: ActionGoalField[];
    available: boolean;
    namespace: string;
}

export interface ActionListResponse {
    actions: ActionInfo[];
    total_count: number;
}

export interface ActionSendGoalRequest {
    parameters: Record<string, any>;
}

export interface ActionSendGoalResponse {
    success: boolean;
    message: string;
    goal_id?: string;
    accepted?: boolean;
    rejection_reason?: string;
}

export interface ActionCancelGoalResponse {
    success: boolean;
    message: string;
    goal_id: string;
}

export interface GoalStatus {
    goal_id: string;
    action_name: string;
    action_type: string;
    status: string;
    error?: string;
    feedback?: Record<string, string>;
}

export interface ActiveGoalsResponse {
    active_goals: GoalStatus[];
    total_count: number;
}

class ActionService {
    private baseUrl: string;

    constructor() {
        // Default to localhost:5173, can be configured via environment variables
        this.baseUrl = import.meta.env.VITE_API_BASE_URL || 'http://localhost:5173';
    }

    /**
     * Fetch all available actions from the backend
     */
    async getActions(): Promise<ActionListResponse> {
        try {
            const response = await fetch(`${this.baseUrl}/api/actions`);

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data: ActionListResponse = await response.json();
            return data;
        } catch (error) {
            console.error('Error fetching actions:', error);
            throw new Error(`Failed to fetch actions: ${error instanceof Error ? error.message : 'Unknown error'}`);
        }
    }

    /**
     * Get details for a specific action
     */
    async getAction(actionName: string): Promise<ActionInfo> {
        try {
            const response = await fetch(`${this.baseUrl}/api/actions/${actionName}`);

            if (!response.ok) {
                if (response.status === 404) {
                    throw new Error(`Action '${actionName}' not found`);
                }
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data: ActionInfo = await response.json();
            return data;
        } catch (error) {
            console.error(`Error fetching action ${actionName}:`, error);
            throw new Error(`Failed to fetch action: ${error instanceof Error ? error.message : 'Unknown error'}`);
        }
    }

    /**
     * Send goal to an action with specified parameters
     */
    async sendGoal(actionName: string, parameters: Record<string, any>): Promise<ActionSendGoalResponse> {
        try {
            const response = await fetch(`${this.baseUrl}/api/actions/${actionName}/send_goal`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ parameters }),
            });

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data: ActionSendGoalResponse = await response.json();
            return data;
        } catch (error) {
            console.error(`Error sending goal to action ${actionName}:`, error);
            throw new Error(`Failed to send goal: ${error instanceof Error ? error.message : 'Unknown error'}`);
        }
    }

    /**
     * Cancel a running goal
     */
    async cancelGoal(goalId: string): Promise<ActionCancelGoalResponse> {
        try {
            const response = await fetch(`${this.baseUrl}/api/goals/${goalId}/cancel`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
            });

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data: ActionCancelGoalResponse = await response.json();
            return data;
        } catch (error) {
            console.error(`Error cancelling goal ${goalId}:`, error);
            throw new Error(`Failed to cancel goal: ${error instanceof Error ? error.message : 'Unknown error'}`);
        }
    }

    /**
     * Get status of a specific goal
     */
    async getGoalStatus(goalId: string): Promise<GoalStatus> {
        try {
            const response = await fetch(`${this.baseUrl}/api/goals/${goalId}/status`);

            if (!response.ok) {
                if (response.status === 404) {
                    throw new Error(`Goal '${goalId}' not found`);
                }
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data: GoalStatus = await response.json();
            return data;
        } catch (error) {
            console.error(`Error fetching goal status ${goalId}:`, error);
            throw new Error(`Failed to fetch goal status: ${error instanceof Error ? error.message : 'Unknown error'}`);
        }
    }

    /**
     * Get all active goals
     */
    async getActiveGoals(): Promise<ActiveGoalsResponse> {
        try {
            const response = await fetch(`${this.baseUrl}/api/goals/active`);

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data: ActiveGoalsResponse = await response.json();
            return data;
        } catch (error) {
            console.error('Error fetching active goals:', error);
            throw new Error(`Failed to fetch active goals: ${error instanceof Error ? error.message : 'Unknown error'}`);
        }
    }

    /**
     * Check if the backend is healthy
     */
    async healthCheck(): Promise<boolean> {
        try {
            const response = await fetch(`${this.baseUrl}/`);
            return response.ok;
        } catch (error) {
            console.error('Health check failed:', error);
            return false;
        }
    }
}

// Export a singleton instance
export const actionService = new ActionService();
export default ActionService;
