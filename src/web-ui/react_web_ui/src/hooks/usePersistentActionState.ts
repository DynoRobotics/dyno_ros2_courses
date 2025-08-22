import { useState, useEffect, useCallback, useRef } from 'react';
import { layoutService, type ActionManagerPersistentState } from '../services/layoutService';
import type { ActionInfo } from '../services/actionService';

interface UsePersistentActionStateProps {
    panelId?: string;
    layoutId?: string;
    actions: ActionInfo[];
    defaultNamespace?: string;
}

interface UsePersistentActionStateReturn {
    selectedAction: string;
    selectedNamespace: string;
    goalValues: Record<string, any>;
    setSelectedAction: (action: string) => void;
    setSelectedNamespace: (namespace: string) => void;
    setGoalValues: (values: Record<string, any> | ((prev: Record<string, any>) => Record<string, any>)) => void;
    clearState: () => void;
}

// Debounce utility
function useDebounce<T extends (...args: any[]) => void>(callback: T, delay: number): T {
    const timeoutRef = useRef<NodeJS.Timeout>();

    return useCallback((...args: Parameters<T>) => {
        if (timeoutRef.current) {
            clearTimeout(timeoutRef.current);
        }
        timeoutRef.current = setTimeout(() => callback(...args), delay);
    }, [callback, delay]) as T;
}

export function usePersistentActionState({
    panelId,
    layoutId,
    actions,
    defaultNamespace
}: UsePersistentActionStateProps): UsePersistentActionStateReturn {
    const [selectedAction, setSelectedActionState] = useState<string>('');
    const [selectedNamespace, setSelectedNamespaceState] = useState<string>('');
    const [goalValues, setGoalValuesState] = useState<Record<string, any>>({});

    const isInitialized = useRef(false);

    // Get available namespaces and actions for validation
    const getAvailableNamespaces = useCallback((): string[] => {
        const namespaces = new Set<string>();
        actions.forEach(action => {
            if (action.namespace) {
                namespaces.add(action.namespace);
            } else {
                namespaces.add(''); // Empty namespace for root actions
            }
        });
        return Array.from(namespaces).sort();
    }, [actions]);

    const getAvailableActions = useCallback((): string[] => {
        return actions.map(action => action.name);
    }, [actions]);

    // Debounced save function to avoid excessive localStorage writes
    const debouncedSave = useDebounce((state: Partial<ActionManagerPersistentState>) => {
        if (panelId && layoutId && isInitialized.current) {
            layoutService.updatePanelState(layoutId, panelId, state);
        }
    }, 500);

    // Load persistent state when component mounts or when actions change
    useEffect(() => {
        if (!panelId || !layoutId || actions.length === 0) return;

        const persistentState = layoutService.getPanelState(layoutId, panelId);

        if (persistentState) {
            // Validate the persistent state against current available actions/namespaces
            const validatedState = layoutService.validatePersistentState(
                persistentState,
                getAvailableActions(),
                getAvailableNamespaces()
            );

            // Apply validated state
            if (validatedState.selectedNamespace !== undefined) {
                setSelectedNamespaceState(validatedState.selectedNamespace);
            } else if (defaultNamespace) {
                // Fall back to default namespace if no persistent state
                const availableNamespaces = getAvailableNamespaces();
                if (availableNamespaces.includes(defaultNamespace)) {
                    setSelectedNamespaceState(defaultNamespace);
                }
            }

            if (validatedState.selectedAction) {
                setSelectedActionState(validatedState.selectedAction);
            }

            if (validatedState.goalValues) {
                setGoalValuesState(validatedState.goalValues);
            }

            // If state was modified during validation, save the cleaned version
            if (JSON.stringify(validatedState) !== JSON.stringify(persistentState)) {
                layoutService.updatePanelState(layoutId, panelId, validatedState);
            }
        } else if (defaultNamespace) {
            // No persistent state, apply default namespace if available
            const availableNamespaces = getAvailableNamespaces();
            if (availableNamespaces.includes(defaultNamespace)) {
                setSelectedNamespaceState(defaultNamespace);
            }
        }

        isInitialized.current = true;
    }, [panelId, layoutId, actions, defaultNamespace, getAvailableActions, getAvailableNamespaces]);

    // Wrapped setters that also save to persistent storage
    const setSelectedAction = useCallback((action: string) => {
        setSelectedActionState(action);

        // Reset goal values when action changes
        setGoalValuesState({});

        // Save to persistent storage
        debouncedSave({
            selectedAction: action,
            goalValues: {}
        });
    }, [debouncedSave]);

    const setSelectedNamespace = useCallback((namespace: string) => {
        setSelectedNamespaceState(namespace);

        // Reset selected action and goal values when namespace changes
        setSelectedActionState('');
        setGoalValuesState({});

        // Save to persistent storage
        debouncedSave({
            selectedNamespace: namespace,
            selectedAction: '',
            goalValues: {}
        });
    }, [debouncedSave]);

    const setGoalValues = useCallback((values: Record<string, any> | ((prev: Record<string, any>) => Record<string, any>)) => {
        const newValues = typeof values === 'function' ? values(goalValues) : values;
        setGoalValuesState(newValues);

        // Save to persistent storage
        debouncedSave({
            goalValues: newValues
        });
    }, [goalValues, debouncedSave]);

    const clearState = useCallback(() => {
        setSelectedActionState('');
        setSelectedNamespaceState('');
        setGoalValuesState({});

        if (panelId && layoutId) {
            layoutService.clearPanelState(layoutId, panelId);
        }
    }, [panelId, layoutId]);

    return {
        selectedAction,
        selectedNamespace,
        goalValues,
        setSelectedAction,
        setSelectedNamespace,
        setGoalValues,
        clearState
    };
}
