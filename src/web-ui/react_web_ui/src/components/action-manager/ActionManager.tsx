import React, { useState, useEffect, useCallback } from 'react';
import {
    Box,
    Select,
    FormLabel,
    VStack,
    Text,
    Card,
    CardHeader,
    CardBody,
    Heading,
    Spinner,
    Alert,
    AlertIcon,
    Badge,
    HStack,
    Input,
    NumberInput,
    NumberInputField,
    NumberInputStepper,
    NumberIncrementStepper,
    NumberDecrementStepper,
    Checkbox,
    Button,
    Divider,
    useToast,
} from '@chakra-ui/react';
import { actionService } from '../../services/actionService';
import type { ActionInfo, ActionGoalField, GoalStatus } from '../../services/actionService';
import { usePersistentActionState } from '../../hooks/usePersistentActionState';
import { useWebSocket } from '../../contexts/WebSocketContext';

export interface ActionManagerProps {
    id?: string;
    layoutId?: string;
    title?: string;
    defaultNamespace?: string;
    allowedActions?: string[];
    onActionSelect?: (action: string) => void;
}

const ActionManager: React.FC<ActionManagerProps> = ({
    id,
    layoutId,
    title = 'Action Manager',
    defaultNamespace,
    allowedActions,
    onActionSelect
}) => {
    const [actions, setActions] = useState<ActionInfo[]>([]);
    const [loading, setLoading] = useState<boolean>(true);
    const [error, setError] = useState<string | null>(null);
    const [sendingGoal, setSendingGoal] = useState<boolean>(false);
    const [activeGoals, setActiveGoals] = useState<GoalStatus[]>([]);
    const [cancellingGoals, setCancellingGoals] = useState<Set<string>>(new Set());
    const toast = useToast();

    // WebSocket connection
    const { subscribe, unsubscribe, isConnected, connectionState } = useWebSocket();

    // Use persistent state hook for action selections and goal values
    const {
        selectedAction,
        selectedNamespace,
        goalValues,
        setSelectedAction: setPersistentSelectedAction,
        setSelectedNamespace: setPersistentSelectedNamespace,
        setGoalValues: setPersistentGoalValues,
        clearState
    } = usePersistentActionState({
        panelId: id,
        layoutId,
        actions,
        defaultNamespace
    });

    // WebSocket event handlers
    const handleGoalUpdate = useCallback((data: GoalStatus) => {
        setActiveGoals(prev => {
            // Check if goal has reached a terminal state
            const terminalStates = ['completed', 'failed', 'rejected', 'error'];
            if (terminalStates.includes(data.status)) {
                // Remove goal from active list if it's in a terminal state
                return prev.filter(goal => goal.goal_id !== data.goal_id);
            }

            const existingIndex = prev.findIndex(goal => goal.goal_id === data.goal_id);
            if (existingIndex >= 0) {
                // Update existing goal
                const updated = [...prev];
                updated[existingIndex] = data;
                return updated;
            } else {
                // Add new goal (only if not in terminal state)
                return [...prev, data];
            }
        });
    }, []);

    const handleGoalAdded = useCallback((data: GoalStatus) => {
        setActiveGoals(prev => {
            // Check if goal already exists to avoid duplicates
            if (prev.some(goal => goal.goal_id === data.goal_id)) {
                return prev;
            }
            return [...prev, data];
        });
    }, []);

    const handleGoalRemoved = useCallback((data: { goal_id: string }) => {
        setActiveGoals(prev => prev.filter(goal => goal.goal_id !== data.goal_id));
    }, []);

    const handleGoalsSnapshot = useCallback((data: GoalStatus[]) => {
        setActiveGoals(data);
    }, []);

    // Fetch actions from API on component mount
    useEffect(() => {
        const fetchActions = async () => {
            try {
                setLoading(true);
                setError(null);
                const response = await actionService.getActions();
                setActions(response.actions);
            } catch (err) {
                setError(err instanceof Error ? err.message : 'Failed to fetch actions');
                console.error('Error fetching actions:', err);
            } finally {
                setLoading(false);
            }
        };

        fetchActions();

        // Fetch initial active goals via REST API
        const fetchInitialGoals = async () => {
            try {
                const response = await actionService.getActiveGoals();
                setActiveGoals(response.active_goals);
            } catch (err) {
                console.error('Error fetching initial active goals:', err);
            }
        };

        fetchInitialGoals();
    }, []);

    // Set up WebSocket subscriptions
    useEffect(() => {
        // Subscribe to goal events
        subscribe('goal_update', handleGoalUpdate);
        subscribe('goal_added', handleGoalAdded);
        subscribe('goal_removed', handleGoalRemoved);
        subscribe('goals_snapshot', handleGoalsSnapshot);

        return () => {
            // Unsubscribe when component unmounts
            unsubscribe('goal_update', handleGoalUpdate);
            unsubscribe('goal_added', handleGoalAdded);
            unsubscribe('goal_removed', handleGoalRemoved);
            unsubscribe('goals_snapshot', handleGoalsSnapshot);
        };
    }, [subscribe, unsubscribe, handleGoalUpdate, handleGoalAdded, handleGoalRemoved, handleGoalsSnapshot]);

    const handleCancelGoal = async (goalId: string) => {
        try {
            setCancellingGoals(prev => new Set(prev).add(goalId));

            const response = await actionService.cancelGoal(goalId);

            toast({
                title: 'Goal cancelled',
                description: response.message,
                status: 'success',
                duration: 3000,
                isClosable: true,
            });

            // No need to refresh - WebSocket will handle the update
        } catch (err) {
            toast({
                title: 'Failed to cancel goal',
                description: err instanceof Error ? err.message : 'Unknown error occurred',
                status: 'error',
                duration: 5000,
                isClosable: true,
            });
        } finally {
            setCancellingGoals(prev => {
                const newSet = new Set(prev);
                newSet.delete(goalId);
                return newSet;
            });
        }
    };

    const handleActionChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
        const action = event.target.value;
        setPersistentSelectedAction(action);

        // Call the callback if provided
        if (onActionSelect) {
            onActionSelect(action);
        }
    };

    const handleNamespaceChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
        const namespace = event.target.value;
        setPersistentSelectedNamespace(namespace);
    };

    const getUniqueNamespaces = (): string[] => {
        const namespaces = new Set<string>();
        actions.forEach(action => {
            if (action.namespace) {
                namespaces.add(action.namespace);
            } else {
                namespaces.add(''); // Empty namespace for root actions
            }
        });
        return Array.from(namespaces).sort();
    };

    const getFilteredActions = (): ActionInfo[] => {
        let filteredActions = actions;

        // Filter by namespace
        if (selectedNamespace !== 'all') {
            filteredActions = filteredActions.filter(action => {
                if (selectedNamespace === '') {
                    // Show actions with no namespace (root level)
                    return !action.namespace;
                }
                return action.namespace === selectedNamespace;
            });
        }

        // Filter by allowed actions if specified
        if (allowedActions && allowedActions.length > 0) {
            filteredActions = filteredActions.filter(action =>
                allowedActions.includes(action.name)
            );
        }

        return filteredActions;
    };

    const getSelectedActionInfo = (): ActionInfo | undefined => {
        return getFilteredActions().find(action => action.name === selectedAction);
    };

    const getFilteredActiveGoals = (): GoalStatus[] => {
        // Only show goals if a specific action is selected
        if (!selectedAction) {
            return [];
        }

        return activeGoals.filter(goal => {
            // Extract namespace and action name from the goal's action_name
            // goal.action_name format: "/namespace/action_name" or "/action_name"
            const actionPath = goal.action_name.startsWith('/') ? goal.action_name.slice(1) : goal.action_name;
            const pathParts = actionPath.split('/');

            let goalNamespace = '';
            let goalActionName = '';

            if (pathParts.length === 2) {
                // Has namespace: "namespace/action_name"
                goalNamespace = pathParts[0];
                goalActionName = pathParts[1];
            } else if (pathParts.length === 1) {
                // No namespace: "action_name"
                goalNamespace = '';
                goalActionName = pathParts[0];
            } else {
                // Complex path, take last part as action name and second-to-last as namespace
                goalActionName = pathParts[pathParts.length - 1];
                goalNamespace = pathParts.length > 1 ? pathParts[pathParts.length - 2] : '';
            }

            // Must match the selected action name exactly
            if (goalActionName !== selectedAction) {
                return false;
            }

            // Must match the selected namespace exactly
            if (selectedNamespace === '') {
                // Selected "No Namespace" - only show goals with no namespace
                if (goalNamespace !== '') {
                    return false;
                }
            } else {
                // Selected specific namespace - only show goals from that exact namespace
                if (goalNamespace !== selectedNamespace) {
                    return false;
                }
            }

            return true;
        });
    };

    const handleGoalValueChange = (fieldName: string, value: any) => {
        setPersistentGoalValues(prev => ({
            ...prev,
            [fieldName]: value
        }));
    };

    const renderGoalInput = (field: ActionGoalField) => {
        const value = goalValues[field.name];

        switch (field.type) {
            case 'bool':
                return (
                    <Box>
                        <Checkbox
                            isChecked={value || false}
                            onChange={(e) => handleGoalValueChange(field.name, e.target.checked)}
                        >
                            {field.name}
                        </Checkbox>
                        {field.description && (
                            <Text fontSize="xs" color="gray.500" mt={1} ml={6}>
                                {field.description}
                            </Text>
                        )}
                    </Box>
                );

            case 'float64':
            case 'float32':
            case 'double':
                return (
                    <Box>
                        <FormLabel fontSize="sm">{field.name}</FormLabel>
                        <Input
                            type="text"
                            value={value !== undefined ? value.toString() : ''}
                            onChange={(e) => {
                                const inputValue = e.target.value;

                                // Allow empty string for clearing the field
                                if (inputValue === '') {
                                    handleGoalValueChange(field.name, '');
                                    return;
                                }

                                // Pattern that allows valid partial decimal inputs
                                // Allows: digits, single decimal point, single minus at start, single comma
                                const validPartialDecimal = /^-?(\d*\.?\d*|\.?\d*)$/.test(inputValue.replace(',', '.'));

                                if (validPartialDecimal) {
                                    // Convert comma to dot for European decimal format support
                                    const normalizedValue = inputValue.replace(',', '.');

                                    // Always store the input to show what user is typing
                                    // Only convert to number when it's a complete valid number
                                    const isCompleteNumber = /^-?\d+(\.\d+)?$/.test(normalizedValue);

                                    if (isCompleteNumber) {
                                        const numValue = parseFloat(normalizedValue);
                                        if (!isNaN(numValue) && isFinite(numValue)) {
                                            handleGoalValueChange(field.name, numValue);
                                        } else {
                                            handleGoalValueChange(field.name, normalizedValue);
                                        }
                                    } else {
                                        // Keep as string for partial inputs like "1.", "-", ".", "-.", "0.", etc.
                                        handleGoalValueChange(field.name, normalizedValue);
                                    }
                                }
                                // If input doesn't match valid decimal pattern, ignore the change
                            }}
                            placeholder={`Enter ${field.name} (e.g., 1.57, -0.5)`}
                        />
                        {field.description && (
                            <Text fontSize="xs" color="gray.500" mt={1}>
                                {field.description}
                            </Text>
                        )}
                    </Box>
                );

            case 'int32':
            case 'int64':
            case 'uint32':
            case 'uint64':
                return (
                    <Box>
                        <FormLabel fontSize="sm">{field.name}</FormLabel>
                        <NumberInput
                            value={value || ''}
                            onChange={(valueString) => handleGoalValueChange(field.name, parseInt(valueString) || 0)}
                            precision={0}
                            step={1}
                        >
                            <NumberInputField placeholder={`Enter ${field.name}`} />
                            <NumberInputStepper>
                                <NumberIncrementStepper />
                                <NumberDecrementStepper />
                            </NumberInputStepper>
                        </NumberInput>
                        {field.description && (
                            <Text fontSize="xs" color="gray.500" mt={1}>
                                {field.description}
                            </Text>
                        )}
                    </Box>
                );

            default:
                // String or unknown types
                return (
                    <Box>
                        <FormLabel fontSize="sm">{field.name}</FormLabel>
                        <Input
                            value={value || ''}
                            onChange={(e) => handleGoalValueChange(field.name, e.target.value)}
                            placeholder={`Enter ${field.name}`}
                        />
                        {field.description && (
                            <Text fontSize="xs" color="gray.500" mt={1}>
                                {field.description}
                            </Text>
                        )}
                    </Box>
                );
        }
    };

    const handleSendGoal = async () => {
        const actionInfo = getSelectedActionInfo();
        if (!actionInfo || !actionInfo.available) {
            toast({
                title: 'Action not available',
                description: 'The selected action is not currently available.',
                status: 'error',
                duration: 3000,
                isClosable: true,
            });
            return;
        }

        try {
            setSendingGoal(true);

            // Construct full action name with namespace
            const fullActionName = actionInfo.namespace
                ? `${actionInfo.namespace}/${selectedAction}`
                : selectedAction;

            const response = await actionService.sendGoal(fullActionName, goalValues);

            // Only show toast for rejected goals, not accepted ones
            if (response.accepted === false) {
                toast({
                    title: 'Goal rejected',
                    description: `${response.message}${response.rejection_reason ? ` - ${response.rejection_reason}` : ''} (Goal ID: ${response.goal_id})`,
                    status: 'warning',
                    duration: 5000,
                    isClosable: true,
                });
            } else if (response.accepted === undefined && !response.success) {
                // Fallback for older API responses without accepted field - only show if failed
                toast({
                    title: 'Goal failed',
                    description: `${response.message} (Goal ID: ${response.goal_id})`,
                    status: 'error',
                    duration: 5000,
                    isClosable: true,
                });
            }
            // No toast for accepted goals - they will appear in the active goals list
        } catch (err) {
            toast({
                title: 'Failed to send goal',
                description: err instanceof Error ? err.message : 'Unknown error occurred',
                status: 'error',
                duration: 5000,
                isClosable: true,
            });
        } finally {
            setSendingGoal(false);
        }
    };

    const handleRetry = () => {
        // Trigger a re-fetch by calling the effect again
        setError(null);
        setLoading(true);
        actionService.getActions()
            .then(response => {
                setActions(response.actions);
                setError(null);
            })
            .catch(err => {
                setError(err instanceof Error ? err.message : 'Failed to fetch actions');
            })
            .finally(() => {
                setLoading(false);
            });
    };


    return (
        <Card w="100%" h="100%" display="flex" flexDirection="column">
            <CardHeader pb={2}>
                <Heading size="sm">{title}</Heading>
            </CardHeader>

            <CardBody pt={0} flex="1">
                <VStack spacing={2} align="stretch">
                    {/* Loading State */}
                    {loading && (
                        <Box textAlign="center" py={2}>
                            <Spinner size="sm" />
                            <Text mt={1} fontSize="xs" color="gray.600">
                                Loading actions...
                            </Text>
                        </Box>
                    )}

                    {/* Error State */}
                    {error && !loading && (
                        <Alert status="error">
                            <AlertIcon />
                            <VStack align="start" spacing={1} flex="1">
                                <Text fontSize="sm">Failed to load actions</Text>
                                <Text fontSize="xs" color="gray.600">{error}</Text>
                                <Text
                                    fontSize="xs"
                                    color="blue.500"
                                    cursor="pointer"
                                    textDecoration="underline"
                                    onClick={handleRetry}
                                >
                                    Click to retry
                                </Text>
                            </VStack>
                        </Alert>
                    )}

                    {/* Namespace Selection */}
                    {!loading && !error && actions.length > 0 && (
                        <Box>
                            <FormLabel htmlFor="namespace-select">
                                <HStack>
                                    <Text>Select Namespace:</Text>
                                    <Badge colorScheme="purple" variant="subtle">
                                        {getUniqueNamespaces().length} namespaces
                                    </Badge>
                                </HStack>
                            </FormLabel>
                            <Select
                                id="namespace-select"
                                value={selectedNamespace}
                                onChange={handleNamespaceChange}
                            >
                                <option value="">No Namespace</option>
                                {getUniqueNamespaces().filter(ns => ns !== '').map((namespace) => (
                                    <option key={namespace} value={namespace}>
                                        {namespace}
                                    </option>
                                ))}
                            </Select>
                        </Box>
                    )}

                    {/* Action Selection */}
                    {!loading && !error && (
                        <Box>
                            <FormLabel htmlFor="action-select">
                                <HStack>
                                    <Text>Select Action:</Text>
                                    <Badge colorScheme="blue" variant="subtle">
                                        {getFilteredActions().length} available
                                    </Badge>
                                </HStack>
                            </FormLabel>
                            <Select
                                id="action-select"
                                value={selectedAction}
                                onChange={handleActionChange}
                                placeholder={getFilteredActions().length > 0 ? "Choose an action..." : "No actions available"}
                                disabled={getFilteredActions().length === 0}
                            >
                                {getFilteredActions().map((action) => (
                                    <option key={`${action.namespace || 'root'}-${action.name}`} value={action.name}>
                                        {action.name} ({action.type})
                                    </option>
                                ))}
                            </Select>
                        </Box>
                    )}

                    {/* Selected Action Info */}
                    {selectedAction && !loading && !error && (
                        <Box
                            p={2}
                            bg="gray.50"
                            borderRadius="md"
                            sx={{
                                _dark: {
                                    bg: "gray.700"
                                }
                            }}
                        >
                            <VStack align="start" spacing={1}>
                                <Badge
                                    colorScheme={getSelectedActionInfo()?.available ? "green" : "red"}
                                    variant="subtle"
                                    size="sm"
                                >
                                    {getSelectedActionInfo()?.available ? "Available" : "Unavailable"}
                                </Badge>
                                <Text
                                    fontSize="xs"
                                    color="gray.600"
                                    sx={{
                                        _dark: {
                                            color: "gray.300"
                                        }
                                    }}
                                >
                                    {getSelectedActionInfo()?.description}
                                </Text>
                            </VStack>
                        </Box>
                    )}

                    {/* Goal Parameters Form */}
                    {selectedAction && !loading && !error && getSelectedActionInfo()?.goal && getSelectedActionInfo()!.goal.length > 0 && (
                        <Box>
                            <Divider />
                            <VStack spacing={2} align="stretch">
                                <Text fontWeight="semibold" color="gray.700" fontSize="sm">
                                    Goal Parameters
                                </Text>
                                {getSelectedActionInfo()!.goal.map((field, index) => (
                                    <Box key={index}>
                                        {renderGoalInput(field)}
                                    </Box>
                                ))}
                                <Button
                                    colorScheme="blue"
                                    onClick={handleSendGoal}
                                    isLoading={sendingGoal}
                                    loadingText="Sending..."
                                    isDisabled={!getSelectedActionInfo()?.available}
                                    size="sm"
                                >
                                    Send Goal
                                </Button>
                            </VStack>
                        </Box>
                    )}

                    {/* Active Goals Section */}
                    {getFilteredActiveGoals().length > 0 && (
                        <Box>
                            <Divider />
                            <VStack spacing={2} align="stretch">
                                <HStack justify="space-between" align="center">
                                    <Text fontWeight="semibold" color="gray.700" fontSize="sm">
                                        Active Goals for {selectedAction}
                                    </Text>
                                    <Badge colorScheme="orange" variant="subtle">
                                        {getFilteredActiveGoals().length} running
                                    </Badge>
                                </HStack>

                                {getFilteredActiveGoals().map((goal) => (
                                    <Box
                                        key={goal.goal_id}
                                        p={2}
                                        bg="orange.50"
                                        borderRadius="md"
                                        border="1px solid"
                                        borderColor="orange.200"
                                        sx={{
                                            _dark: {
                                                bg: "orange.900",
                                                borderColor: "orange.700"
                                            }
                                        }}
                                    >
                                        <VStack align="stretch" spacing={1}>
                                            <HStack justify="space-between" align="center">
                                                <VStack align="start" spacing={0} flex="1">
                                                    <Text fontSize="xs" fontWeight="semibold">
                                                        {selectedAction}
                                                    </Text>
                                                    <Text fontSize="xs" color="gray.600">
                                                        ID: {goal.goal_id.substring(0, 8)}...
                                                    </Text>
                                                </VStack>
                                                <HStack spacing={1}>
                                                    <Badge
                                                        colorScheme={
                                                            goal.status === 'accepted' ? 'green' :
                                                                goal.status === 'pending' ? 'yellow' :
                                                                    goal.status === 'rejected' ? 'red' : 'gray'
                                                        }
                                                        size="sm"
                                                        variant="subtle"
                                                    >
                                                        {goal.status}
                                                    </Badge>
                                                    <Button
                                                        size="xs"
                                                        colorScheme="red"
                                                        variant="outline"
                                                        onClick={() => handleCancelGoal(goal.goal_id)}
                                                        isLoading={cancellingGoals.has(goal.goal_id)}
                                                        loadingText="Cancelling..."
                                                    >
                                                        Cancel
                                                    </Button>
                                                </HStack>
                                            </HStack>
                                            {goal.error && (
                                                <Text fontSize="xs" color="red.500">
                                                    Error: {goal.error}
                                                </Text>
                                            )}
                                        </VStack>
                                    </Box>
                                ))}
                            </VStack>
                        </Box>
                    )}

                    {/* Action Status */}
                    {!loading && (
                        <Box>
                            <Text fontSize="sm" color="gray.500">
                                Status: {
                                    error ? 'Error loading actions' :
                                        selectedAction ? 'Action selected' :
                                            actions.length === 0 ? 'No actions available' :
                                                'No action selected'
                                }
                            </Text>
                        </Box>
                    )}
                </VStack>
            </CardBody>
        </Card>
    );
};

export default ActionManager;
