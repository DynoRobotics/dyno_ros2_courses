import React, { createContext, useContext, useEffect, useState, useCallback, useRef } from 'react';

export interface WebSocketMessage {
    type: string;
    data: any;
    timestamp: number;
}

export type ConnectionState = 'connecting' | 'connected' | 'disconnected' | 'error';

interface WebSocketContextType {
    connectionState: ConnectionState;
    subscribe: (eventType: string, handler: (data: any) => void) => void;
    unsubscribe: (eventType: string, handler: (data: any) => void) => void;
    isConnected: boolean;
}

const WebSocketContext = createContext<WebSocketContextType | null>(null);

export const useWebSocket = () => {
    const context = useContext(WebSocketContext);
    if (!context) {
        throw new Error('useWebSocket must be used within a WebSocketProvider');
    }
    return context;
};

interface WebSocketProviderProps {
    children: React.ReactNode;
}

export const WebSocketProvider: React.FC<WebSocketProviderProps> = ({ children }) => {
    const [connectionState, setConnectionState] = useState<ConnectionState>('disconnected');
    const wsRef = useRef<WebSocket | null>(null);
    const subscriptionsRef = useRef<Map<string, Set<(data: any) => void>>>(new Map());
    const reconnectAttemptsRef = useRef(0);
    const reconnectTimerRef = useRef<number | null>(null);
    const maxReconnectAttempts = 5;
    const baseReconnectDelay = 1000; // 1 second
    const maxReconnectDelay = 30000; // 30 seconds

    // Get WebSocket URL from environment or default
    const getWebSocketUrl = useCallback(() => {
        const baseUrl = import.meta.env.VITE_API_BASE_URL || 'http://localhost:5183';
        const wsUrl = baseUrl.replace(/^http/, 'ws');
        return `${wsUrl}/ws/events`;
    }, []);

    // Send subscription request to server
    const sendSubscriptionRequest = useCallback((eventTypes: string[]) => {
        if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
            wsRef.current.send(JSON.stringify({
                action: 'subscribe',
                event_types: eventTypes
            }));
        }
    }, []);

    // Send unsubscription request to server
    const sendUnsubscriptionRequest = useCallback((eventTypes: string[]) => {
        if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
            wsRef.current.send(JSON.stringify({
                action: 'unsubscribe',
                event_types: eventTypes
            }));
        }
    }, []);

    // Handle incoming WebSocket messages
    const handleMessage = useCallback((message: WebSocketMessage) => {
        const handlers = subscriptionsRef.current.get(message.type);
        if (handlers) {
            handlers.forEach(handler => {
                try {
                    handler(message.data);
                } catch (error) {
                    console.error(`Error in WebSocket message handler for ${message.type}:`, error);
                }
            });
        }
    }, []);

    // Resubscribe to all event types after reconnection
    const resubscribeAll = useCallback(() => {
        const allEventTypes = Array.from(subscriptionsRef.current.keys());
        if (allEventTypes.length > 0) {
            sendSubscriptionRequest(allEventTypes);
        }
    }, [sendSubscriptionRequest]);

    // Connect to WebSocket
    const connect = useCallback(() => {
        if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
            return;
        }

        setConnectionState('connecting');

        try {
            const wsUrl = getWebSocketUrl();
            wsRef.current = new WebSocket(wsUrl);

            wsRef.current.onopen = () => {
                console.log('WebSocket connected');
                setConnectionState('connected');
                reconnectAttemptsRef.current = 0;
                resubscribeAll();
            };

            wsRef.current.onmessage = (event) => {
                try {
                    const message: WebSocketMessage = JSON.parse(event.data);
                    handleMessage(message);
                } catch (error) {
                    console.error('Failed to parse WebSocket message:', error);
                }
            };

            wsRef.current.onclose = (event) => {
                console.log('WebSocket disconnected:', event.code, event.reason);
                setConnectionState('disconnected');
                wsRef.current = null;

                // Attempt to reconnect unless it was a clean close
                if (event.code !== 1000) {
                    scheduleReconnect();
                }
            };

            wsRef.current.onerror = (error) => {
                console.error('WebSocket error:', error);
                setConnectionState('error');
            };

        } catch (error) {
            console.error('Failed to create WebSocket connection:', error);
            setConnectionState('error');
        }
    }, [getWebSocketUrl, handleMessage, resubscribeAll]);

    // Schedule reconnection with exponential backoff
    const scheduleReconnect = useCallback(() => {
        if (reconnectAttemptsRef.current >= maxReconnectAttempts) {
            console.error('Max reconnection attempts reached');
            setConnectionState('error');
            return;
        }

        const delay = Math.min(
            baseReconnectDelay * Math.pow(2, reconnectAttemptsRef.current),
            maxReconnectDelay
        );

        console.log(`Scheduling reconnection attempt ${reconnectAttemptsRef.current + 1} in ${delay}ms`);

        reconnectTimerRef.current = setTimeout(() => {
            reconnectAttemptsRef.current++;
            connect();
        }, delay);
    }, [connect]);

    // Disconnect from WebSocket
    const disconnect = useCallback(() => {
        if (reconnectTimerRef.current) {
            clearTimeout(reconnectTimerRef.current);
            reconnectTimerRef.current = null;
        }

        if (wsRef.current) {
            wsRef.current.close(1000, 'Client disconnect');
            wsRef.current = null;
        }

        setConnectionState('disconnected');
    }, []);

    // Subscribe to event type
    const subscribe = useCallback((eventType: string, handler: (data: any) => void) => {
        if (!subscriptionsRef.current.has(eventType)) {
            subscriptionsRef.current.set(eventType, new Set());
        }

        const handlers = subscriptionsRef.current.get(eventType)!;
        const wasEmpty = handlers.size === 0;
        handlers.add(handler);

        // Send subscription request to server if this is the first handler for this event type
        if (wasEmpty) {
            sendSubscriptionRequest([eventType]);
        }
    }, [sendSubscriptionRequest]);

    // Unsubscribe from event type
    const unsubscribe = useCallback((eventType: string, handler: (data: any) => void) => {
        const handlers = subscriptionsRef.current.get(eventType);
        if (handlers) {
            handlers.delete(handler);

            // If no more handlers for this event type, unsubscribe from server
            if (handlers.size === 0) {
                subscriptionsRef.current.delete(eventType);
                sendUnsubscriptionRequest([eventType]);
            }
        }
    }, [sendUnsubscriptionRequest]);

    // Check if connected
    const isConnected = connectionState === 'connected';

    // Connect on mount, disconnect on unmount
    useEffect(() => {
        connect();

        return () => {
            disconnect();
        };
    }, [connect, disconnect]);

    const contextValue: WebSocketContextType = {
        connectionState,
        subscribe,
        unsubscribe,
        isConnected,
    };

    return (
        <WebSocketContext.Provider value={contextValue}>
            {children}
        </WebSocketContext.Provider>
    );
};
