/**
 * High-level ROS 2 Node with typed publishers and subscribers
 */
import { Publisher } from './Publisher';
import { Subscriber } from './Subscriber';
import { SerializerFunctions } from './types';
export declare class Node {
    private nativeNode;
    constructor(name: string, namespace?: string);
    /**
     * Get the node name
     */
    getName(): string;
    /**
     * Get the node namespace
     */
    getNamespace(): string;
    /**
     * Create a typed publisher with automatic CDR serialization
     */
    createPublisher<T>(topic: string, msgType: string, serializer: SerializerFunctions<T>): Publisher<T>;
    /**
     * Create a typed subscriber with automatic CDR deserialization
     */
    createSubscriber<T>(topic: string, msgType: string, serializer: SerializerFunctions<T>, callback: (message: T) => void): Subscriber<T>;
}
