/**
 * High-level ROS 2 Publisher with automatic CDR serialization
 */
import { SerializerFunctions } from './types';
export declare class Publisher<T> {
    private nativePublisher;
    private serializer;
    constructor(nativePublisher: any, serializer: SerializerFunctions<T>);
    /**
     * Publish a message (automatically serializes to CDR)
     */
    publish(message: T): void;
    /**
     * Get the topic this publisher is publishing to
     */
    getTopic(): string;
}
