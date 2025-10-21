/**
 * Serializer functions for ROS 2 messages
 */
export interface SerializerFunctions<T> {
    serialize: (msg: T) => Uint8Array;
    deserialize: (data: Uint8Array) => T;
}
