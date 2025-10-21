/**
 * High-level ROS 2 Node with typed publishers and subscribers
 */

import { Publisher } from './Publisher';
import { Subscriber } from './Subscriber';
import { SerializerFunctions } from './types';

// Import native bindings
const native = require('@ros2-zenoh/native');

export class Node {
  private nativeNode: any;

  constructor(name: string, namespace: string = '/') {
    this.nativeNode = new native.NativeNode(name, namespace);
  }

  /**
   * Get the node name
   */
  public getName(): string {
    return this.nativeNode.getName();
  }

  /**
   * Get the node namespace
   */
  public getNamespace(): string {
    return this.nativeNode.getNamespace();
  }

  /**
   * Create a typed publisher with automatic CDR serialization
   */
  public createPublisher<T>(
    topic: string,
    msgType: string,
    serializer: SerializerFunctions<T>
  ): Publisher<T> {
    const nativePublisher = this.nativeNode.createRawPublisher(topic, msgType);
    return new Publisher(nativePublisher, serializer);
  }

  /**
   * Create a typed subscriber with automatic CDR deserialization
   */
  public createSubscriber<T>(
    topic: string,
    msgType: string,
    serializer: SerializerFunctions<T>,
    callback: (message: T) => void
  ): Subscriber<T> {
    // Wrap the user's callback to deserialize the data
    // NAPI uses Node.js convention: (err, value)
    const wrappedCallback = (err: Error | null, buffer: Buffer) => {
      if (err) {
        console.error(`Error in subscriber callback for ${topic}:`, err);
        return;
      }
      
      try {
        const message = serializer.deserialize(new Uint8Array(buffer));
        callback(message);
      } catch (error) {
        console.error(`Failed to deserialize message on ${topic}:`, error);
      }
    };

    const nativeSubscriber = this.nativeNode.createRawSubscriber(
      topic,
      msgType,
      wrappedCallback
    );
    
    return new Subscriber(nativeSubscriber);
  }
}

