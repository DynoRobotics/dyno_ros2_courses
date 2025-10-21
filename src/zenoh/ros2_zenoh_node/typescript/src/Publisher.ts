/**
 * High-level ROS 2 Publisher with automatic CDR serialization
 */

import { SerializerFunctions } from './types';

// Import native bindings (will be properly typed once we generate them)
const native = require('@ros2-zenoh/native');

export class Publisher<T> {
  private nativePublisher: any;
  private serializer: SerializerFunctions<T>;

  constructor(
    nativePublisher: any,
    serializer: SerializerFunctions<T>
  ) {
    this.nativePublisher = nativePublisher;
    this.serializer = serializer;
  }

  /**
   * Publish a message (automatically serializes to CDR)
   */
  public publish(message: T): void {
    const cdrBytes = this.serializer.serialize(message);
    this.nativePublisher.publishRaw(Buffer.from(cdrBytes));
  }

  /**
   * Get the topic this publisher is publishing to
   */
  public getTopic(): string {
    return this.nativePublisher.getTopic();
  }
}

