/**
 * High-level ROS 2 Subscriber with automatic CDR deserialization
 */

import { SerializerFunctions } from './types';

export class Subscriber<T> {
  private nativeSubscriber: any;

  constructor(nativeSubscriber: any) {
    this.nativeSubscriber = nativeSubscriber;
  }

  // The native subscriber already has the callback set up
  // Nothing more needed here since the callback is bound at construction
}

