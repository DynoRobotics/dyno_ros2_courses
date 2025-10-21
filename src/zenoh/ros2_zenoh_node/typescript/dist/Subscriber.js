"use strict";
/**
 * High-level ROS 2 Subscriber with automatic CDR deserialization
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.Subscriber = void 0;
class Subscriber {
    constructor(nativeSubscriber) {
        this.nativeSubscriber = nativeSubscriber;
    }
}
exports.Subscriber = Subscriber;
