"use strict";
/**
 * High-level ROS 2 Publisher with automatic CDR serialization
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.Publisher = void 0;
// Import native bindings (will be properly typed once we generate them)
const native = require('@ros2-zenoh/native');
class Publisher {
    constructor(nativePublisher, serializer) {
        this.nativePublisher = nativePublisher;
        this.serializer = serializer;
    }
    /**
     * Publish a message (automatically serializes to CDR)
     */
    publish(message) {
        const cdrBytes = this.serializer.serialize(message);
        this.nativePublisher.publishRaw(Buffer.from(cdrBytes));
    }
    /**
     * Get the topic this publisher is publishing to
     */
    getTopic() {
        return this.nativePublisher.getTopic();
    }
}
exports.Publisher = Publisher;
