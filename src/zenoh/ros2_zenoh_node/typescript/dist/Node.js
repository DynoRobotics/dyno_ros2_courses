"use strict";
/**
 * High-level ROS 2 Node with typed publishers and subscribers
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.Node = void 0;
const Publisher_1 = require("./Publisher");
const Subscriber_1 = require("./Subscriber");
// Import native bindings
const native = require('@ros2-zenoh/native');
class Node {
    constructor(name, namespace = '/') {
        this.nativeNode = new native.NativeNode(name, namespace);
    }
    /**
     * Get the node name
     */
    getName() {
        return this.nativeNode.getName();
    }
    /**
     * Get the node namespace
     */
    getNamespace() {
        return this.nativeNode.getNamespace();
    }
    /**
     * Create a typed publisher with automatic CDR serialization
     */
    createPublisher(topic, msgType, serializer) {
        const nativePublisher = this.nativeNode.createRawPublisher(topic, msgType);
        return new Publisher_1.Publisher(nativePublisher, serializer);
    }
    /**
     * Create a typed subscriber with automatic CDR deserialization
     */
    createSubscriber(topic, msgType, serializer, callback) {
        // Wrap the user's callback to deserialize the data
        // NAPI uses Node.js convention: (err, value)
        const wrappedCallback = (err, buffer) => {
            if (err) {
                console.error(`Error in subscriber callback for ${topic}:`, err);
                return;
            }
            try {
                const message = serializer.deserialize(new Uint8Array(buffer));
                callback(message);
            }
            catch (error) {
                console.error(`Failed to deserialize message on ${topic}:`, error);
            }
        };
        const nativeSubscriber = this.nativeNode.createRawSubscriber(topic, msgType, wrappedCallback);
        return new Subscriber_1.Subscriber(nativeSubscriber);
    }
}
exports.Node = Node;
