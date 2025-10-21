"use strict";
/**
 * @ros2-zenoh/node - High-level TypeScript API for ROS 2 over Zenoh
 *
 * This package provides an ergonomic TypeScript API for ROS 2 communication
 * over Zenoh, with automatic CDR serialization/deserialization.
 */
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __exportStar = (this && this.__exportStar) || function(m, exports) {
    for (var p in m) if (p !== "default" && !Object.prototype.hasOwnProperty.call(exports, p)) __createBinding(exports, m, p);
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.Subscriber = exports.Publisher = exports.Node = void 0;
var Node_1 = require("./Node");
Object.defineProperty(exports, "Node", { enumerable: true, get: function () { return Node_1.Node; } });
var Publisher_1 = require("./Publisher");
Object.defineProperty(exports, "Publisher", { enumerable: true, get: function () { return Publisher_1.Publisher; } });
var Subscriber_1 = require("./Subscriber");
Object.defineProperty(exports, "Subscriber", { enumerable: true, get: function () { return Subscriber_1.Subscriber; } });
// Re-export common message types from the CDR interfaces package
__exportStar(require("@ros2-cdr/interfaces-ts"), exports);
