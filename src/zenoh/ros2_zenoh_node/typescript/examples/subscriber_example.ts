/**
 * Example: Subscribing to Twist messages with automatic deserialization
 */

import { Node, Twist, serializeTwistCDR, deserializeTwistCDR } from '../src';

async function main() {
  console.log('Starting TypeScript ROS 2 Zenoh Subscriber');

  // Create a node
  const node = new Node('ts_subscriber', '/');
  console.log(`Node created: ${node.getName()} in namespace ${node.getNamespace()}`);

  // Create a typed subscriber with callback
  const subscriber = node.createSubscriber<Twist>(
    '/turtle1/cmd_vel',
    'geometry_msgs/msg/Twist',
    {
      serialize: serializeTwistCDR,
      deserialize: deserializeTwistCDR,
    },
    (msg) => {
      // Callback receives the fully deserialized TypeScript object
      console.log(`Received Twist: linear.x=${msg.linear.x.toFixed(2)}, angular.z=${msg.angular.z.toFixed(2)}`);
    }
  );

  console.log('Subscriber waiting for messages on /turtle1/cmd_vel...');
  console.log('Press Ctrl+C to exit');

  // Keep the process alive
  await new Promise(() => {});
}

main().catch(console.error);

