/**
 * Example: Publishing Twist messages with automatic serialization
 */

import { Node, Twist, serializeTwistCDR, deserializeTwistCDR } from '../src';

async function main() {
  console.log('Starting TypeScript ROS 2 Zenoh Publisher');

  // Create a node
  const node = new Node('ts_publisher', '/');
  console.log(`Node created: ${node.getName()} in namespace ${node.getNamespace()}`);

  // Create a typed publisher
  const publisher = node.createPublisher<Twist>(
    '/turtle1/cmd_vel',
    'geometry_msgs/msg/Twist',
    {
      serialize: serializeTwistCDR,
      deserialize: deserializeTwistCDR,
    }
  );

  console.log(`Publisher created for topic: ${publisher.getTopic()}`);

  // Publish messages
  console.log('Publishing Twist messages...');
  for (let i = 0; i < 10; i++) {
    const twist: Twist = {
      linear: { x: 1.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.5 },
    };

    // Just pass the object directly!
    publisher.publish(twist);
    console.log(`Published message ${i}: linear.x=${twist.linear.x}, angular.z=${twist.angular.z}`);

    await new Promise(resolve => setTimeout(resolve, 1000));
  }

  console.log('Publisher example completed');
}

main().catch(console.error);

