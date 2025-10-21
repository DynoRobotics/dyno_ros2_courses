// Test the high-level TypeScript API (after compilation)
const { Node } = require('./dist/index');
const { serializeTwistCDR, deserializeTwistCDR } = require('@ros2-cdr/interfaces-ts');

console.log('Testing High-Level TypeScript API...\n');

// Create nodes
const pubNode = new Node('ts_pub', '/');
const subNode = new Node('ts_sub', '/');
console.log('✓ Nodes created');

// Create typed subscriber
let messagesReceived = 0;
const subscriber = subNode.createSubscriber(
  '/test_topic',
  'geometry_msgs/msg/Twist',
  { serialize: serializeTwistCDR, deserialize: deserializeTwistCDR },
  (msg) => {
    messagesReceived++;
    console.log('Received message:', JSON.stringify(msg, null, 2));
    
    if (msg && msg.linear && msg.angular) {
      console.log(`✓ Received Twist #${messagesReceived}:`, 
        `linear.x=${msg.linear.x.toFixed(2)}, angular.z=${msg.angular.z.toFixed(2)}`);
    } else {
      console.log('✗ Invalid message structure:', msg);
    }
    
    if (messagesReceived >= 3) {
      console.log('\n✓ High-level API test successful!');
      process.exit(0);
    }
  }
);
console.log('✓ Subscriber created');

// Create typed publisher
const publisher = pubNode.createPublisher(
  '/test_topic',
  'geometry_msgs/msg/Twist',
  { serialize: serializeTwistCDR, deserialize: deserializeTwistCDR }
);
console.log('✓ Publisher created\n');

// Give subscriber time to initialize
setTimeout(() => {
  console.log('Publishing messages with high-level API...');
  
  // Just pass the object directly!
  const twist = {
    linear: { x: 2.5, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 1.2 }
  };
  
  publisher.publish(twist);
  console.log('Published message 1');
  
  setTimeout(() => {
    publisher.publish(twist);
    console.log('Published message 2');
  }, 500);
  
  setTimeout(() => {
    publisher.publish(twist);
    console.log('Published message 3');
  }, 1000);
  
  setTimeout(() => {
    if (messagesReceived === 0) {
      console.error('\n✗ No messages received');
      process.exit(1);
    }
  }, 3000);
}, 1000);

