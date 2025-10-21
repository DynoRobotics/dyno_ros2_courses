// Test publisher and subscriber with raw bytes
const native = require('./index.js');

console.log('Testing ROS 2 Zenoh Native Bindings...\n');

// Create nodes
const pubNode = new native.NativeNode('js_pub', '/');
const subNode = new native.NativeNode('js_sub', '/');
console.log('✓ Nodes created');

// Create subscriber first
let messagesReceived = 0;
const subscriber = subNode.createRawSubscriber(
  '/test_topic',
  'std_msgs/msg/String',
  (err, buffer) => {
    if (err) {
      console.error('✗ Error in callback:', err);
      return;
    }
    
    messagesReceived++;
    console.log(`✓ Subscriber received ${buffer.length} bytes (message #${messagesReceived})`);
    
    if (messagesReceived >= 3) {
      console.log('\n✓ All messages received successfully!');
      process.exit(0);
    }
  }
);
console.log('✓ Subscriber created');

// Create publisher
const publisher = pubNode.createRawPublisher('/test_topic', 'std_msgs/msg/String');
console.log('✓ Publisher created\n');

// Give subscriber time to initialize
setTimeout(() => {
  console.log('Publishing messages...');
  
  // Publish some test data (just dummy bytes for this test)
  const testData = Buffer.from([0, 1, 0, 0, 5, 0, 0, 0, 72, 101, 108, 108, 111]); // CDR: "Hello"
  
  publisher.publishRaw(testData);
  console.log('Published message 1');
  
  setTimeout(() => {
    publisher.publishRaw(testData);
    console.log('Published message 2');
  }, 500);
  
  setTimeout(() => {
    publisher.publishRaw(testData);
    console.log('Published message 3');
  }, 1000);
  
  // Timeout if no messages received
  setTimeout(() => {
    if (messagesReceived === 0) {
      console.error('\n✗ No messages received after 3 seconds');
      process.exit(1);
    }
  }, 3000);
}, 1000);

