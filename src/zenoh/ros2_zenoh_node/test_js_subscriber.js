// Test JavaScript subscriber receiving from Rust publisher
const native = require('./index.js');

console.log('JavaScript Subscriber starting...\n');

const node = new native.NativeNode('js_sub', '/');
console.log('✓ Node created:', node.getName());

let messagesReceived = 0;
const subscriber = node.createRawSubscriber(
  '/turtle1/cmd_vel',
  'geometry_msgs/msg/Twist',
  (err, buffer) => {
    if (err) {
      console.error('✗ Error:', err);
      return;
    }
    
    messagesReceived++;
    console.log(`✓ Received message #${messagesReceived}: ${buffer.length} bytes`);
    console.log('  First 20 bytes:', buffer.slice(0, 20).toString('hex'));
    
    if (messagesReceived >= 5) {
      console.log('\n✓ Received 5 messages successfully!');
      process.exit(0);
    }
  }
);

console.log('✓ Subscriber created and waiting for messages...');
console.log('Press Ctrl+C to exit\n');

// Timeout after 30 seconds
setTimeout(() => {
  if (messagesReceived === 0) {
    console.error('✗ No messages received after 30 seconds');
    process.exit(1);
  }
}, 30000);


