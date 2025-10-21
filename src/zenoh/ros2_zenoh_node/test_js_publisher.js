// Test JavaScript publisher sending to Rust subscriber
const native = require('./index.js');

console.log('JavaScript Publisher starting...\n');

const node = new native.NativeNode('js_pub', '/');
console.log('✓ Node created:', node.getName());

const publisher = node.createRawPublisher('/turtle1/cmd_vel', 'geometry_msgs/msg/Twist');
console.log('✓ Publisher created');

// Create a Twist message in CDR format
// Twist: linear (Vector3) + angular (Vector3) = 6 doubles = 48 bytes + 4 byte header
const twistCDR = Buffer.from([
  0x00, 0x01, 0x00, 0x00,  // CDR header (little endian)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x3f,  // linear.x = 1.0
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // linear.y = 0.0
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // linear.z = 0.0
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // angular.x = 0.0
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // angular.y = 0.0
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x3f   // angular.z = 0.5
]);

console.log('\nPublishing 10 Twist messages...');
for (let i = 0; i < 10; i++) {
  publisher.publishRaw(twistCDR);
  console.log(`Published message ${i}: linear.x=1.0, angular.z=0.5`);
  
  // Sleep 1 second
  const start = Date.now();
  while (Date.now() - start < 1000) {}
}

console.log('\n✓ Publisher example completed');
process.exit(0);


