// Test CDR serialization/deserialization directly
const { serializeTwistCDR, deserializeTwistCDR } = require('@ros2-cdr/interfaces-ts');

console.log('Testing TypeScript CDR functions...\n');

const twist = {
  linear: { x: 2.5, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: 1.2 }
};

console.log('Original Twist:', JSON.stringify(twist, null, 2));

// Serialize
const bytes = serializeTwistCDR(twist);
console.log('\nSerialized to', bytes.length, 'bytes');
console.log('First 20 bytes:', Buffer.from(bytes.slice(0, 20)).toString('hex'));

// Deserialize
const deserialized = deserializeTwistCDR(bytes);
console.log('\nDeserialized Twist:', JSON.stringify(deserialized, null, 2));

if (deserialized.linear.x === twist.linear.x && 
    deserialized.angular.z === twist.angular.z) {
  console.log('\n✓ CDR serialization/deserialization works correctly!');
} else {
  console.log('\n✗ Values do not match!');
}


