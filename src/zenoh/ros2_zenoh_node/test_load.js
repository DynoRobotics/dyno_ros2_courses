// Simple test to verify the NAPI module loads
try {
  const native = require('./index.js');
  console.log('✓ Native module loaded successfully');
  console.log('Exports:', Object.keys(native));
  
  // Try creating a node
  const node = new native.NativeNode('test_node', '/');
  console.log('✓ Node created:', node.getName());
  
  process.exit(0);
} catch (error) {
  console.error('✗ Error loading native module:', error.message);
  console.error(error.stack);
  process.exit(1);
}


