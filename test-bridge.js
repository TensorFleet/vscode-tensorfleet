// Test script to verify MCP Bridge is working
const net = require('net');
const path = require('path');
const os = require('os');

const socketPath = path.join(os.tmpdir(), 'tensorfleet-mcp-bridge.sock');

console.log('Testing MCP Bridge connection...');
console.log(`Socket path: ${socketPath}`);

const client = net.createConnection(socketPath, () => {
  console.log('✓ Connected to MCP Bridge');
  
  const testCommand = {
    command: 'openGazeboPanel',
    params: { world: 'empty', model: 'iris' }
  };
  
  console.log('\nSending command:', JSON.stringify(testCommand, null, 2));
  client.write(JSON.stringify(testCommand));
});

client.on('data', (data) => {
  const response = JSON.parse(data.toString());
  console.log('\n✓ Response received:', JSON.stringify(response, null, 2));
  
  if (response.success) {
    console.log('\n✅ SUCCESS! Gazebo panel should be opening in VS Code');
  } else {
    console.log('\n❌ FAILED:', response.error);
  }
  
  client.end();
  process.exit(response.success ? 0 : 1);
});

client.on('error', (error) => {
  console.error('\n❌ Connection error:', error.message);
  console.log('\nMake sure:');
  console.log('1. VS Code is running');
  console.log('2. TensorFleet extension is activated');
  console.log('3. MCP Bridge has started');
  process.exit(1);
});

client.setTimeout(2000, () => {
  console.error('\n❌ Connection timeout');
  client.end();
  process.exit(1);
});

