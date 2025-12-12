#!/usr/bin/env node
/**
 * Check runtime compatibility for TensorFleet Drone JS Project
 * 
 * This script verifies that your JavaScript runtime supports the modern
 * syntax features used in this project (optional chaining, etc.)
 */

const MIN_NODE_VERSION = 14;

function checkRuntime() {
    const isBun = typeof Bun !== 'undefined';

    // Check if user tried to run with bun but node is executing
    const commandLine = process.env.npm_lifecycle_event || process.env._ || '';
    const likelyTriedBun = commandLine.includes('bun') || process.argv.some(arg => arg.includes('bun'));

    if (isBun) {
        console.log('✅ Running on Bun - all features supported!');
        console.log(`   Version: ${Bun.version}`);
        return true;
    }

    console.log('⚠️  Running on Node.js');

    const nodeVersion = process.version;
    const majorVersion = parseInt(nodeVersion.slice(1).split('.')[0], 10);

    console.log(`   Version: ${nodeVersion}`);

    // Warn if they tried to use bun but node is running
    if (likelyTriedBun && majorVersion < MIN_NODE_VERSION) {
        console.error(`\n⚠️  WARNING: You may have tried to run with 'bun' but Node.js is executing!`);
        console.error(`   This can happen if:`);
        console.error(`   - Bun is not installed or not in PATH`);
        console.error(`   - Script shebang forces Node.js`);
        console.error(`   - npm/npx is intercepting the command`);
    }

    if (majorVersion < MIN_NODE_VERSION) {
        console.error(`\n❌ ERROR: Node.js ${nodeVersion} is too old!`);
        console.error(`   This project requires Node.js v${MIN_NODE_VERSION}.0.0 or higher.`);
        console.error(`\n   You'll see errors like: "SyntaxError: Unexpected token '.'"`);
        console.error(`\n   Solutions:`);
        console.error(`   1. Install Bun (recommended): curl -fsSL https://bun.sh/install | bash`);
        console.error(`   2. Upgrade Node.js: https://nodejs.org/`);
        return false;
    }

    console.log(`✅ Node.js version is compatible (>= v${MIN_NODE_VERSION}.0.0)`);
    console.log(`\n💡 Tip: Consider switching to Bun for better performance!`);
    console.log(`   Install: curl -fsSL https://bun.sh/install | bash`);
    return true;
}

function testOptionalChaining() {
    try {
        // Test optional chaining
        const obj = { a: { b: 42 } };
        const result = obj?.a?.b;

        if (result === 42) {
            console.log('✅ Optional chaining (?.) is supported');
            return true;
        }
    } catch (err) {
        console.error('❌ Optional chaining (?.) is NOT supported');
        console.error(`   Error: ${err.message}`);
        return false;
    }
    return false;
}

console.log('🔍 TensorFleet Drone JS - Runtime Compatibility Check\n');
console.log('='.repeat(60));

const runtimeOk = checkRuntime();
console.log('='.repeat(60));

if (runtimeOk) {
    console.log('\n🧪 Testing modern JavaScript features...\n');
    const featuresOk = testOptionalChaining();

    if (featuresOk) {
        console.log('\n✅ All checks passed! You\'re ready to run the project.\n');
        process.exit(0);
    } else {
        console.log('\n❌ Feature test failed. Please upgrade your runtime.\n');
        process.exit(1);
    }
} else {
    console.log('\n❌ Runtime check failed. Please upgrade to continue.\n');
    process.exit(1);
}
