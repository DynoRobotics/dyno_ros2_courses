#!/usr/bin/env node
/**
 * CLI tool for validating ros2-zenoh documentation
 * 
 * Usage:
 *   npx @ros2-zenoh/tools validate
 *   ros2-zenoh-tools validate
 */

import { resolve } from 'path';
import { validateAllDocuments, formatErrors } from '../src/index.js';

async function main() {
  const args = process.argv.slice(2);
  
  // Default to docs/ directory
  const docsDir = args[0] || resolve(process.cwd(), 'docs');
  
  console.log(`🔍 Validating documents in: ${docsDir}\n`);
  
  try {
    // Run validation
    const errors = await validateAllDocuments(docsDir);
    
    // Format and display results
    const output = formatErrors(errors);
    console.log(output);
    
    // Exit with error code if validation failed
    const hasErrors = errors.some(e => e.severity === 'error');
    process.exit(hasErrors ? 1 : 0);
    
  } catch (error) {
    console.error('❌ Validation failed:');
    console.error(error);
    process.exit(1);
  }
}

main();

