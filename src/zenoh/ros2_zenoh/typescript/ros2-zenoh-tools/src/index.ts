/**
 * @ros2-zenoh/tools
 * 
 * Documentation and validation tools for ros2-zenoh
 */

export * from './types.js';
export * from './id-derivation.js';
export * from './frontmatter-validation.js';
export * from './document-parser.js';
export * from './naming-conventions.js';

import { glob } from 'glob';
import type { ValidationError } from './types.js';
import { parseDocument, shouldValidateFile } from './document-parser.js';
import { validateFrontmatter } from './frontmatter-validation.js';

/**
 * Validate all documents in a directory
 */
export async function validateAllDocuments(docsDir: string): Promise<ValidationError[]> {
  const allErrors: ValidationError[] = [];
  
  // Find all markdown files
  const files = await glob(`${docsDir}/**/*.md`, {
    ignore: ['**/node_modules/**', '**/dist/**', '**/.git/**']
  });
  
  for (const file of files) {
    // Skip files that don't need validation
    if (!shouldValidateFile(file)) {
      continue;
    }
    
    try {
      // Parse document
      const doc = await parseDocument(file);
      
      // Validate frontmatter
      const errors = validateFrontmatter(doc.frontmatter, file);
      allErrors.push(...errors);
      
    } catch (error) {
      allErrors.push({
        file,
        message: `Failed to parse: ${error instanceof Error ? error.message : String(error)}`,
        severity: 'error'
      });
    }
  }
  
  return allErrors;
}

/**
 * Format validation errors for display
 */
export function formatErrors(errors: ValidationError[]): string {
  if (errors.length === 0) {
    return '✅ All documents valid!';
  }
  
  const errorCount = errors.filter(e => e.severity === 'error').length;
  const warningCount = errors.filter(e => e.severity === 'warning').length;
  
  let output = '';
  
  // Group by file
  const byFile = new Map<string, ValidationError[]>();
  for (const error of errors) {
    if (!byFile.has(error.file)) {
      byFile.set(error.file, []);
    }
    byFile.get(error.file)!.push(error);
  }
  
  // Format each file's errors
  for (const [file, fileErrors] of byFile.entries()) {
    output += `\n❌ ${file}:\n`;
    for (const error of fileErrors) {
      const icon = error.severity === 'error' ? '  ✗' : '  ⚠';
      const field = error.field ? ` [${error.field}]` : '';
      output += `${icon}${field} ${error.message}\n`;
    }
  }
  
  output += `\n📊 Summary: ${errorCount} errors, ${warningCount} warnings\n`;
  
  return output;
}

