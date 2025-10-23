/**
 * Parse markdown documents and extract frontmatter
 */

import { readFile } from 'fs/promises';
import { parse as parseYAML } from 'yaml';
import matter from 'gray-matter';
import type { Frontmatter, Document } from './types.js';
import { deriveCanonicalId } from './id-derivation.js';

/**
 * Parse a markdown document
 */
export async function parseDocument(filePath: string): Promise<Document> {
  const fileContent = await readFile(filePath, 'utf-8');
  
  // Extract frontmatter using gray-matter
  const { data: frontmatter, content } = matter(fileContent);
  
  // Derive canonical ID
  const derivedId = deriveCanonicalId(frontmatter as Frontmatter);
  
  return {
    filePath,
    frontmatter: frontmatter as Frontmatter,
    content,
    derivedId
  };
}

/**
 * Check if file should be validated (based on path)
 */
export function shouldValidateFile(filePath: string): boolean {
  // Validate files in specific directories
  const validatePaths = [
    '/06-Specs/',
    '/05-Patterns/',
    '/03-Conventions/',
    '/09-Architecture-Decision-Records/'
  ];
  
  // Skip template files
  if (filePath.includes('/templates/') || filePath.includes('/.obsidian/templates/')) {
    return false;
  }
  
  // Check if in validation path
  return validatePaths.some(path => filePath.includes(path));
}

