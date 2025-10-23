/**
 * Derive canonical IDs from frontmatter
 * 
 * IDs are derived from structured frontmatter fields (type, name, number, variant)
 * and follow the format: namespace:identifier
 * 
 * Examples:
 *   type: spec, name: clock → spec:clock
 *   type: spec, name: clock, variant: python → spec:clock-python
 *   type: adr, number: "002" → adr:002
 *   type: pattern, name: fixed-rate-loop → pattern:fixed-rate-loop
 */

import type { Frontmatter, SpecFrontmatter, ADRFrontmatter, PatternFrontmatter, ConventionFrontmatter } from './types.js';

/**
 * Derive canonical ID from frontmatter
 */
export function deriveCanonicalId(frontmatter: Frontmatter): string {
  switch (frontmatter.type) {
    case 'spec':
      return deriveSpecId(frontmatter as SpecFrontmatter);
    
    case 'adr':
      return deriveADRId(frontmatter as ADRFrontmatter);
    
    case 'pattern':
      return derivePatternId(frontmatter as PatternFrontmatter);
    
    case 'convention':
      return deriveConventionId(frontmatter as ConventionFrontmatter);
    
    case 'tutorial':
    case 'howto':
    case 'concept':
    case 'index':
    case 'reference':
    case 'api-reference':
      // These types don't have standardized IDs yet
      // Use type:name if name exists
      if ('name' in frontmatter && frontmatter.name) {
        return `${frontmatter.type}:${frontmatter.name}`;
      }
      return frontmatter.type;
    
    default:
      throw new Error(`Unknown document type: ${(frontmatter as any).type}`);
  }
}

/**
 * Derive ID for spec
 * 
 * Format: spec:name or spec:name-variant
 * 
 * Examples:
 *   name: clock → spec:clock
 *   name: clock, variant: python → spec:clock-python
 *   name: rate → spec:rate
 */
function deriveSpecId(spec: SpecFrontmatter): string {
  if (!spec.name) {
    throw new Error('Spec missing required field: name');
  }
  
  const baseName = spec.name.toLowerCase();
  
  // If variant is specified and not 'core', append it
  if (spec.variant && spec.variant !== 'core') {
    return `spec:${baseName}-${spec.variant}`;
  }
  
  return `spec:${baseName}`;
}

/**
 * Derive ID for ADR
 * 
 * Format: adr:number
 * 
 * Examples:
 *   number: "001" → adr:001
 *   number: "002" → adr:002
 */
function deriveADRId(adr: ADRFrontmatter): string {
  if (!adr.number) {
    throw new Error('ADR missing required field: number');
  }
  
  // Ensure number is zero-padded to 3 digits
  const paddedNumber = adr.number.padStart(3, '0');
  
  return `adr:${paddedNumber}`;
}

/**
 * Derive ID for pattern
 * 
 * Format: pattern:name
 * 
 * Examples:
 *   name: fixed-rate-loop → pattern:fixed-rate-loop
 */
function derivePatternId(pattern: PatternFrontmatter): string {
  if (!pattern.name) {
    throw new Error('Pattern missing required field: name');
  }
  
  return `pattern:${pattern.name.toLowerCase()}`;
}

/**
 * Derive ID for convention
 * 
 * Format: convention:name
 * 
 * Examples:
 *   name: pausable-component-api → convention:pausable-component-api
 */
function deriveConventionId(convention: ConventionFrontmatter): string {
  if (!convention.name) {
    throw new Error('Convention missing required field: name');
  }
  
  return `convention:${convention.name.toLowerCase()}`;
}

/**
 * Extract namespace from canonical ID
 */
export function extractNamespace(id: string): string {
  const parts = id.split(':');
  if (parts.length < 2) {
    throw new Error(`Invalid ID format: ${id} (expected namespace:name)`);
  }
  return parts[0];
}

/**
 * Extract name from canonical ID
 */
export function extractName(id: string): string {
  const parts = id.split(':');
  if (parts.length < 2) {
    throw new Error(`Invalid ID format: ${id} (expected namespace:name)`);
  }
  return parts.slice(1).join(':');  // Handle IDs with multiple colons
}

/**
 * Validate ID format
 */
export function isValidId(id: string): boolean {
  // ID must be namespace:name format
  if (!id.includes(':')) {
    return false;
  }
  
  const parts = id.split(':');
  if (parts.length < 2) {
    return false;
  }
  
  const [namespace, name] = parts;
  
  // Namespace must be lowercase
  if (namespace !== namespace.toLowerCase()) {
    return false;
  }
  
  // Name must be lowercase with hyphens or numbers
  if (!/^[a-z0-9-]+$/.test(name)) {
    return false;
  }
  
  return true;
}

