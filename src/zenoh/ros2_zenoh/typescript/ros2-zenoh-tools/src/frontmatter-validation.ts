/**
 * Frontmatter validation logic
 */

import type { Frontmatter, ValidationError } from './types.js';

/**
 * Validate frontmatter against schema
 */
export function validateFrontmatter(
  fm: any,  // Use 'any' since we're validating unknown input
  filePath: string
): ValidationError[] {
  const errors: ValidationError[] = [];
  
  // Required: type
  if (!fm.type) {
    errors.push({
      file: filePath,
      field: 'type',
      message: 'Missing required field: type',
      severity: 'error'
    });
    return errors; // Can't continue without type
  }
  
  // Validate type-specific required fields
  switch (fm.type) {
    case 'spec':
      validateSpec(fm, filePath, errors);
      break;
    case 'adr':
      validateADR(fm, filePath, errors);
      break;
    case 'pattern':
      validatePattern(fm, filePath, errors);
      break;
    case 'convention':
      validateConvention(fm, filePath, errors);
      break;
    case 'api-reference':
      validateAPIReference(fm, filePath, errors);
      break;
    case 'tutorial':
    case 'howto':
    case 'concept':
      validateGeneral(fm, filePath, errors);
      break;
  }
  
  // Validate universal optional fields
  validateUniversalFields(fm, filePath, errors);
  
  return errors;
}

/**
 * Validate spec frontmatter
 */
function validateSpec(
  fm: any,
  filePath: string,
  errors: ValidationError[]
): void {
  // Required fields
  if (!fm.name) {
    errors.push({
      file: filePath,
      field: 'name',
      message: 'Spec missing required field: name',
      severity: 'error'
    });
  } else {
    // Validate name format (lowercase-with-hyphens)
    if (!/^[a-z0-9]+(-[a-z0-9]+)*$/.test(fm.name)) {
      errors.push({
        file: filePath,
        field: 'name',
        message: `Name must be lowercase with hyphens: ${fm.name}`,
        severity: 'error'
      });
    }
  }
  
  if (!fm.title) {
    errors.push({
      file: filePath,
      field: 'title',
      message: 'Missing required field: title',
      severity: 'error'
    });
  }
  
  if (!fm.version) {
    errors.push({
      file: filePath,
      field: 'version',
      message: 'Missing required field: version',
      severity: 'error'
    });
  } else {
    // Validate version format (X.Y or X.Y.Z)
    if (!/^\d+\.\d+(\.\d+)?$/.test(fm.version)) {
      errors.push({
        file: filePath,
        field: 'version',
        message: `Invalid version format: ${fm.version} (use X.Y or X.Y.Z)`,
        severity: 'error'
      });
    }
  }
  
  if (!fm.status) {
    errors.push({
      file: filePath,
      field: 'status',
      message: 'Missing required field: status',
      severity: 'error'
    });
  }
}

/**
 * Validate ADR frontmatter
 */
function validateADR(
  fm: any,
  filePath: string,
  errors: ValidationError[]
): void {
  // Required fields
  if (!fm.number) {
    errors.push({
      file: filePath,
      field: 'number',
      message: 'ADR missing required field: number',
      severity: 'error'
    });
  } else {
    // Validate number format (digits only, zero-padded)
    if (!/^\d{3}$/.test(fm.number)) {
      errors.push({
        file: filePath,
        field: 'number',
        message: `ADR number should be 3 digits (e.g., "002"): ${fm.number}`,
        severity: 'warning'
      });
    }
  }
  
  if (!fm.title) {
    errors.push({
      file: filePath,
      field: 'title',
      message: 'Missing required field: title',
      severity: 'error'
    });
  }
  
  if (!fm.status) {
    errors.push({
      file: filePath,
      field: 'status',
      message: 'Missing required field: status',
      severity: 'error'
    });
  }
}

/**
 * Validate pattern frontmatter
 */
function validatePattern(
  fm: any,
  filePath: string,
  errors: ValidationError[]
): void {
  if (!fm.name) {
    errors.push({
      file: filePath,
      field: 'name',
      message: 'Pattern missing required field: name',
      severity: 'error'
    });
  }
  
  if (!fm.title) {
    errors.push({
      file: filePath,
      field: 'title',
      message: 'Missing required field: title',
      severity: 'error'
    });
  }
}

/**
 * Validate convention frontmatter
 */
function validateConvention(
  fm: any,
  filePath: string,
  errors: ValidationError[]
): void {
  if (!fm.name) {
    errors.push({
      file: filePath,
      field: 'name',
      message: 'Convention missing required field: name',
      severity: 'error'
    });
  }
  
  if (!fm.title) {
    errors.push({
      file: filePath,
      field: 'title',
      message: 'Missing required field: title',
      severity: 'error'
    });
  }
}

/**
 * Validate API reference frontmatter
 */
function validateAPIReference(
  fm: any,
  filePath: string,
  errors: ValidationError[]
): void {
  if (!fm.module) {
    errors.push({
      file: filePath,
      field: 'module',
      message: 'API reference missing required field: module',
      severity: 'error'
    });
  }
  
  if (!fm.title) {
    errors.push({
      file: filePath,
      field: 'title',
      message: 'Missing required field: title',
      severity: 'error'
    });
  }
}

/**
 * Validate general document types (tutorial, howto, concept)
 */
function validateGeneral(
  fm: any,
  filePath: string,
  errors: ValidationError[]
): void {
  if (!fm.title) {
    errors.push({
      file: filePath,
      field: 'title',
      message: 'Missing required field: title',
      severity: 'error'
    });
  }
}

/**
 * Validate universal optional fields
 */
function validateUniversalFields(
  fm: any,
  filePath: string,
  errors: ValidationError[]
): void {
  // Title length
  if (fm.title) {
    if (fm.title.length < 3 || fm.title.length > 100) {
      errors.push({
        file: filePath,
        field: 'title',
        message: `Title length should be 3-100 chars: ${fm.title.length}`,
        severity: 'warning'
      });
    }
  }
  
  // Summary length
  if (fm.summary && fm.summary.length > 120) {
    errors.push({
      file: filePath,
      field: 'summary',
      message: `Summary too long: ${fm.summary.length} chars (max 120)`,
      severity: 'warning'
    });
  }
  
  // Date formats
  if (fm.created && !/^\d{4}-\d{2}-\d{2}$/.test(fm.created)) {
    errors.push({
      file: filePath,
      field: 'created',
      message: `Invalid date format: ${fm.created} (use YYYY-MM-DD)`,
      severity: 'warning'
    });
  }
  
  if (fm.updated && !/^\d{4}-\d{2}-\d{2}$/.test(fm.updated)) {
    errors.push({
      file: filePath,
      field: 'updated',
      message: `Invalid date format: ${fm.updated} (use YYYY-MM-DD)`,
      severity: 'warning'
    });
  }
}
