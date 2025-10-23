/**
 * Type definitions for ros2-zenoh documentation frontmatter
 */

export type DocumentType =
  | 'spec'
  | 'adr'
  | 'pattern'
  | 'convention'
  | 'tutorial'
  | 'howto'
  | 'concept'
  | 'index'
  | 'reference'
  | 'api-reference';

export type Classification = 'public' | 'internal' | 'confidential' | 'restricted';

export type LLMProcessing = 'allowed' | 'cloud-ok' | 'on-prem-only' | 'no-llm' | 'review-required';

export type Status = 'draft' | 'review' | 'approved' | 'implemented' | 'deprecated' | 'accepted' | 'proposed' | 'superseded';

/**
 * Base frontmatter fields (common to all documents)
 */
export interface BaseFrontmatter {
  type: DocumentType;
  title: string;
  summary?: string;
  tags?: string[];
  
  // Classification & Governance
  classification?: Classification;
  llm_processing?: LLMProcessing;
  schema_version?: string;
  
  // Authorship
  author?: string;
  maintainer?: string;
  reviewed_by?: string;
}

/**
 * Spec-specific frontmatter
 */
export interface SpecFrontmatter extends BaseFrontmatter {
  type: 'spec';
  name: string;  // Component name (e.g., "clock", "rate", "clock-python")
  component_type?: string;  // e.g., "utility", "communication"
  variant?: 'core' | 'python' | 'rust' | 'c' | 'typescript';
  version: string;
  status: Status;
  
  // Relationships
  depends_on?: {
    specs?: string[];
    adrs?: string[];
  };
  related_specs?: string[];
  related_adrs?: string[];
  related_patterns?: string[];
  
  // Implementation tracking
  ros2_zenoh?: {
    languages?: string[];
    components?: string[];
    phase?: string;
    test_coverage?: string | null;
  };
}

/**
 * ADR-specific frontmatter
 */
export interface ADRFrontmatter extends BaseFrontmatter {
  type: 'adr';
  number: string;  // e.g., "001", "002"
  status: Status;
  date?: string;
  supersedes?: string | null;
  superseded_by?: string | null;
}

/**
 * Pattern-specific frontmatter
 */
export interface PatternFrontmatter extends BaseFrontmatter {
  type: 'pattern';
  name: string;
  category?: string;
  version: string;
  status: Status;
  languages?: string[];
  derived_from?: string;
}

/**
 * Convention-specific frontmatter
 */
export interface ConventionFrontmatter extends BaseFrontmatter {
  type: 'convention';
  name: string;
  status?: string;
  related_specs?: string[];
  related_adrs?: string[];
  related_patterns?: string[];
}

/**
 * Union type for all frontmatter
 */
export type Frontmatter =
  | SpecFrontmatter
  | ADRFrontmatter
  | PatternFrontmatter
  | ConventionFrontmatter
  | BaseFrontmatter;

/**
 * Validation error
 */
export interface ValidationError {
  file: string;
  field?: string;
  message: string;
  severity: 'error' | 'warning';
}

/**
 * Parsed document
 */
export interface Document {
  filePath: string;
  frontmatter: Frontmatter;
  content: string;
  derivedId: string;
}

