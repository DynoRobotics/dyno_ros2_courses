#!/usr/bin/env python3
"""
Validate YAML frontmatter in markdown files.

Checks that specs and patterns have required frontmatter fields.
"""

import re
import sys
from pathlib import Path
from typing import Dict, List, Optional

import yaml


# Required fields by document type
REQUIRED_UNIVERSAL = ['type']

REQUIRED_BY_TYPE = {
    'spec': ['id', 'title', 'version', 'status'],  # Specs use component_type instead of category
    'pattern': ['id', 'title', 'category', 'version', 'status', 'languages'],
    'adr': ['id', 'title', 'status'],
    'convention': ['id', 'title'],
    'api-reference': ['module'],  # Auto-generated docs have different requirements
}

# Valid enum values
VALID_ENUMS = {
    'type': ['reference', 'index', 'draft', 'tutorial', 'howto', 'concept', 
             'spec', 'adr', 'convention', 'pattern', 'api-reference'],
    'classification': ['public', 'internal', 'confidential', 'restricted'],
    'llm_processing': ['allowed', 'cloud-ok', 'on-prem-only', 'no-llm', 'review-required'],
    'status': ['draft', 'review', 'approved', 'implemented', 'deprecated', 
               'accepted', 'proposed', 'superseded'],
    'category': ['spec', 'pattern', 'moc', 'index', 'navigation', 'explanation', 
                 'learning', 'problem-solving', 'reference'],
}


def extract_frontmatter(markdown_file: Path) -> Optional[Dict]:
    """Extract YAML frontmatter from markdown file."""
    content = markdown_file.read_text()
    
    # Match --- frontmatter ---
    match = re.match(r'^---\n(.*?)\n---', content, re.DOTALL)
    if not match:
        return None
    
    try:
        frontmatter = yaml.safe_load(match.group(1))
        return frontmatter
    except yaml.YAMLError as e:
        raise ValueError(f"Invalid YAML in frontmatter: {e}")


def validate_frontmatter(
    frontmatter: Dict,
    file_path: Path
) -> List[str]:
    """
    Validate frontmatter against schema.
    
    Returns list of error messages (empty if valid).
    """
    errors = []
    
    # Check universal required fields
    for field in REQUIRED_UNIVERSAL:
        if field not in frontmatter:
            errors.append(f"Missing required field: {field}")
    
    # Check type-specific required fields
    doc_type = frontmatter.get('type')
    if doc_type in REQUIRED_BY_TYPE:
        for field in REQUIRED_BY_TYPE[doc_type]:
            if field not in frontmatter:
                errors.append(
                    f"Missing required field for {doc_type}: {field}"
                )
    
    # Validate enums
    for field, valid_values in VALID_ENUMS.items():
        if field in frontmatter:
            value = frontmatter[field]
            if value not in valid_values:
                errors.append(
                    f"Invalid {field}: '{value}'. "
                    f"Must be one of: {', '.join(valid_values)}"
                )
    
    # Validate ID format (flexible - allows : or - separator)
    if 'id' in frontmatter:
        id_value = frontmatter['id']
        # Accept either 'namespace:name' or 'namespace-name' format
        if not re.match(r'^[a-z]+[:-][a-z0-9-]+$', id_value):
            errors.append(
                f"Invalid id format: '{id_value}'. "
                "Must be lowercase with - or : separator (e.g., 'spec:my-component' or 'adr-001')"
            )
    
    # Validate date format
    if 'updated' in frontmatter:
        date_str = frontmatter['updated']
        if not re.match(r'^\d{4}-\d{2}-\d{2}$', str(date_str)):
            errors.append(
                f"Invalid date format: '{date_str}'. "
                "Must be YYYY-MM-DD"
            )
    
    # Validate version format (if present) - accept X.Y or X.Y.Z
    if 'version' in frontmatter:
        version = frontmatter['version']
        if not re.match(r'^\d+\.\d+(\.\d+)?$', str(version)):
            errors.append(
                f"Invalid version format: '{version}'. "
                "Must be 'X.Y' or 'X.Y.Z' (e.g., '1.0' or '1.0.0')"
            )
    
    # Validate summary length
    if 'summary' in frontmatter:
        summary = frontmatter['summary']
        if len(summary) > 120:
            errors.append(
                f"Summary too long: {len(summary)} chars. "
                "Maximum 120 characters."
            )
    
    # Check schema_version if present
    if 'schema_version' in frontmatter:
        schema = frontmatter['schema_version']
        if schema != "1.0":
            errors.append(
                f"Unknown schema version: '{schema}'. "
                "Expected '1.0'"
            )
    
    return errors


def main():
    """Validate all markdown files with frontmatter."""
    docs_dir = Path("docs")
    if not docs_dir.exists():
        print("Error: docs/ directory not found")
        return 1
    
    errors_found = False
    
    for md_file in docs_dir.rglob("*.md"):
        # Skip template files
        if '.obsidian/templates' in str(md_file) or '/templates/' in str(md_file):
            continue
        
        frontmatter = extract_frontmatter(md_file)
        
        if frontmatter is None:
            # No frontmatter - only required for certain files
            require_frontmatter = [
                '06-Specs', '05-Patterns', '00-Index',
                '03-Conventions', '09-Architecture-Decision-Records'
            ]
            if md_file.parent.name in require_frontmatter or 'Core' in str(md_file) or 'Python' in str(md_file):
                print(f"❌ {md_file}: Missing frontmatter")
                errors_found = True
            continue
        
        errors = validate_frontmatter(frontmatter, md_file)
        
        if errors:
            print(f"❌ {md_file}:")
            for error in errors:
                print(f"   - {error}")
            errors_found = True
        else:
            print(f"✓ {md_file}")
    
    if errors_found:
        print("\n❌ Frontmatter validation failed!")
        return 1
    else:
        print("\n✓ All frontmatter valid!")
        return 0


if __name__ == "__main__":
    sys.exit(main())


