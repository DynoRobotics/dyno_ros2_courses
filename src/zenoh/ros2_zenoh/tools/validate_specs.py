#!/usr/bin/env python3
"""
Validate that specifications are complete and ready for implementation.

Checks that specs have all required sections.
"""

import re
import sys
from pathlib import Path
from typing import List, Set

from validate_frontmatter import extract_frontmatter


# Required sections for specifications
REQUIRED_SPEC_SECTIONS = [
    "# .+ Specification",  # Title (regex)
    "## Purpose",
    "## Domain Model",
    "## Inputs",
    "## Outputs",
    "## State",
    "## Behavior",
    "## Quality Attributes",
    "## Test Requirements",
]

# Required subsections for Domain Model
REQUIRED_DOMAIN_SUBSECTIONS = [
    "### Entities",
    "### Value Objects",
    "### Aggregates",
]

# Required subsections for Test Requirements
REQUIRED_TEST_SUBSECTIONS = [
    "### Unit Tests",
    "### Integration Tests",
]


def extract_headings(markdown_content: str) -> List[str]:
    """Extract all markdown headings from content."""
    headings = []
    for line in markdown_content.split('\n'):
        if line.startswith('#'):
            headings.append(line.strip())
    return headings


def check_section_present(headings: List[str], section_pattern: str) -> bool:
    """Check if a section (or pattern) is present in headings."""
    for heading in headings:
        if re.match(section_pattern, heading):
            return True
    return False


def validate_spec_completeness(spec_file: Path) -> List[str]:
    """
    Validate that a spec has all required sections.
    
    Returns list of warnings/errors.
    """
    content = spec_file.read_text()
    frontmatter = extract_frontmatter(spec_file)
    
    if not frontmatter:
        return ["No frontmatter found"]
    
    # Skip if not a spec
    if frontmatter.get('category') != 'spec':
        return []
    
    # Skip if status is draft (can be incomplete)
    if frontmatter.get('status') == 'draft':
        return []  # Drafts can be incomplete
    
    errors = []
    headings = extract_headings(content)
    
    # Check required sections
    for section in REQUIRED_SPEC_SECTIONS:
        if not check_section_present(headings, section):
            errors.append(f"Missing required section: {section}")
    
    # Check domain model subsections
    if "## Domain Model" in headings:
        for subsection in REQUIRED_DOMAIN_SUBSECTIONS:
            if subsection not in headings:
                errors.append(
                    f"Missing required subsection in Domain Model: {subsection}"
                )
    
    # Check test requirements subsections
    if "## Test Requirements" in headings:
        for subsection in REQUIRED_TEST_SUBSECTIONS:
            if subsection not in headings:
                errors.append(
                    f"Missing required subsection in Test Requirements: {subsection}"
                )
    
    # Check for code blocks in Behavior section
    if "## Behavior" in content:
        behavior_section = content.split("## Behavior")[1].split("##")[0]
        if "```typescript" not in behavior_section:
            errors.append(
                "Behavior section should include TypeScript code blocks"
            )
    
    # Check for YAML in State section
    if "## State" in content:
        state_section = content.split("## State")[1].split("##")[0]
        if "```yaml" not in state_section:
            errors.append("State section should include YAML code block")
    
    # Check if spec is marked as safety-critical
    if frontmatter.get('safety_critical'):
        if "### Safety Tests" not in headings:
            errors.append(
                "Safety-critical spec must have '### Safety Tests' section"
            )
        
        if "## Quality Attributes" in content:
            qa_section = content.split("## Quality Attributes")[1].split("##")[0]
            if "### Safety" not in qa_section:
                errors.append(
                    "Safety-critical spec must have '### Safety' quality attribute"
                )
    
    # Check if spec is marked for generation
    if frontmatter.get('generate'):
        if "## Generate" not in content:
            errors.append(
                "Spec marked for generation should have '## Generate' section "
                "with example commands"
            )
    
    return errors


def validate_spec_links(spec_file: Path) -> List[str]:
    """Validate that wikilinks in spec are valid."""
    content = spec_file.read_text()
    errors = []
    
    # Extract [[wikilinks]]
    wikilinks = re.findall(r'\[\[([^\]]+)\]\]', content)
    
    docs_dir = spec_file.parent.parent
    
    for link in wikilinks:
        # Simple check - file should exist somewhere in docs/
        found = False
        for md_file in docs_dir.rglob("*.md"):
            if link in md_file.stem or link in md_file.name:
                found = True
                break
        
        if not found:
            errors.append(f"Broken wikilink: [[{link}]]")
    
    return errors


def main():
    """Validate all specs."""
    specs_dir = Path("docs/06-Specs")
    
    if not specs_dir.exists():
        print("✓ No specs directory yet")
        return 0
    
    errors_found = False
    
    for spec_file in specs_dir.glob("*.md"):
        errors = validate_spec_completeness(spec_file)
        link_errors = validate_spec_links(spec_file)
        
        all_errors = errors + link_errors
        
        if all_errors:
            print(f"⚠️  {spec_file.name}:")
            for error in all_errors:
                print(f"   - {error}")
            errors_found = True
        else:
            print(f"✓ {spec_file.name}")
    
    if errors_found:
        print("\n⚠️  Some specs have warnings (non-blocking)")
        return 0  # Don't fail CI on warnings
    else:
        print("\n✓ All specs valid!")
        return 0


if __name__ == "__main__":
    sys.exit(main())


