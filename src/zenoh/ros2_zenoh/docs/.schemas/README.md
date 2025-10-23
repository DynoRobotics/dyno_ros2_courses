# Documentation Validation

This directory will contain JSON Schema definitions for validating frontmatter (future).

## Current Validation

### 1. Frontmatter Validation (`tools/validate_frontmatter.py`)

Checks:
- ✅ Required fields present (`id`, `title`, `type`, `updated`)
- ✅ Valid enum values (classification, llm_processing, status, etc.)
- ✅ ID format (`namespace:kebab-case`)
- ✅ Date format (YYYY-MM-DD)
- ✅ Version format (X.Y)
- ✅ Summary length (< 120 chars)

Run: `just validate` or `python tools/validate_frontmatter.py`

### 2. Spec Completeness (`tools/validate_specs.py`)

Checks:
- ✅ All required sections present
- ✅ Domain Model subsections (Entities, Value Objects, Aggregates)
- ✅ Test Requirements subsections (Unit, Integration)
- ✅ TypeScript code blocks in Behavior section
- ✅ YAML code blocks in State section
- ✅ Safety tests for safety-critical specs
- ✅ Wikilinks are valid

Run: `just validate` or `python tools/validate_specs.py`

### 3. Code Example Testing (`tools/test_documentation.py`)

Checks:
- ✅ Python code blocks have valid syntax
- ✅ Tutorial examples have necessary imports
- ✅ Runnable code is syntactically correct

Run: `just test-docs` or `python tools/test_documentation.py`

## Pre-Commit Hooks

All validations run automatically on `git commit`:

```bash
# Install hooks
pre-commit install

# Run manually
pre-commit run --all-files
```

## Future: JSON Schema

We can add JSON Schema for more sophisticated validation:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Spec Frontmatter",
  "type": "object",
  "required": ["id", "title", "type"],
  "properties": {
    "id": {
      "type": "string",
      "pattern": "^spec:[a-z0-9-]+$"
    }
  }
}
```

Then use `check-jsonschema` pre-commit hook.


