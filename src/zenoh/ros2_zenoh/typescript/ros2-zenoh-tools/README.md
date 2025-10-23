# @ros2-zenoh/tools

**Documentation and validation tools for ros2-zenoh**

TypeScript tools for validating documentation frontmatter, deriving canonical IDs, and ensuring consistency across the ros2-zenoh project.

---

## Features

- ✅ **Frontmatter validation** - Validate YAML frontmatter against schema
- ✅ **ID derivation** - Derive canonical IDs from structured fields
- ✅ **Type safety** - Full TypeScript types for all frontmatter
- ✅ **CLI tool** - Command-line validation for pre-commit hooks
- ✅ **Programmatic API** - Use in MCP server, Obsidian plugins, web apps

---

## Installation

```bash
npm install @ros2-zenoh/tools
```

Or use directly with npx:

```bash
npx @ros2-zenoh/tools validate
```

---

## CLI Usage

### Validate Documentation

```bash
# Validate docs/ directory
npx @ros2-zenoh/tools validate

# Validate specific directory
npx @ros2-zenoh/tools validate path/to/docs
```

### Output

```
🔍 Validating documents in: /path/to/docs

❌ docs/06-Specs/Core/clock.md:
  ✗ [name] Name format invalid: must be lowercase with hyphens
  ⚠ [summary] Summary too long: 125 chars (max 120)

✅ docs/06-Specs/Python/rate-python.md

📊 Summary: 1 errors, 1 warnings
```

---

## Programmatic API

### Derive Canonical ID

```typescript
import { deriveCanonicalId } from '@ros2-zenoh/tools';

const id = deriveCanonicalId({
  type: 'spec',
  name: 'clock',
  variant: 'python'
});

console.log(id);  // "spec:clock-python"
```

### Validate Frontmatter

```typescript
import { validateFrontmatter } from '@ros2-zenoh/tools';

const errors = validateFrontmatter(frontmatter, 'path/to/file.md');

if (errors.length > 0) {
  console.error('Validation failed:', errors);
}
```

### Parse Document

```typescript
import { parseDocument } from '@ros2-zenoh/tools';

const doc = await parseDocument('docs/06-Specs/Core/clock.md');

console.log(doc.derivedId);       // "spec:clock"
console.log(doc.frontmatter.name); // "clock"
console.log(doc.content);          // markdown content
```

### Validate All Documents

```typescript
import { validateAllDocuments, formatErrors } from '@ros2-zenoh/tools';

const errors = await validateAllDocuments('docs/');
console.log(formatErrors(errors));
```

---

## Pre-commit Integration

Add to `.pre-commit-config.yaml`:

```yaml
repos:
  - repo: local
    hooks:
      - id: validate-docs
        name: Validate Documentation
        entry: npx @ros2-zenoh/tools validate
        language: node
        files: 'docs/.*\.md$'
        pass_filenames: false
```

---

## ID Derivation Rules

### Specs

```typescript
// Core spec
{ type: 'spec', name: 'clock' }
→ spec:clock

// Language binding
{ type: 'spec', name: 'clock', variant: 'python' }
→ spec:clock-python
```

### ADRs

```typescript
{ type: 'adr', number: '002' }
→ adr:002
```

### Patterns

```typescript
{ type: 'pattern', name: 'fixed-rate-loop' }
→ pattern:fixed-rate-loop
```

### Conventions

```typescript
{ type: 'convention', name: 'pausable-component-api' }
→ convention:pausable-component-api
```

---

## Validation Rules

### Universal Fields

- ✅ `type` - Required, must be valid document type
- ✅ `title` - Required, 3-100 characters
- ⚠️ `summary` - Optional, max 120 characters
- ⚠️ `classification` - Optional, must be valid enum
- ⚠️ `llm_processing` - Optional, must be valid enum

### Spec-Specific

- ✅ `name` - Required, lowercase with hyphens
- ✅ `version` - Required, X.Y or X.Y.Z format
- ✅ `status` - Required, must be valid enum

### ADR-Specific

- ✅ `number` - Required, digits only
- ✅ `status` - Required, must be valid enum
- ⚠️ `date` - Optional, YYYY-MM-DD format

### Pattern-Specific

- ✅ `name` - Required
- ✅ `version` - Required
- ✅ `status` - Required

### Convention-Specific

- ✅ `name` - Required

---

## Development

```bash
# Install dependencies
npm install

# Build
npm run build

# Watch mode
npm run dev

# Run validation
npm run validate
```

---

## Use Cases

### 1. Pre-commit Hooks

Automatically validate on git commit:

```yaml
- id: validate-docs
  entry: npx @ros2-zenoh/tools validate
```

### 2. GitHub Actions

```yaml
- name: Validate Documentation
  run: npx @ros2-zenoh/tools validate
```

### 3. MCP Server

```typescript
import { deriveCanonicalId, validateFrontmatter } from '@ros2-zenoh/tools';

// Use in MCP tools
export const tool = {
  name: 'validate_spec',
  handler: async (spec) => {
    const errors = validateFrontmatter(spec.frontmatter, spec.path);
    return { valid: errors.length === 0, errors };
  }
};
```

### 4. Obsidian Plugin

```typescript
import { deriveCanonicalId } from '@ros2-zenoh/tools';

// Get canonical ID for current file
const id = deriveCanonicalId(frontmatter);
```

### 5. Web UI

```typescript
import { validateFrontmatter } from '@ros2-zenoh/tools';

// Live validation in browser
function onFrontmatterChange(fm) {
  const errors = validateFrontmatter(fm, 'current-file.md');
  displayErrors(errors);
}
```

---

## License

MIT

---

## Related

- [ros2-zenoh Documentation](../../docs/)
- [Wikilinks and IDs Guide](../../docs/00-Index/wikilinks-and-ids-guide.md)
- [.cursorrules](../../.cursorrules) - AI coding standards


