# Wikilinks and IDs Quick Reference

**The Three Namespaces: Don't Confuse Them!**

---

## TL;DR

| Context | What to Use | Example |
|---------|-------------|---------|
| **Markdown wikilinks** | Filename (lowercase) | `[[clock]]` |
| **Frontmatter refs** | Just name/number | `related_specs: [clock]` |
| **Tool/Neo4j IDs** | Derived automatically | `spec:clock` |

**You only write filenames and name fields - tools derive IDs!**

---

## 1. Wikilinks (In Markdown)

### ✅ Correct

```markdown
See [[clock]] for core behavior.
See [[clock-python]] for Python binding.
Follows [[002-three-time-modes]] decision.
```

### ❌ Wrong

```markdown
See [[spec:clock]] for behavior.     # NO! Don't use derived IDs
See [[Clock]] for behavior.          # NO! Use lowercase
See [[spec-clock]] for behavior.     # NO! No namespace prefix
```

**Rule**: Wikilinks use **filenames** (without `.md`), always lowercase!

---

## 2. Frontmatter Cross-References

### ✅ Correct

```yaml
related_specs: [clock, rate, timer]
related_adrs: [002, 005]
related_patterns: [fixed-rate-loop]
```

### ❌ Wrong

```yaml
related_specs: ["spec:clock", "spec:rate"]  # NO! No namespace
related_specs: [[clock]]                     # NO! Not wikilinks
related_specs: [Clock]                       # NO! Use lowercase
```

**Rule**: Use **just names or numbers** - tools add the namespace!

---

## 3. Derived IDs (Automatic)

### How It Works

**Frontmatter:**
```yaml
type: spec
name: clock
```

**Derived ID:**
```
spec:clock
```

**You never write "spec:clock" anywhere!** Tools derive it.

### Examples

| Type | Frontmatter | Derived ID |
|------|-------------|------------|
| Spec | `type: spec, name: clock` | `spec:clock` |
| Python Binding | `type: spec, name: clock-python` | `spec:clock-python` |
| ADR | `type: adr, number: "002"` | `adr:002` |
| Pattern | `type: pattern, name: fixed-rate-loop` | `pattern:fixed-rate-loop` |
| Convention | `type: convention, name: pausable-api` | `convention:pausable-api` |

---

## 4. Filename Convention

**Filename MUST match name field:**

### ✅ Correct

```
File: clock.md
Frontmatter: name: clock

File: clock-python.md
Frontmatter: name: clock-python

File: 002-three-time-modes.md
Frontmatter: type: adr, number: "002"
```

### ❌ Wrong

```
File: my-clock.md
Frontmatter: name: clock  # Doesn't match!
```

**Validation will catch mismatches!**

---

## Complete Example

### File: `clock.md`

```yaml
---
type: spec
name: clock              # ← Matches filename
variant: core
title: "Clock Component - Core Specification"

related_specs: [rate, timer]      # ← Just names
related_adrs: [002]                # ← Just number
related_patterns: [fixed-rate-loop]
---

# Clock Component

The Clock component provides time management for testing.

See [[rate]] for fixed-rate execution patterns.
See [[timer]] for timer-based scheduling.
Follows [[002-three-time-modes]] decision.
```

### What Tools See

**Derived ID**: `spec:clock`

**Neo4j Relationships**:
```cypher
(spec:clock)-[:RELATES_TO]->(spec:rate)
(spec:clock)-[:RELATES_TO]->(spec:timer)
(spec:clock)-[:INFLUENCED_BY]->(adr:002)
(spec:clock)-[:USES_PATTERN]->(pattern:fixed-rate-loop)
```

**Obsidian Wikilinks**:
- `[[rate]]` → `rate.md`
- `[[timer]]` → `timer.md`
- `[[002-three-time-modes]]` → `002-three-time-modes.md`

---

## Why This Design?

### Three Independent Systems

1. **Filesystem** (Obsidian wikilinks)
   - Human-readable filenames
   - Case-insensitive linking
   - Easy browsing

2. **Frontmatter** (Structured data)
   - Machine-parseable
   - Type-safe relationships
   - No duplication

3. **Derived IDs** (Tools/Neo4j)
   - Stable references
   - Global uniqueness
   - Graph database keys

**They're aligned but not coupled** - gives flexibility!

---

## Common LLM Mistakes

### Mistake 1: Using IDs in Wikilinks

```markdown
# ❌ WRONG
See [[spec:clock]] for details.

# ✅ CORRECT
See [[clock]] for details.
```

### Mistake 2: Including Namespace in Frontmatter

```yaml
# ❌ WRONG
related_specs: ["spec:clock"]

# ✅ CORRECT
related_specs: [clock]
```

### Mistake 3: Filename Doesn't Match Name

```yaml
# File: my-component.md

# ❌ WRONG
name: component

# ✅ CORRECT
name: my-component
```

---

## Validation

Pre-commit hooks check:
- ✅ Filename matches `name` field
- ✅ All `related_specs` reference valid specs
- ✅ All `related_adrs` reference valid ADRs
- ✅ Wikilinks resolve to existing files

**You'll get warnings if you make mistakes!**

---

## Quick Decision Tree

**Writing a link in markdown?**
→ Use wikilink with filename: `[[clock]]`

**Writing frontmatter cross-reference?**
→ Use just name/number in typed field: `related_specs: [clock]`

**Need to reference in a tool/query?**
→ Derive ID: `f"{type}:{name}"` → `spec:clock`

**Creating a new file?**
→ Make filename match name field: `clock.md` + `name: clock`

---

## Summary

**Simple rule**: You work with **filenames** and **names**. Tools handle **IDs**.

- Wikilinks = filenames
- Frontmatter = names/numbers  
- Tools = derive IDs

**Don't mix them up!**


