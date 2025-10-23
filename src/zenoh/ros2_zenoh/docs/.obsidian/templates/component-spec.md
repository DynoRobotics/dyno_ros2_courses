---
# Core Identity
id: spec:{{COMPONENT_NAME}}
title: {{COMPONENT_TITLE}} Specification
type: reference
updated: {{DATE}}

# Classification & Governance
classification: internal
llm_processing: cloud-ok
license: MIT
author: Development Team

# Discoverability
summary: {{ONE_SENTENCE_SUMMARY}}
tags: [spec, {{TAG1}}, {{TAG2}}]

# Spec-Specific
category: spec
domain: {{DOMAIN}}.{{SUBDOMAIN}}
pattern: pattern:{{PATTERN_NAME}}
version: "0.1"
status: draft
generate: true
derived_from: ""

# Safety & Compliance
safety_critical: false
security_sensitive: false
compliance: []
hazard_level: none

# Relationships
depends_on: []
related: []
supersedes: ""

# Implementation Tracking
implementation:
  python: planned
  rust: planned
  c: planned
  typescript: planned

# External References
issue: ""
pr: ""
epic: ""

# Future-Proofing
schema_version: "1.0"
---

# {{COMPONENT_TITLE}} Specification

## Purpose

Brief description of what this component does and why it exists.

## Domain Model

### Entities
*(Objects with identity)*

**EntityName**
- Identity: (field1, field2)
- Responsibilities: What it does
- Invariants: Rules that must hold

### Value Objects
*(Immutable descriptors)*

**ValueObjectName**
- Type: Underlying type
- Format: Expected format/range
- Validation: Rules

### Aggregates
*(Consistency boundaries)*

**AggregateName**
- Root: Root entity
- Members: What's included
- Invariant: Consistency rule

## Inputs

| Topic | Type | Rate | Domain Concept | Constraints |
|-------|------|------|----------------|-------------|
| `/input_topic` | `package/MsgType` | 10 Hz | What this represents | Valid range, etc. |

## Outputs

| Topic | Type | Rate | Domain Concept | Guarantees |
|-------|------|------|----------------|-----------|
| `/output_topic` | `package/MsgType` | 10 Hz | What this produces | What's guaranteed |

## State

```yaml
state:
  field_name:
    type: int | float | str | bool | custom_type
    default: default_value
    description: What this field represents
```

## Behavior (Business Rules)

### Rule 1: Rule Name

**Purpose:** What this rule enforces

**Logic:**
```typescript
function ruleName(input: InputType): OutputType {
  // TypeScript pseudo-code showing the logic
  if (condition) {
    return result;
  }
}
```

**Invariants:**
- Condition that must always hold
- Relationship between inputs/outputs

## Quality Attributes

### Performance
- **Requirement:** < 10ms latency (p99)
- **Verification:** Benchmark tests
- **Mitigation:** If requirement not met, do X

### Reliability
- **Requirement:** 99.9% uptime
- **Verification:** Long-running tests
- **Mitigation:** Graceful degradation

### Safety (if safety_critical: true)
- **Requirement:** Must stop within X seconds
- **Verification:** Safety tests with fault injection
- **Mitigation:** Watchdog, fail-safe mode

### Security (if security_sensitive: true)
- **Requirement:** Encrypted data at rest
- **Verification:** Security audit
- **Mitigation:** Key rotation, access control

## Test Requirements

### Unit Tests
- [ ] Test behavior rule 1
- [ ] Test behavior rule 2
- [ ] Test edge case: empty input
- [ ] Test edge case: invalid data
- [ ] Test error handling

### Integration Tests
- [ ] End-to-end: input → output
- [ ] Multiple inputs handled correctly
- [ ] State transitions work
- [ ] Resource cleanup works

### Safety Tests (if safety_critical)
- [ ] Fail-safe mode activates on error
- [ ] Watchdog catches hung state
- [ ] Emergency stop works

### Performance Tests
- [ ] Latency < requirement (p50, p99)
- [ ] Throughput > requirement
- [ ] Memory usage < budget
- [ ] No memory leaks

## Generate

```bash
# Generate implementation (Phase 4+)
ros2-zenoh generate from-spec {{COMPONENT_NAME}}.md --lang python

# Generate tests
ros2-zenoh generate tests {{COMPONENT_NAME}}.md --lang python

# Generate all languages
ros2-zenoh generate package {{COMPONENT_NAME}}.md --langs python,rust,c,typescript
```

## References

- Pattern: [[{{PATTERN_NAME}}]]
- Related Spec: [[RelatedComponent]]
- Concept: [[RelevantConcept]]

## Notes

Additional context, design decisions, trade-offs, etc.

