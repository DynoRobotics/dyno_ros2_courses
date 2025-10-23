---
# Required fields
classification: internal
llm_processing: allowed
schema_version: "1.0"
type: adr
id: "adr-{{NUMBER}}-{{SLUG}}"
title: "ADR-{{NUMBER}}: {{TITLE}}"
summary: "{{ONE_LINE_SUMMARY}}"

# ADR-specific fields
status: "{{STATUS}}"  # proposed | accepted | deprecated | superseded
date: "{{DATE}}"
deciders: ["{{AUTHOR}}"]
consulted: []
informed: []

# Optional metadata
tags: []
related_patterns: []
related_specs: []
supersedes: null
superseded_by: null

# Future-proofing
ros2_zenoh:
  languages: []
  components: []
  phase: null
---

# ADR-{{NUMBER}}: {{TITLE}}

## Status

**{{STATUS}}** ({{DATE}})

{{#if supersedes}}
Supersedes: [[{{supersedes}}]]
{{/if}}

{{#if superseded_by}}
Superseded by: [[{{superseded_by}}]]
{{/if}}

---

## Context

<!-- What is the issue that we're seeing that is motivating this decision or change? -->

<!-- Include any constraints, requirements, or assumptions. -->

---

## Decision

<!-- What is the change that we're proposing and/or doing? -->

<!-- State the decision clearly and unambiguously. -->

---

## Consequences

### Positive

<!-- What becomes easier or better after this decision? -->

### Negative

<!-- What becomes harder or worse? What are the trade-offs? -->

### Neutral

<!-- What changes but is neither clearly positive nor negative? -->

---

## Alternatives Considered

### Option 1: {{OPTION_NAME}}

**Pros:**
- 

**Cons:**
- 

**Why rejected:**

### Option 2: {{OPTION_NAME}}

**Pros:**
- 

**Cons:**
- 

**Why rejected:**

---

## References

<!-- Links to related documents, discussions, or external resources -->

- 

---

## Implementation Notes

<!-- Technical details about implementing this decision -->

<!-- Include code examples if helpful -->

```python
# Example
```

---

## Review History

| Date | Reviewer | Decision |
|------|----------|----------|
| {{DATE}} | {{AUTHOR}} | Proposed |


