# 📥 Inbox

**Temporary staging area for unorganized notes**

---

## Purpose

This is a temporary workspace for:
- ✍️ Quick ideas that need capturing
- 🤔 Notes you're not sure where to place yet
- 📝 Drafts before formalizing
- 🧪 Experimental content

---

## Rules

1. ✅ Files can stay here temporarily while you develop them
2. ⚠️ Organize within **7 days**
3. ❌ Don't commit files older than **14 days**
4. 📝 Add proper frontmatter before moving to permanent location

---

## Creating Notes in Inbox

### Method 1: Create Manually

Just create a new file here directly:
```
_inbox/my-idea.md
```

### Method 2: Wikilink from Inbox

```markdown
[[_inbox/my-draft|My Draft Idea]]
```

Clicking creates the file in inbox.

---

## Organizing

When ready to make permanent, move to appropriate directory:

### Specifications
```bash
mv _inbox/my-spec.md 06-Specs/Core/my-spec.md
```

### Patterns
```bash
mv _inbox/my-pattern.md 05-Patterns/my-pattern.md
```

### Tutorials
```bash
mv _inbox/my-tutorial.md 02-Tutorials/my-tutorial.md
```

### Concepts
```bash
mv _inbox/my-concept.md 01-Concepts/my-concept.md
```

---

## Validation

Check inbox status:

```bash
cd /path/to/ros2_zenoh
npx @ros2-zenoh/tools inbox-status
```

Output shows:
- ✅ Recent files (< 7 days)
- ⚠️ Files needing organization (7-14 days)
- ❌ Files too old (14+ days) - must organize

---

## Why Inbox?

**Low friction**: Capture ideas quickly without deciding final location

**Enforced organization**: Validation ensures files don't languish here forever

**Clear workflow**: 
1. Create → 2. Develop → 3. Organize → 4. Commit

---

**💡 Tip**: Use this for genuine drafts/experiments. For files you know belong in a specific directory, create them there directly!


