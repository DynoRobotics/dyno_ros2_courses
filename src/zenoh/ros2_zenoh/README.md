# ROS2 Zenoh Ecosystem

**Multi-language, documentation-driven ROS2 ↔ Zenoh bridge**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## Overview

`ros2_zenoh` is a curated, quality-controlled ecosystem for bridging ROS2 and Zenoh across multiple programming languages. Everything in this directory is documented, tested, and validated.

**Key Features:**
- 📚 **Documentation-Driven**: Specifications → Implementation → Tests
- 🌍 **Multi-Language**: Python, Rust, C, TypeScript
- ⚡ **Fast**: Rust-based tooling (Ruff, typos, etc.)
- ✅ **Quality Gates**: Automated validation on every commit
- 🧪 **Testable**: Comprehensive testing infrastructure

---

## Repository Structure

```
ros2_zenoh/
├── docs/                    # Obsidian vault (SOURCE OF TRUTH)
│   ├── 00-Index/           # Navigation, domain model, ADRs
│   ├── 01-Concepts/        # Explanations (Diátaxis)
│   ├── 02-Tutorials/       # Learning guides
│   ├── 03-HowTo/           # Problem-solving
│   ├── 04-Reference/       # API documentation
│   ├── 05-Patterns/        # Proven code patterns
│   └── 06-Specs/           # Component specifications
│
├── python/                  # Python implementation
│   └── ros2_zenoh_python/
│
├── rust/                    # Rust implementation (future)
├── typescript/              # TypeScript implementation (future)
├── c/                       # C implementation (future)
│
├── interface-generator/     # Multi-language message generator
├── mcp-server/             # MCP server for tooling
├── patterns/               # Pattern library
│
├── tools/                   # Validation scripts
├── .cursorrules            # AI coding standards
├── ruff.toml               # Python linting config
├── .markdownlint.yaml      # Markdown linting config
├── .yamllint.yaml          # YAML linting config
├── .pre-commit-config.yaml # Git pre-commit hooks
├── Justfile                # Task runner
└── pyproject.toml          # Python workspace config
```

---

## Quick Start

### Prerequisites

**Choose your setup method:**

#### Option A: Nix (Recommended - Zero-config)

```bash
# Install Nix (one-time)
curl -L https://nixos.org/nix/install | sh

# Install direnv (optional but nice)
nix-env -iA nixpkgs.direnv
eval "$(direnv hook bash)"  # Add to ~/.bashrc

# Enter directory and allow
cd ros2_zenoh && direnv allow

# Everything auto-installs! ✨
```

#### Option B: Manual Install

```bash
# Python 3.10+
python --version

# Install development tools
pip install ruff mypy pytest pytest-asyncio pre-commit

# Install just (task runner)
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/bin

# Or: cargo install just
```

### Setup

```bash
cd ros2_zenoh

# Install pre-commit hooks
pre-commit install

# Install Python package in dev mode
just dev-install

# Run all validations
just validate

# Run tests
just test
```

---

## Development Workflow

### 1. Document First (Specification-Driven)

```bash
# Create new component spec
just new-spec MyComponent

# Edit: docs/06-Specs/MyComponent.md
# Fill in: domain model, inputs, outputs, behavior
```

### 2. Implement from Spec

```python
# Write implementation following the spec
# Include type hints, docstrings, tests
```

### 3. Validate & Test

```bash
# Format code
just fmt

# Lint everything
just lint

# Type check
just typecheck

# Run tests
just test

# Or run everything at once
just ci
```

### 4. Commit

```bash
# Pre-commit hooks run automatically
git add .
git commit -m "feat: implement MyComponent from spec"
```

---

## Validation Tools

### Python: Ruff (100x faster than Black/Flake8)

```bash
# Format
ruff format python/

# Lint
ruff check python/ --fix

# Or via just
just fmt
just lint
```

### Markdown: markdownlint

```bash
# Lint docs
markdownlint --fix 'docs/**/*.md'

# Or via just
just lint
```

### YAML: yamllint

```bash
# Validate frontmatter
yamllint docs/

# Or via just
just lint
```

### Typos: typos-cli (Rust-based spell checker)

```bash
# Check for typos
just typos

# Fix typos automatically
just typos-fix
```

---

## Testing

```bash
# Run all tests
just test

# Run with coverage
just test-cov

# Run only fast tests (skip slow/interop)
just test-fast

# Run interop tests
just test-interop

# Test documentation code examples
just test-docs

# Watch mode (auto-run on change)
just watch
```

### Documentation Testing

All Python code blocks in documentation are automatically tested:

```bash
# Test that code examples work
just test-docs
```

This validates:
- ✅ Python syntax is valid
- ✅ Tutorial examples have necessary imports
- ✅ Code blocks are runnable

**Markers for non-runnable code:**
- `{{placeholder}}` - Template variables
- `...` - Pseudo-code ellipsis
- `# Pseudo-code` - Explicit marker
- `TODO` / `FIXME` - Not implemented

Example:

````markdown
```python
# This will be tested
from ros2_zenoh_python import Node

async with Node("test") as node:
    print("Works!")
```

```python
# This will be skipped
def example({{NAME}}):  # Template
    ...  # Pseudo-code
```
````

---

## Documentation

Documentation uses **Obsidian** with **Diátaxis** framework:

- **Concepts** (`01-Concepts/`) - Explain *why* and *how*
- **Tutorials** (`02-Tutorials/`) - Learn by doing
- **How-To Guides** (`03-HowTo/`) - Solve specific problems
- **Reference** (`04-Reference/`) - Technical specifications

### View Documentation

```bash
# Option 1: Obsidian (recommended)
# File → Open Vault → ros2_zenoh/docs

# Option 2: Any markdown viewer
# Files are plain markdown with YAML frontmatter
```

---

## Quality Standards

All code in this directory MUST:

✅ Have a specification in `docs/06-Specs/`
✅ Include comprehensive tests (>90% coverage)
✅ Pass all linters (ruff, markdownlint, yamllint)
✅ Pass type checking (mypy)
✅ Have documentation (docstrings, examples)
✅ Pass pre-commit hooks

---

## Architecture Decisions

See [`docs/00-Index/`](docs/00-Index/) for:

- ADR-001: Custom Mock Classes
- ADR-002: Pure Asyncio (No Timer Class)
- ADR-003: Three Distinct Time Modes
- ADR-004: advance_by() for Multi-Event Processing
- ADR-005: Ring Buffer Semantics
- [Full list](docs/00-Index/Architecture-MOC.md)

---

## Contributing

1. Read the [Architecture Guide](docs/00-Index/Architecture-MOC.md)
2. Follow [Documentation-Driven Workflow](docs/00-Index/ACTION_PLAN.md)
3. Write specification first
4. Implement from spec
5. Add tests
6. Submit PR (all validations must pass)

---

## Project Status

**Phase 1:** Python Testing Infrastructure (Current)
- ⏳ Clock implementation
- ⏳ MockNode implementation
- ⏳ Pytest fixtures

See [ACTION_PLAN.md](docs/00-Index/ACTION_PLAN.md) for full roadmap.

---

## License

MIT License - See [LICENSE](LICENSE) for details

---

## Links

- **Full Plan:** [ROS2_ZENOH_ECOSYSTEM_PLAN.md](ROS2_ZENOH_ECOSYSTEM_PLAN.md)
- **Migration Plan:** [docs/00-Index/MIGRATION_PLAN.md](docs/00-Index/MIGRATION_PLAN.md)
- **Action Plan:** [docs/00-Index/ACTION_PLAN.md](docs/00-Index/ACTION_PLAN.md)
- **Domain Model:** [docs/00-Index/Domain-Model.md](docs/00-Index/Domain-Model.md)

---

**Built with ❤️ using documentation-driven development**

