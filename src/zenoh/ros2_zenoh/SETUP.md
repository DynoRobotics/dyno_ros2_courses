# Setup Guide

**Quick start for the ros2_zenoh ecosystem**

---

## Prerequisites

```bash
# Python 3.10+
python3 --version

# Pip (latest)
pip install --upgrade pip
```

---

## Installation Methods

Choose the method that works best for you:

### Method 1: Nix + direnv (Recommended - Fully Reproducible)

**Perfect for:** Anyone who wants zero-config, reproducible environment

```bash
# One-time: Install Nix
curl -L https://nixos.org/nix/install | sh

# One-time: Install direnv (optional but recommended)
nix-env -iA nixpkgs.direnv

# Hook direnv into your shell (add to ~/.bashrc or ~/.zshrc)
eval "$(direnv hook bash)"  # or: eval "$(direnv hook zsh)"

# Clone and enter directory
cd ros2_zenoh

# Allow direnv (one-time per directory)
direnv allow

# That's it! Everything is ready.
# Tools auto-install on first run.
```

**What you get:**
- ✅ Automatic environment activation on `cd`
- ✅ All tools at correct versions (just, ruff, typos, etc.)
- ✅ Python virtual environment auto-created
- ✅ Pre-commit hooks auto-installed
- ✅ No Docker needed
- ✅ Works on Linux & macOS

---

### Method 2: Docker (Good for teams standardizing on Docker)

**Perfect for:** Teams already using Docker/VSCode DevContainers

```bash
# Use .devcontainer (if exists)
# Or run in Docker manually
docker-compose up -d
```

---

### Method 3: Manual Install (Traditional)

**Perfect for:** You know what you're doing

```bash
# Core Python tools (Ruff, mypy, pytest, pre-commit)
pip install ruff mypy pytest pytest-asyncio pytest-cov pre-commit pyyaml

# Install just (task runner) - Choose one:

# Option A: Cargo (if you have Rust)
cargo install just

# Option B: Install script
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/bin

# Option C: Package manager
# Ubuntu/Debian: sudo apt install just
# macOS: brew install just

# Install other tools
npm install -g markdownlint-cli2
pip install yamllint
cargo install typos-cli  # Optional but recommended
```

### 2. Setup Pre-Commit Hooks

```bash
cd /path/to/ros2_zenoh

# Install hooks
pre-commit install

# Test (optional)
pre-commit run --all-files
```

### 3. Install Python Package (Development Mode)

```bash
# When Python implementation is copied
just dev-install
```

---

## Verify Setup

```bash
# Format code
just fmt

# Run all validations
just validate

# Test documentation
just test-docs

# Show available commands
just --list
```

---

## IDE Setup

### VS Code / Cursor

1. **Install Extensions:**
   - Python
   - Ruff
   - Markdown All in One
   - YAML

2. **Settings** (`.vscode/settings.json`):
```json
{
  "python.linting.enabled": true,
  "python.linting.ruffEnabled": true,
  "python.formatting.provider": "none",
  "[python]": {
    "editor.formatOnSave": true,
    "editor.codeActionsOnSave": {
      "source.fixAll": true,
      "source.organizeImports": true
    },
    "editor.defaultFormatter": "charliermarsh.ruff"
  },
  "files.associations": {
    ".cursorrules": "markdown"
  }
}
```

### Obsidian (Documentation)

1. **Open Vault:**
   - File → Open Vault
   - Select: `ros2_zenoh/docs`

2. **Install Plugins:**
   - Templater (for spec/pattern templates)
   - Dataview (for dashboards)
   - Markdown Linter (optional)

3. **Configuration:**
   - Settings → Templates → Template folder: `.obsidian/templates`

---

## Workflow

### Creating a New Specification

```bash
# 1. Create from template
just new-spec MyComponent

# 2. Edit: docs/06-Specs/MyComponent.md
#    Fill in all sections

# 3. Validate
just validate

# 4. Implement code following the spec

# 5. Write tests

# 6. Commit (pre-commit hooks run automatically)
git add .
git commit -m "feat: implement MyComponent from spec"
```

### Creating a New Pattern

```bash
# 1. Create from template
just new-pattern MyPattern

# 2. Edit: docs/05-Patterns/MyPattern.md
#    Add examples for all languages

# 3. Validate
just validate

# 4. Commit
git commit -m "docs: add MyPattern pattern"
```

---

## Troubleshooting

### Ruff Not Found

```bash
# Ensure ruff is installed
pip install ruff

# Or install in isolated environment
pipx install ruff
```

### Just Not Found

```bash
# Check installation
which just

# Add to PATH if needed
export PATH="$HOME/bin:$PATH"

# Or use absolute path
~/bin/just --list
```

### Pre-Commit Hooks Failing

```bash
# Update hooks
pre-commit autoupdate

# Clear cache
pre-commit clean

# Re-install
pre-commit uninstall
pre-commit install
```

### Import Errors

```bash
# Install package in development mode
cd python/ros2_zenoh_python
pip install -e .

# Or use just
just dev-install
```

---

## Common Commands

```bash
# Format and lint
just fmt
just lint

# Type check
just typecheck

# Run all validations
just validate

# Test
just test               # All tests
just test-fast          # Skip slow tests
just test-cov           # With coverage
just test-docs          # Documentation examples

# Clean
just clean              # Remove build artifacts

# Statistics
just stats              # Show project stats

# Full CI
just ci                 # All validations + tests
```

---

## Next Steps

1. **Read the docs:** `docs/00-Index/README.md`
2. **Check action plan:** `docs/00-Index/ACTION_PLAN.md`
3. **Review architecture:** `docs/00-Index/Architecture-MOC.md`
4. **Start coding!**

---

## Getting Help

- Read: [README.md](README.md)
- Check: [Architecture MOC](docs/00-Index/Architecture-MOC.md)
- View: [Ecosystem Plan](ROS2_ZENOH_ECOSYSTEM_PLAN.md)
- List commands: `just --list`

---

**Happy coding!** 🚀

