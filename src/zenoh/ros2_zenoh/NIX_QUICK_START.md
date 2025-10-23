# Nix Quick Start

**Zero-config development environment using Nix + direnv**

---

## One-Time Setup

### 1. Install Nix

```bash
# Official installer (multi-user, recommended)
sh <(curl -L https://nixos.org/nix/install) --daemon

# Or single-user
sh <(curl -L https://nixos.org/nix/install) --no-daemon
```

### 2. Enable Flakes (Required)

Add to `~/.config/nix/nix.conf`:
```
experimental-features = nix-command flakes
```

Or create file if it doesn't exist:
```bash
mkdir -p ~/.config/nix
echo "experimental-features = nix-command flakes" > ~/.config/nix/nix.conf
```

### 3. Install direnv (Optional but Highly Recommended)

```bash
# Using Nix
nix-env -iA nixpkgs.direnv

# Hook into your shell (add to ~/.bashrc or ~/.zshrc)
eval "$(direnv hook bash)"   # For bash
eval "$(direnv hook zsh)"    # For zsh
```

---

## Using the Environment

### With direnv (Auto-activation)

```bash
cd /path/to/ros2_zenoh

# First time: allow direnv
direnv allow

# Now every time you cd into this directory:
# - Environment auto-activates
# - Tools are available
# - Python venv is activated
# - Pre-commit hooks are installed

# Just start working!
just --list
just validate
```

### Without direnv (Manual activation)

```bash
cd /path/to/ros2_zenoh

# Enter the Nix development shell
nix develop

# Or use the shorthand
nix develop -c $SHELL

# Now tools are available
just --list
```

---

## What Gets Installed

### Automatically included:

**Rust-based tools (fast!):**
- `just` - Task runner
- `typos` - Spell checker

**Markdown/YAML:**
- `markdownlint-cli2` - Markdown linting

**Python:**
- Python 3.11
- Virtual environment (`.venv`) auto-created
- Tools installed via pip: ruff, mypy, pytest, pre-commit

**Development:**
- git
- ripgrep, fd, bat (modern CLI tools)
- pre-commit (hooks auto-installed)

---

## Common Workflows

### First Time Setup

```bash
cd ros2_zenoh
direnv allow  # or: nix develop

# Everything is ready!
just validate
```

### Daily Development

```bash
# Just cd into directory (with direnv)
cd ros2_zenoh

# Or activate manually
nix develop

# Work as normal
just fmt
just test
git commit -m "feat: ..."
```

### Update Dependencies

```bash
# Update Nix flake inputs
nix flake update

# This updates all tool versions
```

---

## Advantages of Nix

### vs Docker

| Aspect | Nix | Docker |
|--------|-----|--------|
| **Startup** | Instant | Slow |
| **Resources** | Native | Virtualization |
| **Integration** | Native | Port forwarding |
| **Reproducible** | ✅ Yes | ✅ Yes |
| **Size** | Small | Large |

### vs Manual Install

| Aspect | Nix | Manual |
|--------|-----|--------|
| **Setup Time** | 5 min (once) | 15+ min |
| **Consistency** | Perfect | Varies |
| **Updates** | `nix flake update` | Manual |
| **Clean** | Auto-cleanup | Manual |
| **Multiple Projects** | Isolated | Conflicts |

---

## Troubleshooting

### direnv not working

```bash
# Check direnv is installed
which direnv

# Check hook is in shell config
grep direnv ~/.bashrc  # or ~/.zshrc

# Reload shell
source ~/.bashrc

# Re-allow
direnv allow
```

### Nix flakes not enabled

```bash
# Check config
cat ~/.config/nix/nix.conf

# Should contain:
# experimental-features = nix-command flakes

# Restart Nix daemon (if multi-user)
sudo systemctl restart nix-daemon
```

### Tools not found

```bash
# Ensure you're in Nix shell
nix develop

# Or with direnv
direnv reload
```

### Python packages not installed

```bash
# Delete venv and re-enter
rm -rf .venv
nix develop  # or: direnv reload

# Packages install automatically
```

---

## Advanced Usage

### Run Single Command

```bash
# Run command in Nix environment without entering shell
nix develop -c just validate
```

### Update Single Tool

```bash
# Edit flake.nix, then:
nix flake update nixpkgs
```

### Pin to Specific Version

```nix
# In flake.nix, change:
nixpkgs.url = "github:NixOS/nixpkgs/nixos-23.11";  # Specific release
```

### Use in CI

```yaml
# .github/workflows/ci.yml
- uses: cachix/install-nix-action@v22
  with:
    nix_path: nixpkgs=channel:nixos-unstable
- run: nix develop -c just ci
```

---

## Files

```
ros2_zenoh/
├── flake.nix         # Nix environment definition
├── flake.lock        # Locked versions (commit this!)
├── .envrc            # direnv configuration
├── .venv/            # Python venv (auto-created, gitignored)
└── .direnv/          # direnv cache (gitignored)
```

---

## Why Nix?

**Reproducibility:**
- Exact same environment on every machine
- No "works on my machine" problems
- Lock file ensures versions don't drift

**Convenience:**
- One command setup (`direnv allow`)
- Automatic activation on `cd`
- Tools at correct versions

**Clean:**
- No global installs
- No conflicts between projects
- Remove `.direnv/` to clean everything

**Fast:**
- Native execution (no VM)
- Instant startup
- Binary caching (no compilation)

---

## Learn More

- Nix Manual: https://nixos.org/manual/nix/stable/
- Nix Flakes: https://nixos.wiki/wiki/Flakes
- direnv: https://direnv.net/
- Zero to Nix: https://zero-to-nix.com/

---

**Happy Nix-ing!** 🎉


