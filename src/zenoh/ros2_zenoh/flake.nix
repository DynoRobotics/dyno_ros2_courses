{
  description = "ros2-zenoh development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        
        # Python environment with required packages
        pythonEnv = pkgs.python311.withPackages (ps: with ps; [
          pip
          virtualenv
          pyyaml
          # Test tools installed via pip in shellHook
        ]);
      in
      {
        devShells.default = pkgs.mkShell {
          name = "ros2-zenoh-dev";
          
          buildInputs = with pkgs; [
            # Python
            pythonEnv
            
            # Rust-based tools (fast!)
            just              # Task runner
            typos             # Spell checker
            
            # Markdown/YAML tools
            markdownlint-cli2 # Markdown linting
            nodePackages.yaml-language-server
            
            # Development tools
            git
            pre-commit
            
            # Optional but useful
            ripgrep           # Fast search
            fd                # Fast find
            bat               # Better cat
            
            # For building Python packages
            pkg-config
            openssl
          ];

          shellHook = ''
            # Create virtual environment if it doesn't exist
            if [ ! -d .venv ]; then
              echo "Creating Python virtual environment..."
              python -m venv .venv
            fi
            
            # Activate virtual environment
            source .venv/bin/activate
            
            # Upgrade pip
            pip install --upgrade pip --quiet
            
            # Install Python development tools
            if ! command -v ruff &> /dev/null; then
              echo "Installing Python tools (ruff, mypy, pytest, pre-commit)..."
              pip install ruff mypy pytest pytest-asyncio pytest-cov pre-commit pyyaml --quiet
            fi
            
            # Install pre-commit hooks if not already done
            if [ ! -f .git/hooks/pre-commit ]; then
              echo "Installing pre-commit hooks..."
              pre-commit install
            fi
            
            # Display welcome message
            echo ""
            echo "╔══════════════════════════════════════════════════════════════╗"
            echo "║  ros2-zenoh Development Environment (Nix)                   ║"
            echo "╚══════════════════════════════════════════════════════════════╝"
            echo ""
            echo "Tools available:"
            echo "  • just              - Task runner ($(just --version))"
            echo "  • ruff              - Python linter/formatter"
            echo "  • typos             - Spell checker"
            echo "  • markdownlint      - Markdown linter"
            echo "  • pre-commit        - Git hooks"
            echo ""
            echo "Quick start:"
            echo "  just --list         # Show all commands"
            echo "  just validate       # Run all validations"
            echo "  just test           # Run tests"
            echo "  just fmt            # Format code"
            echo ""
            echo "Documentation:"
            echo "  SETUP.md            # Setup guide"
            echo "  README.md           # Project overview"
            echo "  docs/               # Obsidian vault"
            echo ""
          '';

          # Environment variables
          PYTHON_VENV = ".venv";
          
          # Ensure Python finds packages
          PYTHONPATH = "./python:$PYTHONPATH";
          
          # For colored output
          FORCE_COLOR = "1";
        };

        # Packages that can be built with `nix build`
        packages = {
          # Future: Add package builds here
        };
      }
    );
}


