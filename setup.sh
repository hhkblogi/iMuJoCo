#!/bin/zsh
# setup.sh - Set up Python environment using pyenv + virtualenv
# Requires: zsh (uses zsh-specific features)

set -e

# Configuration
PYTHON_VERSION="3.14"
REPO_DIR_NAME=$(basename "$(pwd)")
VENV_NAME="${REPO_DIR_NAME}-venv"

echo "=== Python Environment Setup ==="
echo "Python version: $PYTHON_VERSION"
echo "Virtualenv name: $VENV_NAME"
echo ""

# Check if pyenv is available
if ! command -v pyenv &> /dev/null; then
    echo "Error: pyenv is not installed"
    exit 1
fi

# Check if pyenv-virtualenv is available
if ! pyenv commands | grep -q virtualenv; then
    echo "Error: pyenv-virtualenv is not installed"
    echo "Install with: brew install pyenv-virtualenv (macOS)"
    echo "See: https://github.com/pyenv/pyenv-virtualenv#installation"
    exit 1
fi

# Find matching Python version
INSTALLED_VERSION=$(pyenv versions --bare | grep "^${PYTHON_VERSION}" | sort -V | tail -1)
if [[ -z "$INSTALLED_VERSION" ]]; then
    echo "Python $PYTHON_VERSION is not installed."
    echo "Available versions matching $PYTHON_VERSION:"
    pyenv install --list | grep "^[[:space:]]*${PYTHON_VERSION}" | head -5
    echo ""
    echo "Install with: pyenv install $PYTHON_VERSION"
    exit 1
fi
echo "Using Python: $INSTALLED_VERSION"

# Check if virtualenv already exists
if pyenv versions --bare | grep -q "^${VENV_NAME}$"; then
    echo "Virtualenv '$VENV_NAME' already exists."

    # Check if .python-version is set correctly
    if [[ -f ".python-version" ]] && [[ "$(cat .python-version)" == "$VENV_NAME" ]]; then
        echo "Local Python version already set to '$VENV_NAME'."
        echo ""
        echo "=== Already Set Up ==="
        echo "To recreate, run: pyenv virtualenv-delete $VENV_NAME && ./setup.sh"
        exit 0
    else
        echo "Setting local Python version to: $VENV_NAME"
        pyenv local "$VENV_NAME"
    fi
else
    # Create virtualenv
    echo "Creating virtualenv: $VENV_NAME"
    pyenv virtualenv "$INSTALLED_VERSION" "$VENV_NAME"

    # Set local Python version
    echo "Setting local Python version to: $VENV_NAME"
    pyenv local "$VENV_NAME"
fi

# Activate virtualenv for current script
export PYENV_VERSION="$VENV_NAME"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"

# Install packages from pyproject.toml if it exists
if [[ -f "pyproject.toml" ]]; then
    echo ""
    echo "Installing packages from pyproject.toml..."

    # Upgrade pip first
    pip install --upgrade pip || { echo "Error: Failed to upgrade pip." >&2; exit 1; }

    # Install dev dependencies if defined (no Python packages in this workspace)
    if grep -Fq "[project.optional-dependencies]" pyproject.toml; then
        pip install ".[dev]" || { echo "Error: Failed to install dev dependencies." >&2; exit 1; }
    else
        echo "No [project.optional-dependencies] found; skipping dependency installation."
    fi
fi

echo ""
echo "=== Setup Complete ==="
echo "Virtualenv '$VENV_NAME' is now active."
echo "Python: $(python --version)"
echo "Location: $(which python)"
echo ""
echo "The virtualenv will auto-activate when you cd into this directory."
echo "Note: Ensure pyenv is initialized in your shell config (~/.zshrc):"
echo "  eval \"\$(pyenv init -)\""
echo "  eval \"\$(pyenv virtualenv-init -)\""
