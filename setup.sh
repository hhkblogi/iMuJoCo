#!/bin/zsh
# setup.sh - Set up Python environment using pyenv + virtualenv
# Requires: zsh (uses zsh-specific features)

set -e

# Colors (disable if not interactive terminal)
if [[ -t 1 ]]; then
    GREEN='\033[0;32m'
    YELLOW='\033[0;33m'
    RED='\033[0;31m'
    NC='\033[0m'
else
    GREEN=''
    YELLOW=''
    RED=''
    NC=''
fi

log_info() { echo "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo "${RED}[ERROR]${NC} $1" >&2; }

# Configuration
PYTHON_VERSION="3.14"
# Derive repository directory name, preferring git toplevel when available
if command -v git &> /dev/null && git rev-parse --show-toplevel &> /dev/null 2>&1; then
    REPO_DIR_NAME=$(basename "$(git rev-parse --show-toplevel)")
else
    REPO_DIR_NAME=$(basename "$(pwd)")
fi
# Allow overriding the virtualenv name via environment variable
VENV_NAME="${VENV_NAME:-${REPO_DIR_NAME}-venv}"

echo "=== Python Environment Setup ==="
echo "Python version: $PYTHON_VERSION"
echo "Virtualenv name: $VENV_NAME"
echo ""

# Check if pyenv is available
if ! command -v pyenv &> /dev/null; then
    log_error "pyenv is not installed"
    exit 1
fi

# Check if pyenv-virtualenv is available
if ! pyenv commands | grep -q virtualenv; then
    log_error "pyenv-virtualenv is not installed"
    log_info "On macOS: brew install pyenv-virtualenv"
    log_info "Other platforms: https://github.com/pyenv/pyenv-virtualenv#installation"
    exit 1
fi

# Find matching Python version (stable releases only, exclude alpha/beta)
INSTALLED_VERSION=$(pyenv versions --bare | grep "^${PYTHON_VERSION}\.[0-9]" | sort -t. -k1,1n -k2,2n -k3,3n | tail -1)
if [[ -z "$INSTALLED_VERSION" ]]; then
    log_error "Python $PYTHON_VERSION is not installed."
    log_info "Available stable versions matching $PYTHON_VERSION:"
    pyenv install --list | grep "^[[:space:]]*${PYTHON_VERSION}\.[0-9]" | head -5
    log_info "Install with: pyenv install $PYTHON_VERSION"
    exit 1
fi
log_info "Using Python: $INSTALLED_VERSION"

# Check if virtualenv already exists
if pyenv versions --bare | grep -q "^${VENV_NAME}$"; then
    log_info "Virtualenv '$VENV_NAME' already exists."

    # Check if .python-version is set correctly
    if [[ -f ".python-version" ]] && [[ "$(cat .python-version)" == "$VENV_NAME" ]]; then
        log_info "Local Python version already set to '$VENV_NAME'."
        echo ""
        echo "=== Already Set Up ==="
        log_info "To recreate, run: pyenv virtualenv-delete $VENV_NAME && ./setup.sh"
        exit 0
    else
        log_info "Setting local Python version to: $VENV_NAME"
        pyenv local "$VENV_NAME"
    fi
else
    # Create virtualenv
    log_info "Creating virtualenv: $VENV_NAME"
    pyenv virtualenv "$INSTALLED_VERSION" "$VENV_NAME"

    # Set local Python version
    log_info "Setting local Python version to: $VENV_NAME"
    pyenv local "$VENV_NAME"
fi

# Activate virtualenv for current script
export PYENV_VERSION="$VENV_NAME"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"

# Verify we're using the correct Python
CURRENT_PYTHON=$(which python)
if [[ "$CURRENT_PYTHON" != *"$VENV_NAME"* ]]; then
    log_warn "Python may not be from virtualenv: $CURRENT_PYTHON"
fi

# Install packages from pyproject.toml if it exists
if [[ -f "pyproject.toml" ]]; then
    echo ""
    log_info "Installing packages from pyproject.toml..."

    # Upgrade pip first
    if ! pip install --upgrade pip; then
        log_error "Failed to upgrade pip."
        exit 1
    fi

    # Install dev dependencies if defined
    if grep -Fq "[project.optional-dependencies]" pyproject.toml; then
        if ! pip install ".[dev]"; then
            log_error "Failed to install dev dependencies."
            exit 1
        fi
        # Note if this is a workspace with no Python packages
        if grep -Fq "packages = []" pyproject.toml; then
            log_info "Installed dev dependencies (no Python packages in this workspace)."
        fi
    elif grep -Fq "packages = []" pyproject.toml; then
        log_info "No Python packages or dev dependencies defined; skipping installation."
    else
        log_info "No [project.optional-dependencies] found; skipping dependency installation."
    fi
fi

echo ""
echo "=== Setup Complete ==="
log_info "Virtualenv '$VENV_NAME' is now active."
log_info "Python: $(python --version)"
log_info "Location: $(which python)"
echo ""
log_info "The virtualenv will auto-activate when you cd into this directory."
log_info "Note: Ensure pyenv is initialized in your shell config (~/.zshrc):"
echo "  eval \"\$(pyenv init -)\""
echo "  eval \"\$(pyenv virtualenv-init -)\""
