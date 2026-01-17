#!/bin/zsh
# Build MuJoCo for Apple platforms (iOS, tvOS, macOS)
# Usage: ./scripts/build_mujoco.sh [ios|tvos|macos|all|force-all|clean|xcode]

set -e

# Add common paths for tools (Homebrew, MacPorts, etc.)
export PATH="/opt/homebrew/bin:/usr/local/bin:/usr/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${0}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CMAKE_DIR="${PROJECT_ROOT}/cmake/mujoco"
BUILD_DIR="${PROJECT_ROOT}/build/mujoco"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo "[INFO] $1" >&2
}

log_warn() {
    echo "[WARN] $1" >&2
}

log_error() {
    echo "[ERROR] $1" >&2
}

log_progress() {
    # This shows in Xcode's build log
    echo "[MuJoCo Build] $1" >&2
}

# Check prerequisites
check_prerequisites() {
    if ! command -v cmake &> /dev/null; then
        log_error "CMake is not installed. Please install it via 'brew install cmake'"
        exit 1
    fi

    if [ ! -f "${PROJECT_ROOT}/third_party/mujoco/CMakeLists.txt" ]; then
        log_error "MuJoCo submodule not found. Run: git submodule update --init --recursive"
        exit 1
    fi
}

# Build for a specific platform
build_platform() {
    local PLATFORM=$1
    local PLATFORM_UPPER=$(echo "$PLATFORM" | tr '[:lower:]' '[:upper:]')
    local BUILD_FLAG="BUILD_${PLATFORM_UPPER}"
    local PLATFORM_BUILD_DIR="${BUILD_DIR}/${PLATFORM}"

    log_progress "Starting MuJoCo build for ${PLATFORM}..."

    # Create build directory
    mkdir -p "${PLATFORM_BUILD_DIR}"

    # Configure
    log_progress "Configuring CMake for ${PLATFORM}..."
    cmake -S "${CMAKE_DIR}" \
          -B "${PLATFORM_BUILD_DIR}" \
          -D${BUILD_FLAG}=ON \
          -DCMAKE_BUILD_TYPE=Release \
          -Wno-dev >&2

    # Build
    log_progress "Compiling MuJoCo for ${PLATFORM} (this may take a few minutes)..."
    cmake --build "${PLATFORM_BUILD_DIR}" --config Release --parallel >&2

    log_progress "MuJoCo for ${PLATFORM} built successfully!"
    log_info "Output: ${PROJECT_ROOT}/build/mujoco/${PLATFORM}/libmujoco.a"
}

# Build for Xcode (called from Aggregate target)
build_for_xcode() {
    # Xcode sets these environment variables
    local PLATFORM_NAME="${PLATFORM_NAME:-macosx}"

    case "$PLATFORM_NAME" in
        macosx)
            build_platform "macOS"
            ;;
        iphoneos|iphonesimulator)
            build_platform "iOS"
            ;;
        appletvos|appletvsimulator)
            build_platform "tvOS"
            ;;
        *)
            log_warn "Unknown platform: $PLATFORM_NAME, building for macOS"
            build_platform "macOS"
            ;;
    esac
}

# Clean build directory
clean_build() {
    log_info "Cleaning MuJoCo build directory..."
    rm -rf "${BUILD_DIR}"
    log_info "Clean complete."
}

# Main
main() {
    local TARGET="${1:-xcode}"

    case "$TARGET" in
        clean)
            clean_build
            ;;
        ios)
            check_prerequisites
            build_platform "iOS"
            ;;
        tvos)
            check_prerequisites
            build_platform "tvOS"
            ;;
        macos)
            check_prerequisites
            build_platform "macOS"
            ;;
        all)
            check_prerequisites
            log_info "Building MuJoCo for all platforms..."
            build_platform "iOS"
            build_platform "tvOS"
            build_platform "macOS"
            log_info "All platforms built successfully!"
            ;;
        force-all)
            check_prerequisites
            log_info "Force rebuilding MuJoCo for all platforms..."
            clean_build
            build_platform "iOS"
            build_platform "tvOS"
            build_platform "macOS"
            log_info "All platforms rebuilt successfully!"
            ;;
        xcode)
            check_prerequisites
            # Called from Xcode Aggregate target
            build_for_xcode
            ;;
        *)
            echo "Usage: $0 [ios|tvos|macos|all|force-all|clean|xcode]"
            echo ""
            echo "Options:"
            echo "  ios       - Build for iOS"
            echo "  tvos      - Build for tvOS"
            echo "  macos     - Build for macOS"
            echo "  all       - Build for all platforms"
            echo "  force-all - Clean and rebuild all platforms"
            echo "  clean     - Remove build directory"
            echo "  xcode     - Auto-detect platform from Xcode environment (default)"
            exit 1
            ;;
    esac
}

main "$@"
