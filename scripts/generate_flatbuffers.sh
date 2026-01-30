#!/bin/zsh
# Generate FlatBuffers C++ headers from schema files
# Usage: ./scripts/generate_flatbuffers.sh [output_dir]
#
# Arguments:
#   output_dir    Where to generate C++ headers (default: uses FLATBUFFERS_OUTPUT_DIR
#                 env var, or build/generated/flatbuffers if not set)
#
# Environment variables (set by Xcode Run Script phase):
#   FLATBUFFERS_OUTPUT_DIR  Output directory for generated headers
#   SRCROOT                 Project source root (Xcode provides this)
#
# Requirements: Xcode Command Line Tools (provides cmake via xcrun)

set -e

# Add Homebrew paths for Xcode build phase (Xcode doesn't source shell profile)
export PATH="/opt/homebrew/bin:/usr/local/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${0}")" && pwd)"
# Always use script location to find project root (works from Xcode or CLI)
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
FLATBUFFERS_DIR="${PROJECT_ROOT}/third_party/flatbuffers"
FLATBUFFERS_PATCHES="${PROJECT_ROOT}/third_party/patches"
FLATC_BUILD_DIR="${PROJECT_ROOT}/build/flatc"
FLATC="${FLATC_BUILD_DIR}/flatc"
SCHEMA_DIR="${PROJECT_ROOT}/schema"

# Output directory: 1) command line arg, 2) env var, 3) default (build/generated/flatbuffers)
OUTPUT_DIR="${1:-${FLATBUFFERS_OUTPUT_DIR:-${PROJECT_ROOT}/build/generated/flatbuffers}}"

# Clear Xcode environment variables that interfere with CMake cross-compilation
unset SDKROOT
unset ARCHS
unset PLATFORM_NAME
unset IPHONEOS_DEPLOYMENT_TARGET
unset TVOS_DEPLOYMENT_TARGET
unset MACOSX_DEPLOYMENT_TARGET
unset CONFIGURATION
unset BUILT_PRODUCTS_DIR
unset TARGET_BUILD_DIR

# Colors (disable if not interactive terminal)
if [[ -t 1 ]]; then
    GREEN='\033[0;32m'
    YELLOW='\033[0;33m'
    NC='\033[0m'
else
    GREEN=''
    YELLOW=''
    NC=''
fi

log_info() {
    echo "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo "${YELLOW}[WARN]${NC} $1"
}

# Check if flatbuffers submodule is initialized
check_submodule() {
    if [[ ! -d "${FLATBUFFERS_DIR}" ]] || [[ ! -f "${FLATBUFFERS_DIR}/CMakeLists.txt" ]]; then
        echo "ERROR: FlatBuffers submodule not initialized." >&2
        echo "Run: git submodule update --init --recursive" >&2
        exit 1
    fi
}

# Find cmake - prefer Xcode's bundled cmake, fallback to PATH
find_cmake() {
    # Try Xcode's cmake first (no Homebrew dependency)
    local xcode_cmake
    xcode_cmake=$(xcrun --find cmake 2>/dev/null) && [[ -x "$xcode_cmake" ]] && {
        echo "$xcode_cmake"
        return 0
    }
    # Fallback to PATH
    command -v cmake 2>/dev/null && return 0
    echo "ERROR: cmake not found. Install Xcode Command Line Tools: xcode-select --install" >&2
    return 1
}

if ! CMAKE=$(find_cmake); then
    echo "ERROR: Failed to locate cmake. Aborting." >&2
    exit 1
fi

# Apply patches to third-party dependencies
apply_patches() {
    if [[ ! -d "${FLATBUFFERS_PATCHES}" ]]; then
        return 0
    fi

    # (N) is a zsh glob qualifier: if no files match, the pattern expands to an empty array
    local patches=("${FLATBUFFERS_PATCHES}"/flatbuffers-*.patch(N))
    if [[ ${#patches[@]} -eq 0 ]]; then
        return 0
    fi

    log_info "Applying patches to flatbuffers..."
    for patch in "${patches[@]}"; do
        local patch_name=$(basename "${patch}")
        # Check if patch is already applied (reverse check succeeds if patch can be reversed)
        if git -C "${FLATBUFFERS_DIR}" apply --check --reverse "${patch}" 2>/dev/null; then
            log_info "  ${patch_name} (already applied)"
        # If not already applied, ensure it can be applied cleanly in the forward direction
        elif git -C "${FLATBUFFERS_DIR}" apply --check "${patch}" 2>/dev/null; then
            log_info "  ${patch_name}"
            git -C "${FLATBUFFERS_DIR}" apply "${patch}"
        else
            log_warn "  ${patch_name} (cannot apply cleanly, skipping)"
        fi
    done
}

# Build flatc from submodule if not already built
build_flatc() {
    if [[ -x "${FLATC}" ]]; then
        log_info "flatc already built at ${FLATC}"
        return 0
    fi

    # Check if we're running inside Xcode's sandbox (can't build flatc there)
    if [[ -n "${XCODE_VERSION_ACTUAL:-}" ]]; then
        echo "ERROR: flatc not found. Run this first from terminal:" >&2
        echo "  ./scripts/generate_flatbuffers.sh" >&2
        echo "This will build flatc. Then retry your Xcode build." >&2
        exit 1
    fi

    log_info "Building flatc from submodule..."

    mkdir -p "${FLATC_BUILD_DIR}"

    "$CMAKE" -S "${FLATBUFFERS_DIR}" -B "${FLATC_BUILD_DIR}" \
        -DFLATBUFFERS_BUILD_TESTS=OFF \
        -DFLATBUFFERS_BUILD_FLATLIB=OFF \
        -DFLATBUFFERS_BUILD_FLATC=ON \
        -DCMAKE_BUILD_TYPE=Release

    # Use --parallel without count - CMake auto-detects available cores
    "$CMAKE" --build "${FLATC_BUILD_DIR}" --target flatc --parallel

    log_info "flatc built successfully"
}

# Generate C++ headers from .fbs files
# Output mirrors namespace: imujoco.schema -> imujoco/schema/
generate_cpp() {
    log_info "Generating C++ headers from schema files..."

    # Create namespace-mirrored directory structure
    local NAMESPACE_DIR="${OUTPUT_DIR}/imujoco/schema"
    mkdir -p "${NAMESPACE_DIR}"

    # Find all .fbs files and generate headers
    # (N) is a zsh glob qualifier: if no files match, the pattern expands to an empty array
    local fbs_files=("${SCHEMA_DIR}"/*.fbs(N))

    if [[ ${#fbs_files[@]} -eq 0 ]]; then
        log_warn "No .fbs files found in ${SCHEMA_DIR}"
        return 0
    fi

    for fbs_file in "${fbs_files[@]}"; do
        local schema_name=$(basename "${fbs_file}")
        log_info "  Generating from ${schema_name}..."
        "${FLATC}" --cpp --scoped-enums -o "${NAMESPACE_DIR}" "${fbs_file}"
    done

    log_info "Generated headers in ${NAMESPACE_DIR}"
    if [[ -t 1 ]]; then
        ls -la "${NAMESPACE_DIR}"
    fi
}

# Main
main() {
    log_info "FlatBuffers Schema Generator"
    log_info "Project root: ${PROJECT_ROOT}"

    check_submodule
    apply_patches
    build_flatc
    generate_cpp

    log_info "Done!"
}

main "$@"
