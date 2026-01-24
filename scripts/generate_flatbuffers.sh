#!/bin/zsh
# Generate FlatBuffers C++ headers from schema files
# Usage: ./scripts/generate_flatbuffers.sh [output_dir]
#
# Arguments:
#   output_dir    Where to generate C++ headers (default: uses FLATBUFFERS_OUTPUT_DIR
#                 env var, or schema/generated/cpp if not set)
#
# Environment variables (set by Xcode Run Script phase):
#   FLATBUFFERS_OUTPUT_DIR  Output directory for generated headers
#   SRCROOT                 Project source root (Xcode provides this)
#
# Requirements: Xcode Command Line Tools (provides cmake via xcrun)

set -e

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

CMAKE=$(find_cmake)

# Apply patches to third-party dependencies
apply_patches() {
    if [[ ! -d "${FLATBUFFERS_PATCHES}" ]]; then
        return 0
    fi

    local patches=("${FLATBUFFERS_PATCHES}"/flatbuffers-*.patch(N))
    if [[ ${#patches[@]} -eq 0 ]]; then
        return 0
    fi

    log_info "Applying patches to flatbuffers..."
    for patch in "${patches[@]}"; do
        local patch_name=$(basename "${patch}")
        # Check if patch is already applied (git apply --check returns 0 if would apply cleanly)
        if git -C "${FLATBUFFERS_DIR}" apply --check --reverse "${patch}" 2>/dev/null; then
            log_info "  ${patch_name} (already applied)"
        else
            log_info "  ${patch_name}"
            git -C "${FLATBUFFERS_DIR}" apply "${patch}"
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
    local fbs_files=("${SCHEMA_DIR}"/*.fbs(N))

    if [[ ${#fbs_files[@]} -eq 0 ]]; then
        log_warn "No .fbs files found in ${SCHEMA_DIR}"
        return 0
    fi

    for fbs_file in "${fbs_files[@]}"; do
        local basename=$(basename "${fbs_file}")
        log_info "  Generating from ${basename}..."
        "${FLATC}" --cpp --scoped-enums -o "${NAMESPACE_DIR}" "${fbs_file}"
    done

    log_info "Generated headers in ${NAMESPACE_DIR}:"
    ls -la "${NAMESPACE_DIR}"
}

# Main
main() {
    log_info "FlatBuffers Schema Generator"
    log_info "Project root: ${PROJECT_ROOT}"

    apply_patches
    build_flatc
    generate_cpp

    log_info "Done!"
}

main "$@"
