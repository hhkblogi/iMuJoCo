#!/bin/zsh
# Build MuJoCo XCFramework for Apple platforms
# Usage: ./scripts/build_mujoco_xcframework.sh [--static]
#
# Options:
#   --static     Build as static framework (merged static libraries)
#   (default)    Build as dynamic framework (dylib with deps bundled inside)

set -e

# Add Homebrew paths for cmake (Xcode's shell doesn't have the same PATH)
export PATH="/opt/homebrew/bin:/usr/local/bin:$PATH"

# Clear Xcode environment variables that interfere with CMake cross-compilation
# This is needed when running from Xcode's Run Script build phase
unset SDKROOT
unset ARCHS
unset PLATFORM_NAME
unset IPHONEOS_DEPLOYMENT_TARGET
unset TVOS_DEPLOYMENT_TARGET
unset MACOSX_DEPLOYMENT_TARGET
unset CONFIGURATION
unset BUILT_PRODUCTS_DIR
unset TARGET_BUILD_DIR

SCRIPT_DIR="$(cd "$(dirname "${0}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build/mujoco"
FRAMEWORK_DIR="${PROJECT_ROOT}/build/frameworks"
MUJOCO_INCLUDE="${PROJECT_ROOT}/third_party/mujoco/include"
CMAKE_DIR="${PROJECT_ROOT}/cmake/mujoco"

# Parse arguments - dynamic is now default
BUILD_SHARED=ON
if [[ "$1" == "--static" ]]; then
    BUILD_SHARED=OFF
fi

# Colors
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m'

log_info() {
    echo "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo "${RED}[ERROR]${NC} $1"
}

# Build MuJoCo for a specific platform
build_mujoco() {
    local PLATFORM=$1
    local PLATFORM_LOWER=$(echo ${PLATFORM} | tr '[:upper:]' '[:lower:]')
    local PLATFORM_BUILD_DIR="${BUILD_DIR}/${PLATFORM}"

    log_info "Building MuJoCo for ${PLATFORM} (shared=${BUILD_SHARED})..."

    # Set platform flag
    local PLATFORM_FLAG=""
    case ${PLATFORM} in
        macOS)         PLATFORM_FLAG="-DBUILD_MACOS=ON" ;;
        iOS)           PLATFORM_FLAG="-DBUILD_IOS=ON" ;;
        iOSSimulator)  PLATFORM_FLAG="-DBUILD_IOS_SIMULATOR=ON" ;;
        tvOS)          PLATFORM_FLAG="-DBUILD_TVOS=ON" ;;
        tvOSSimulator) PLATFORM_FLAG="-DBUILD_TVOS_SIMULATOR=ON" ;;
    esac

    # Configure
    cmake -S "${CMAKE_DIR}" -B "${PLATFORM_BUILD_DIR}" \
        ${PLATFORM_FLAG} \
        -DBUILD_SHARED_MUJOCO=${BUILD_SHARED} \
        -DCMAKE_BUILD_TYPE=Release

    # Build
    cmake --build "${PLATFORM_BUILD_DIR}" --config Release -j$(sysctl -n hw.ncpu)
}

# Create static framework (merges all .a files)
create_static_framework() {
    local PLATFORM=$1
    local FRAMEWORK_PATH="${FRAMEWORK_DIR}/${PLATFORM}/mujoco.framework"

    log_info "Creating static framework for ${PLATFORM}..."

    # Clean and create directory structure
    rm -rf "${FRAMEWORK_PATH}"
    mkdir -p "${FRAMEWORK_PATH}/Headers"
    mkdir -p "${FRAMEWORK_PATH}/Modules"

    # Merge all static libraries into one fat library
    log_info "Merging static libraries for ${PLATFORM}..."
    libtool -static -o "${FRAMEWORK_PATH}/mujoco" \
        "${BUILD_DIR}/${PLATFORM}/libmujoco.a" \
        "${BUILD_DIR}/${PLATFORM}/liblodepng.a" \
        "${BUILD_DIR}/${PLATFORM}/libqhullstatic_r.a" \
        "${BUILD_DIR}/${PLATFORM}/_deps/ccd-build/src/libccd.a" \
        "${BUILD_DIR}/${PLATFORM}/_deps/tinyxml2-build/libtinyxml2.a" \
        "${BUILD_DIR}/${PLATFORM}/_deps/tinyobjloader-build/libtinyobjloader.a"

    # Also create libmujoco.a for linker -l flag compatibility
    cp "${FRAMEWORK_PATH}/mujoco" "${FRAMEWORK_PATH}/libmujoco.a"

    # Copy headers
    cp -R "${MUJOCO_INCLUDE}/mujoco" "${FRAMEWORK_PATH}/Headers/"

    # Create module map
    cat > "${FRAMEWORK_PATH}/Modules/module.modulemap" << 'EOF'
framework module mujoco {
    header "mujoco/mujoco.h"
    export *
}
EOF

    # Create Info.plist
    cat > "${FRAMEWORK_PATH}/Info.plist" << EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleName</key>
    <string>mujoco</string>
    <key>CFBundleIdentifier</key>
    <string>org.mujoco.mujoco</string>
    <key>CFBundleVersion</key>
    <string>3.4.0</string>
    <key>CFBundleShortVersionString</key>
    <string>3.4.0</string>
    <key>CFBundlePackageType</key>
    <string>FMWK</string>
</dict>
</plist>
EOF

    log_info "Static framework created at ${FRAMEWORK_PATH}"
}

# Create dynamic framework (dylib with deps linked inside)
# macOS uses versioned bundles, iOS/tvOS use shallow bundles
create_dynamic_framework() {
    local PLATFORM=$1
    local FRAMEWORK_PATH="${FRAMEWORK_DIR}/${PLATFORM}/mujoco.framework"
    local DYLIB_PATH="${BUILD_DIR}/${PLATFORM}/mujoco.framework/mujoco"

    log_info "Creating dynamic framework for ${PLATFORM}..."

    # Clean framework directory
    rm -rf "${FRAMEWORK_PATH}"

    # macOS requires versioned bundle structure
    if [[ "${PLATFORM}" == "macOS" ]]; then
        create_versioned_framework "${PLATFORM}" "${FRAMEWORK_PATH}" "${DYLIB_PATH}"
    else
        create_shallow_framework "${PLATFORM}" "${FRAMEWORK_PATH}" "${DYLIB_PATH}"
    fi

    log_info "Dynamic framework created at ${FRAMEWORK_PATH}"
}

# Create shallow bundle (iOS/tvOS style)
create_shallow_framework() {
    local PLATFORM=$1
    local FRAMEWORK_PATH=$2
    local DYLIB_PATH=$3

    mkdir -p "${FRAMEWORK_PATH}/Headers"
    mkdir -p "${FRAMEWORK_PATH}/Modules"

    # Copy the dylib from CMake build
    copy_dylib "${PLATFORM}" "${DYLIB_PATH}" "${FRAMEWORK_PATH}/mujoco"

    # Fix install name
    install_name_tool -id "@rpath/mujoco.framework/mujoco" "${FRAMEWORK_PATH}/mujoco"

    # Copy headers
    cp -R "${MUJOCO_INCLUDE}/mujoco" "${FRAMEWORK_PATH}/Headers/"

    # Create module map
    cat > "${FRAMEWORK_PATH}/Modules/module.modulemap" << 'EOF'
framework module mujoco {
    header "mujoco/mujoco.h"
    link "mujoco"
    export *
}
EOF

    # Create Info.plist
    create_info_plist "${FRAMEWORK_PATH}/Info.plist"

    # Sign the framework (ad-hoc for local dev)
    codesign --force --sign - "${FRAMEWORK_PATH}/mujoco"
}

# Create versioned bundle (macOS style)
create_versioned_framework() {
    local PLATFORM=$1
    local FRAMEWORK_PATH=$2
    local DYLIB_PATH=$3

    # Create versioned directory structure
    mkdir -p "${FRAMEWORK_PATH}/Versions/A/Headers"
    mkdir -p "${FRAMEWORK_PATH}/Versions/A/Modules"
    mkdir -p "${FRAMEWORK_PATH}/Versions/A/Resources"

    # Copy the dylib from CMake build
    copy_dylib "${PLATFORM}" "${DYLIB_PATH}" "${FRAMEWORK_PATH}/Versions/A/mujoco"

    # Fix install name for versioned bundle
    install_name_tool -id "@rpath/mujoco.framework/Versions/A/mujoco" "${FRAMEWORK_PATH}/Versions/A/mujoco"

    # Copy headers
    cp -R "${MUJOCO_INCLUDE}/mujoco" "${FRAMEWORK_PATH}/Versions/A/Headers/"

    # Create module map
    cat > "${FRAMEWORK_PATH}/Versions/A/Modules/module.modulemap" << 'EOF'
framework module mujoco {
    header "mujoco/mujoco.h"
    link "mujoco"
    export *
}
EOF

    # Create Info.plist in Resources
    create_info_plist "${FRAMEWORK_PATH}/Versions/A/Resources/Info.plist"

    # Create symlinks
    ln -sfh A "${FRAMEWORK_PATH}/Versions/Current"
    ln -sfh Versions/Current/mujoco "${FRAMEWORK_PATH}/mujoco"
    ln -sfh Versions/Current/Headers "${FRAMEWORK_PATH}/Headers"
    ln -sfh Versions/Current/Modules "${FRAMEWORK_PATH}/Modules"
    ln -sfh Versions/Current/Resources "${FRAMEWORK_PATH}/Resources"

    # Sign the framework (ad-hoc for local dev)
    codesign --force --sign - "${FRAMEWORK_PATH}/Versions/A/mujoco"
}

# Helper: Copy dylib with fallback locations
copy_dylib() {
    local PLATFORM=$1
    local DYLIB_PATH=$2
    local DEST=$3

    if [[ -f "${DYLIB_PATH}" ]]; then
        cp "${DYLIB_PATH}" "${DEST}"
    else
        # Fallback: look for libmujoco.dylib in alternative locations
        log_warn "Primary dylib not found at ${DYLIB_PATH}, trying fallback locations..."
        local FALLBACK1="${BUILD_DIR}/${PLATFORM}/libmujoco.dylib"
        local FALLBACK2="${BUILD_DIR}/${PLATFORM}/mujoco.framework/Versions/A/mujoco"

        if [[ -f "${FALLBACK1}" ]]; then
            log_info "Using fallback: ${FALLBACK1}"
            cp "${FALLBACK1}" "${DEST}"
        elif [[ -f "${FALLBACK2}" ]]; then
            log_info "Using fallback: ${FALLBACK2}"
            cp "${FALLBACK2}" "${DEST}"
        fi
    fi

    # Verify dylib was copied successfully
    if [[ ! -f "${DEST}" ]]; then
        log_error "Failed to locate MuJoCo dylib for ${PLATFORM}"
        log_error "Expected at: ${DYLIB_PATH}"
        exit 1
    fi
}

# Helper: Create Info.plist
create_info_plist() {
    local PLIST_PATH=$1
    cat > "${PLIST_PATH}" << EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleName</key>
    <string>mujoco</string>
    <key>CFBundleIdentifier</key>
    <string>org.mujoco.mujoco</string>
    <key>CFBundleVersion</key>
    <string>3.4.0</string>
    <key>CFBundleShortVersionString</key>
    <string>3.4.0</string>
    <key>CFBundlePackageType</key>
    <string>FMWK</string>
    <key>CFBundleExecutable</key>
    <string>mujoco</string>
</dict>
</plist>
EOF
}

# Main
main() {
    if [[ "${BUILD_SHARED}" == "ON" ]]; then
        log_info "Building MuJoCo Dynamic XCFramework..."
    else
        log_info "Building MuJoCo Static XCFramework..."
    fi

    # Build for all platforms
    for PLATFORM in macOS iOS iOSSimulator tvOS tvOSSimulator; do
        build_mujoco "${PLATFORM}"
    done

    # Create frameworks for each platform
    for PLATFORM in macOS iOS iOSSimulator tvOS tvOSSimulator; do
        if [[ "${BUILD_SHARED}" == "ON" ]]; then
            create_dynamic_framework "${PLATFORM}"
        else
            create_static_framework "${PLATFORM}"
        fi
    done

    # Create XCFramework
    log_info "Creating XCFramework..."
    rm -rf "${FRAMEWORK_DIR}/mujoco.xcframework"

    xcodebuild -create-xcframework \
        -framework "${FRAMEWORK_DIR}/macOS/mujoco.framework" \
        -framework "${FRAMEWORK_DIR}/iOS/mujoco.framework" \
        -framework "${FRAMEWORK_DIR}/iOSSimulator/mujoco.framework" \
        -framework "${FRAMEWORK_DIR}/tvOS/mujoco.framework" \
        -framework "${FRAMEWORK_DIR}/tvOSSimulator/mujoco.framework" \
        -output "${FRAMEWORK_DIR}/mujoco.xcframework"

    log_info "XCFramework created at ${FRAMEWORK_DIR}/mujoco.xcframework"

    if [[ "${BUILD_SHARED}" == "ON" ]]; then
        log_info "Dynamic framework - add to 'Embed & Sign' in Xcode"
    else
        log_info "Static framework - add to 'Do Not Embed' in Xcode"
    fi
}

main "$@"
