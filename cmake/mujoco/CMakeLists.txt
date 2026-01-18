# MuJoCo Build Configuration for Apple Platforms (iOS, tvOS, macOS)
# Adapted from mujoco-ios approach - manually handles dependencies for cross-compilation
cmake_minimum_required(VERSION 3.24)

# Allow old cmake_minimum_required in third-party dependencies (tinyobjloader, etc.)
# CMake 4.x removed compatibility with CMake < 3.5
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# =============================================================================
# Platform Configuration (must be before project())
# =============================================================================

option(BUILD_IOS "Build for iOS" OFF)
option(BUILD_TVOS "Build for tvOS" OFF)
option(BUILD_MACOS "Build for macOS" OFF)
option(BUILD_SHARED_MUJOCO "Build MuJoCo as shared library (dylib)" OFF)

if(BUILD_IOS)
    set(CMAKE_SYSTEM_NAME iOS)
    set(CMAKE_OSX_ARCHITECTURES arm64)
    set(CMAKE_OSX_DEPLOYMENT_TARGET 18.0)
    set(MUJOCO_PLATFORM "iOS")
elseif(BUILD_TVOS)
    set(CMAKE_SYSTEM_NAME tvOS)
    set(CMAKE_OSX_ARCHITECTURES arm64)
    set(CMAKE_OSX_DEPLOYMENT_TARGET 18.0)
    set(MUJOCO_PLATFORM "tvOS")
elseif(BUILD_MACOS)
    set(CMAKE_SYSTEM_NAME Darwin)
    set(CMAKE_OSX_ARCHITECTURES arm64)
    set(CMAKE_OSX_DEPLOYMENT_TARGET 15.0)
    set(MUJOCO_PLATFORM "macOS")
else()
    message(FATAL_ERROR "Please specify platform: -DBUILD_IOS=ON, -DBUILD_TVOS=ON, or -DBUILD_MACOS=ON")
endif()

project(MuJoCoLib LANGUAGES C CXX)

# C/C++ Standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Paths
# =============================================================================

get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(MUJOCO_ROOT "${PROJECT_ROOT}/third_party/mujoco")
set(MUJOCO_SRC "${MUJOCO_ROOT}/src")
set(MUJOCO_INCLUDE "${MUJOCO_ROOT}/include")

# Verify MuJoCo exists
if(NOT EXISTS "${MUJOCO_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR "MuJoCo not found at ${MUJOCO_ROOT}. Run: git submodule update --init --recursive")
endif()

# Include directories
include_directories(
    ${MUJOCO_INCLUDE}
    ${MUJOCO_SRC}
    ${MUJOCO_ROOT}
)

# =============================================================================
# Compiler Flags
# =============================================================================

add_compile_options(
    -Wall
    -Wextra
    -fvisibility=hidden
    -fPIC
)

# Disable x86 SIMD on ARM
add_compile_definitions(
    mjUSEPLATFORMSIMD=0
    MUJOCO_DLL_EXPORTS
    _GNU_SOURCE
    CCD_STATIC_DEFINE
    MC_IMPLEM_ENABLE
)

# =============================================================================
# Third-Party Dependencies (manually built for cross-compilation)
# =============================================================================

include(FetchContent)

# --- LodePNG (manually built) ---
set(MUJOCO_DEP_VERSION_lodepng 17d08dd26cac4d63f43af217ebd70318bfb8189c)
FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG ${MUJOCO_DEP_VERSION_lodepng}
)
FetchContent_GetProperties(lodepng)
if(NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    add_library(lodepng STATIC
        ${lodepng_SOURCE_DIR}/lodepng.h
        ${lodepng_SOURCE_DIR}/lodepng.cpp
    )
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
endif()

# --- TinyXML2 ---
set(MUJOCO_DEP_VERSION_tinyxml2 e6caeae85799003f4ca74ff26ee16a789bc2af48)
set(tinyxml2_BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    tinyxml2
    GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
    GIT_TAG ${MUJOCO_DEP_VERSION_tinyxml2}
)
FetchContent_MakeAvailable(tinyxml2)

# --- TinyObjLoader ---
set(MUJOCO_DEP_VERSION_tinyobjloader 1421a10d6ed9742f5b2c1766d22faa6cfbc56248)
FetchContent_Declare(
    tinyobjloader
    GIT_REPOSITORY https://github.com/tinyobjloader/tinyobjloader.git
    GIT_TAG ${MUJOCO_DEP_VERSION_tinyobjloader}
)
FetchContent_MakeAvailable(tinyobjloader)

# --- Qhull (manually built - only static library, no executables) ---
set(MUJOCO_DEP_VERSION_qhull 62ccc56af071eaa478bef6ed41fd7a55d3bb2d80)
FetchContent_Declare(
    qhull
    GIT_REPOSITORY https://github.com/qhull/qhull.git
    GIT_TAG ${MUJOCO_DEP_VERSION_qhull}
)
FetchContent_GetProperties(qhull)
if(NOT qhull_POPULATED)
    FetchContent_Populate(qhull)

    set(libqhull_SOURCES
        ${qhull_SOURCE_DIR}/src/libqhull_r/global_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/stat_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/geom2_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/poly2_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/merge_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/libqhull_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/geom_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/poly_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/qset_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/mem_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/random_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/usermem_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/io_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/user_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/rboxlib_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/userprintf_r.c
        ${qhull_SOURCE_DIR}/src/libqhull_r/userprintf_rbox_r.c
    )
    add_library(qhullstatic_r STATIC ${libqhull_SOURCES})
    target_include_directories(qhullstatic_r PUBLIC
        ${qhull_SOURCE_DIR}/src
        ${qhull_SOURCE_DIR}/src/libqhull_r
    )
endif()

# --- libccd ---
set(MUJOCO_DEP_VERSION_ccd 7931e764a19ef6b21b443376c699bbc9c6d4fba8)
set(ENABLE_DOUBLE_PRECISION ON CACHE BOOL "" FORCE)
set(CCD_HIDE_ALL_SYMBOLS ON CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    ccd
    GIT_REPOSITORY https://github.com/danfis/libccd.git
    GIT_TAG ${MUJOCO_DEP_VERSION_ccd}
)
FetchContent_MakeAvailable(ccd)

# --- MarchingCubeCpp (header only) ---
set(MUJOCO_DEP_VERSION_MarchingCubeCpp f03a1b3ec29b1d7d865691ca8aea4f1eb2c2873d)
FetchContent_Declare(
    marchingcubecpp
    GIT_REPOSITORY https://github.com/aparis69/MarchingCubeCpp.git
    GIT_TAG ${MUJOCO_DEP_VERSION_MarchingCubeCpp}
)
FetchContent_GetProperties(marchingcubecpp)
if(NOT marchingcubecpp_POPULATED)
    FetchContent_Populate(marchingcubecpp)
    include_directories(${marchingcubecpp_SOURCE_DIR})
endif()

# --- TriangleMeshDistance (header only) ---
set(MUJOCO_DEP_VERSION_TriangleMeshDistance 2cb643de1436e1ba8e2be49b07ec5491ac604457)
FetchContent_Declare(
    trianglemeshdistance
    GIT_REPOSITORY https://github.com/InteractiveComputerGraphics/TriangleMeshDistance.git
    GIT_TAG ${MUJOCO_DEP_VERSION_TriangleMeshDistance}
)
FetchContent_GetProperties(trianglemeshdistance)
if(NOT trianglemeshdistance_POPULATED)
    FetchContent_Populate(trianglemeshdistance)
    include_directories(${trianglemeshdistance_SOURCE_DIR})
endif()

# =============================================================================
# MuJoCo 3.4.0 Engine Sources
# =============================================================================

set(ENGINE_SOURCES
    ${MUJOCO_SRC}/engine/engine_callback.c
    ${MUJOCO_SRC}/engine/engine_collision_box.c
    ${MUJOCO_SRC}/engine/engine_collision_convex.c
    ${MUJOCO_SRC}/engine/engine_collision_driver.c
    ${MUJOCO_SRC}/engine/engine_collision_gjk.c
    ${MUJOCO_SRC}/engine/engine_collision_primitive.c
    ${MUJOCO_SRC}/engine/engine_collision_sdf.c
    ${MUJOCO_SRC}/engine/engine_core_constraint.c
    ${MUJOCO_SRC}/engine/engine_core_smooth.c
    ${MUJOCO_SRC}/engine/engine_core_util.c
    ${MUJOCO_SRC}/engine/engine_crossplatform.cc
    ${MUJOCO_SRC}/engine/engine_derivative.c
    ${MUJOCO_SRC}/engine/engine_derivative_fd.c
    ${MUJOCO_SRC}/engine/engine_forward.c
    ${MUJOCO_SRC}/engine/engine_init.c
    ${MUJOCO_SRC}/engine/engine_inverse.c
    ${MUJOCO_SRC}/engine/engine_io.c
    ${MUJOCO_SRC}/engine/engine_island.c
    ${MUJOCO_SRC}/engine/engine_memory.c
    ${MUJOCO_SRC}/engine/engine_name.c
    ${MUJOCO_SRC}/engine/engine_passive.c
    ${MUJOCO_SRC}/engine/engine_plugin.cc
    ${MUJOCO_SRC}/engine/engine_print.c
    ${MUJOCO_SRC}/engine/engine_ray.c
    ${MUJOCO_SRC}/engine/engine_sensor.c
    ${MUJOCO_SRC}/engine/engine_setconst.c
    ${MUJOCO_SRC}/engine/engine_sleep.c
    ${MUJOCO_SRC}/engine/engine_solver.c
    ${MUJOCO_SRC}/engine/engine_support.c
    ${MUJOCO_SRC}/engine/engine_util_blas.c
    ${MUJOCO_SRC}/engine/engine_util_container.c
    ${MUJOCO_SRC}/engine/engine_util_errmem.c
    ${MUJOCO_SRC}/engine/engine_util_misc.c
    ${MUJOCO_SRC}/engine/engine_util_solve.c
    ${MUJOCO_SRC}/engine/engine_util_sparse.c
    ${MUJOCO_SRC}/engine/engine_util_spatial.c
    ${MUJOCO_SRC}/engine/engine_vis_init.c
    ${MUJOCO_SRC}/engine/engine_vis_interact.c
    ${MUJOCO_SRC}/engine/engine_vis_visualize.c
)

set(USER_SOURCES
    ${MUJOCO_SRC}/user/user_init.c
    ${MUJOCO_SRC}/user/user_api.cc
    ${MUJOCO_SRC}/user/user_cache.cc
    ${MUJOCO_SRC}/user/user_composite.cc
    ${MUJOCO_SRC}/user/user_flexcomp.cc
    ${MUJOCO_SRC}/user/user_mesh.cc
    ${MUJOCO_SRC}/user/user_model.cc
    ${MUJOCO_SRC}/user/user_objects.cc
    ${MUJOCO_SRC}/user/user_resource.cc
    ${MUJOCO_SRC}/user/user_util.cc
    ${MUJOCO_SRC}/user/user_vfs.cc
)

set(XML_SOURCES
    ${MUJOCO_SRC}/xml/xml.cc
    ${MUJOCO_SRC}/xml/xml_api.cc
    ${MUJOCO_SRC}/xml/xml_base.cc
    ${MUJOCO_SRC}/xml/xml_native_reader.cc
    ${MUJOCO_SRC}/xml/xml_native_writer.cc
    ${MUJOCO_SRC}/xml/xml_numeric_format.cc
    ${MUJOCO_SRC}/xml/xml_urdf.cc
    ${MUJOCO_SRC}/xml/xml_util.cc
)

set(THREAD_SOURCES
    ${MUJOCO_SRC}/thread/thread_pool.cc
    ${MUJOCO_SRC}/thread/thread_task.cc
)

# =============================================================================
# MuJoCo Library (Static or Shared)
# =============================================================================

if(BUILD_SHARED_MUJOCO)
    add_library(mujoco SHARED
        ${ENGINE_SOURCES}
        ${USER_SOURCES}
        ${XML_SOURCES}
        ${THREAD_SOURCES}
    )

    # For shared library, link dependencies statically (they get embedded)
    target_link_libraries(mujoco
        PRIVATE
        ccd
        lodepng
        qhullstatic_r
        tinyobjloader
        tinyxml2
    )

    # Shared library settings
    set_target_properties(mujoco PROPERTIES
        # Output location
        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_ROOT}/build/mujoco/${MUJOCO_PLATFORM}"
        # macOS/iOS framework settings
        FRAMEWORK TRUE
        FRAMEWORK_VERSION A
        MACOSX_FRAMEWORK_IDENTIFIER org.mujoco.mujoco
        MACOSX_FRAMEWORK_BUNDLE_VERSION 3.4.0
        MACOSX_FRAMEWORK_SHORT_VERSION_STRING 3.4.0
        # Install name for proper linking
        INSTALL_NAME_DIR "@rpath"
        BUILD_WITH_INSTALL_RPATH TRUE
        # Export all public symbols
        C_VISIBILITY_PRESET default
        CXX_VISIBILITY_PRESET default
        VISIBILITY_INLINES_HIDDEN OFF
    )

    # Copy headers into framework
    set_target_properties(mujoco PROPERTIES
        PUBLIC_HEADER "${MUJOCO_INCLUDE}/mujoco/mujoco.h"
    )

    set(MUJOCO_OUTPUT_NAME "libmujoco.dylib")
else()
    add_library(mujoco STATIC
        ${ENGINE_SOURCES}
        ${USER_SOURCES}
        ${XML_SOURCES}
        ${THREAD_SOURCES}
    )

    target_link_libraries(mujoco
        PRIVATE
        ccd
        lodepng
        qhullstatic_r
        tinyobjloader
        tinyxml2
    )

    # Set output directory
    set_target_properties(mujoco PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_ROOT}/build/mujoco/${MUJOCO_PLATFORM}"
    )

    set(MUJOCO_OUTPUT_NAME "libmujoco.a")
endif()

target_include_directories(mujoco PUBLIC
    ${MUJOCO_INCLUDE}
    ${MUJOCO_SRC}
)

# =============================================================================
# Print Configuration
# =============================================================================

message(STATUS "")
message(STATUS "MuJoCo ${MUJOCO_PLATFORM} Build Configuration:")
message(STATUS "  Platform: ${MUJOCO_PLATFORM}")
message(STATUS "  System: ${CMAKE_SYSTEM_NAME}")
message(STATUS "  Deployment Target: ${CMAKE_OSX_DEPLOYMENT_TARGET}")
message(STATUS "  Architecture: ${CMAKE_OSX_ARCHITECTURES}")
message(STATUS "  Library Type: $<IF:$<BOOL:${BUILD_SHARED_MUJOCO}>,SHARED,STATIC>")
message(STATUS "  MuJoCo Root: ${MUJOCO_ROOT}")
message(STATUS "  Output: ${PROJECT_ROOT}/build/mujoco/${MUJOCO_PLATFORM}/${MUJOCO_OUTPUT_NAME}")
message(STATUS "")
