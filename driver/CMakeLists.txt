cmake_minimum_required(VERSION 3.20)
project(mujoco_driver VERSION 0.1.0 LANGUAGES CXX)

# Require C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(MUJOCO_DRIVER_BUILD_PYTHON "Build Python bindings" ON)
option(MUJOCO_DRIVER_BUILD_TESTS "Build unit tests" ON)
option(MUJOCO_DRIVER_BUILD_EXAMPLES "Build examples" ON)

# Find dependencies
find_package(Threads REQUIRED)

include(FetchContent)

# FlatBuffers
find_package(FlatBuffers QUIET)
if(NOT FlatBuffers_FOUND)
    FetchContent_Declare(
        flatbuffers
        GIT_REPOSITORY https://github.com/google/flatbuffers.git
        GIT_TAG v24.3.25
    )
    set(FLATBUFFERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(flatbuffers)
endif()

# Generate FlatBuffers headers
# Schema is at workspace root level (works whether built from driver/ or workspace root)
if(EXISTS "${CMAKE_SOURCE_DIR}/schema")
    set(SCHEMA_DIR "${CMAKE_SOURCE_DIR}/schema")
else()
    set(SCHEMA_DIR "${CMAKE_SOURCE_DIR}/../schema")
endif()
set(GENERATED_DIR "${CMAKE_BINARY_DIR}/generated")
file(MAKE_DIRECTORY ${GENERATED_DIR})

# Custom command to generate FlatBuffers headers
# --gen-object-api generates mutable ControlPacketT and StatePacketT types
add_custom_command(
    OUTPUT
        ${GENERATED_DIR}/control_generated.h
        ${GENERATED_DIR}/state_generated.h
    COMMAND $<TARGET_FILE:flatc> --cpp --gen-object-api -o ${GENERATED_DIR} ${SCHEMA_DIR}/control.fbs
    COMMAND $<TARGET_FILE:flatc> --cpp --gen-object-api -o ${GENERATED_DIR} ${SCHEMA_DIR}/state.fbs
    DEPENDS
        ${SCHEMA_DIR}/control.fbs
        ${SCHEMA_DIR}/state.fbs
        flatc
    COMMENT "Generating FlatBuffers C++ headers with Object API"
)

add_custom_target(generate_flatbuffers
    DEPENDS
        ${GENERATED_DIR}/control_generated.h
        ${GENERATED_DIR}/state_generated.h
)

# Core library
add_library(mujoco_driver
    cc/src/driver.cc
    cc/src/fragment.cc
)

add_dependencies(mujoco_driver generate_flatbuffers)

target_include_directories(mujoco_driver
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cc/include>
        $<BUILD_INTERFACE:${GENERATED_DIR}>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/cc/src
)

target_link_libraries(mujoco_driver
    PUBLIC
        Threads::Threads
        flatbuffers
)

# Python bindings
if(MUJOCO_DRIVER_BUILD_PYTHON)
    set(MUJOCO_DRIVER_PY_BINDINGS
        ${CMAKE_CURRENT_SOURCE_DIR}/python/mujoco_driver/bindings.cc
    )
    set(MUJOCO_DRIVER_PY_INIT
        ${CMAKE_CURRENT_SOURCE_DIR}/python/mujoco_driver/__init__.py
    )

    if(EXISTS "${MUJOCO_DRIVER_PY_BINDINGS}" AND EXISTS "${MUJOCO_DRIVER_PY_INIT}")
        # Prefer uv-managed Python from workspace .venv
        if(EXISTS "${CMAKE_SOURCE_DIR}/.venv")
            set(Python3_ROOT_DIR "${CMAKE_SOURCE_DIR}/.venv")
            set(Python3_FIND_VIRTUALENV FIRST)
        endif()
        find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

        FetchContent_Declare(
            pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11.git
            GIT_TAG v2.13.6
        )
        FetchContent_MakeAvailable(pybind11)

        pybind11_add_module(_mujoco_driver ${MUJOCO_DRIVER_PY_BINDINGS})

        target_include_directories(_mujoco_driver PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/cc/include
            ${GENERATED_DIR}
        )

        target_link_libraries(_mujoco_driver PRIVATE mujoco_driver)

        # Output Python module to build/driver/python directory
        set(PYTHON_OUTPUT_DIR ${CMAKE_BINARY_DIR}/driver/python/mujoco_driver)
        set_target_properties(_mujoco_driver PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY ${PYTHON_OUTPUT_DIR}
        )

        # Copy Python package files to build directory
        file(MAKE_DIRECTORY ${PYTHON_OUTPUT_DIR})
        configure_file(
            ${MUJOCO_DRIVER_PY_INIT}
            ${PYTHON_OUTPUT_DIR}/__init__.py
            COPYONLY
        )
    else()
        message(WARNING
            "MUJOCO_DRIVER_BUILD_PYTHON is ON, but Python bindings sources not found; "
            "skipping Python bindings."
        )
    endif()
endif()

# Tests
if(MUJOCO_DRIVER_BUILD_TESTS)
    enable_testing()

    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(driver_test
        cc/tests/driver_test.cc
        cc/tests/fragment_test.cc
    )

    target_include_directories(driver_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/cc/include
        ${CMAKE_CURRENT_SOURCE_DIR}/cc/src
        ${GENERATED_DIR}
    )

    target_link_libraries(driver_test
        PRIVATE
            mujoco_driver
            flatbuffers
            GTest::gtest_main
    )

    include(GoogleTest)
    gtest_discover_tests(driver_test)
endif()

# Examples
if(MUJOCO_DRIVER_BUILD_EXAMPLES)
    add_executable(simple_control cc/examples/simple_control.cc)
    target_link_libraries(simple_control PRIVATE mujoco_driver)
    target_include_directories(simple_control PRIVATE ${GENERATED_DIR})

    # Echo server for testing driver communication locally
    add_executable(echo_server cc/examples/echo_server.cc)
    target_include_directories(echo_server PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/cc/include
        ${CMAKE_CURRENT_SOURCE_DIR}/cc/src
        ${GENERATED_DIR}
    )
    target_link_libraries(echo_server PRIVATE mujoco_driver flatbuffers)
endif()

# Install
install(TARGETS mujoco_driver
    EXPORT mujoco_driver-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY cc/include/imujoco
    DESTINATION include
)

install(EXPORT mujoco_driver-targets
    FILE mujoco_driver-targets.cmake
    NAMESPACE mujoco_driver::
    DESTINATION lib/cmake/mujoco_driver
)
