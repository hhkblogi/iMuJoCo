load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

# ============================================================================
# C++ runtime (header-only)
# ============================================================================

cc_library(
    name = "runtime_cc",
    hdrs = glob(["include/flatbuffers/*.h"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)

# ============================================================================
# Core flatbuffers library (parser, codegen utilities)
# ============================================================================

cc_library(
    name = "flatbuffers_lib",
    srcs = [
        "src/code_generators.cpp",
        "src/idl_parser.cpp",
        "src/reflection.cpp",
        "src/util.cpp",
    ],
    hdrs = glob(["include/flatbuffers/*.h"]) + glob(["include/codegen/*.h"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)

# ============================================================================
# flatc compiler library
# ============================================================================

cc_library(
    name = "flatc_library",
    srcs = [
        "src/annotated_binary_text_gen.cpp",
        "src/bfbs_gen_lua.cpp",
        "src/bfbs_gen_nim.cpp",
        "src/binary_annotator.cpp",
        "src/flatc.cpp",
    ],
    hdrs = glob([
        "include/flatbuffers/*.h",
        "include/codegen/*.h",
        "src/*.h",
    ]),
    includes = ["include"],
    deps = [":flatbuffers_lib"],
)

# ============================================================================
# gRPC stub generators (flatbuffers' own, NOT the gRPC library)
# ============================================================================

cc_library(
    name = "grpc_generators",
    srcs = [
        "grpc/src/compiler/cpp_generator.cc",
        "grpc/src/compiler/go_generator.cc",
        "grpc/src/compiler/java_generator.cc",
        "grpc/src/compiler/python_generator.cc",
        "grpc/src/compiler/swift_generator.cc",
        "grpc/src/compiler/ts_generator.cc",
    ],
    hdrs = [
        "grpc/src/compiler/cpp_generator.h",
        "grpc/src/compiler/go_generator.h",
        "grpc/src/compiler/java_generator.h",
        "grpc/src/compiler/python_generator.h",
        "grpc/src/compiler/schema_interface.h",
        "grpc/src/compiler/swift_generator.h",
        "grpc/src/compiler/ts_generator.h",
    ],
    includes = ["grpc"],
    deps = [":flatbuffers_lib"],
)

# ============================================================================
# flatc compiler (all language generators)
# ============================================================================

cc_binary(
    name = "flatc",
    srcs = [
        "include/codegen/python.cc",
        "src/file_manager.cpp",
        "src/file_name_manager.cpp",
        "src/flatc_main.cpp",
        "src/idl_gen_binary.cpp",
        "src/idl_gen_cpp.cpp",
        "src/idl_gen_csharp.cpp",
        "src/idl_gen_dart.cpp",
        "src/idl_gen_fbs.cpp",
        "src/idl_gen_go.cpp",
        "src/idl_gen_grpc.cpp",
        "src/idl_gen_java.cpp",
        "src/idl_gen_json_schema.cpp",
        "src/idl_gen_kotlin.cpp",
        "src/idl_gen_kotlin_kmp.cpp",
        "src/idl_gen_lobster.cpp",
        "src/idl_gen_php.cpp",
        "src/idl_gen_python.cpp",
        "src/idl_gen_rust.cpp",
        "src/idl_gen_swift.cpp",
        "src/idl_gen_text.cpp",
        "src/idl_gen_ts.cpp",
    ],
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        ":flatc_library",
        ":grpc_generators",
    ],
)
