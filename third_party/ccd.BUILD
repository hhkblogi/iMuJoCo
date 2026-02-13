load("@rules_cc//cc:defs.bzl", "cc_library")

# Generate ccd/config.h with double precision enabled
genrule(
    name = "ccd_config_h",
    outs = ["src/ccd/config.h"],
    cmd = "\n".join([
        "cat > $@ << 'EOF'",
        "#ifndef CCD_CONFIG_H",
        "#define CCD_CONFIG_H",
        "#define CCD_DOUBLE",
        "#endif",
        "EOF",
    ]),
)

cc_library(
    name = "ccd",
    srcs = [
        "src/alloc.h",
        "src/ccd.c",
        "src/dbg.h",
        "src/list.h",
        "src/mpr.c",
        "src/polytope.c",
        "src/polytope.h",
        "src/simplex.h",
        "src/support.c",
        "src/support.h",
        "src/vec3.c",
    ],
    hdrs = [
        "src/ccd/ccd.h",
        "src/ccd/ccd_export.h",
        "src/ccd/compiler.h",
        "src/ccd/config.h",
        "src/ccd/quat.h",
        "src/ccd/vec3.h",
    ],
    copts = [
        "-Wno-unused-parameter",
    ],
    includes = ["src"],
    local_defines = [
        "CCD_STATIC_DEFINE",
    ],
    visibility = ["//visibility:public"],
)
