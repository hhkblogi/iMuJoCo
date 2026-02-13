load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "qhullstatic_r",
    srcs = glob(["src/libqhull_r/*.c"]),
    hdrs = glob(["src/libqhull_r/*.h"]),
    includes = [
        "src",
        "src/libqhull_r",
    ],
    visibility = ["//visibility:public"],
)
