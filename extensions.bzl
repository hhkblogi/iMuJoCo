"""Module extension for MuJoCo and its third-party C dependencies."""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _mujoco_deps_impl(_ctx):
    """Fetch MuJoCo and its C dependencies."""

    # MuJoCo physics engine
    http_archive(
        name = "mujoco",
        build_file = "//:third_party/mujoco.BUILD",
        strip_prefix = "mujoco-3.4.0",
        urls = ["https://github.com/google-deepmind/mujoco/archive/refs/tags/3.4.0.tar.gz"],
    )

    # FlatBuffers (C++ runtime + flatc compiler only — avoids rules_swift dependency)
    http_archive(
        name = "flatbuffers",
        build_file = "//:third_party/flatbuffers.BUILD",
        patch_cmds = [
            "find . -mindepth 2 -name BUILD -delete",
            "find . -mindepth 2 -name BUILD.bazel -delete",
        ],
        strip_prefix = "flatbuffers-25.12.19",
        urls = ["https://github.com/google/flatbuffers/archive/refs/tags/v25.12.19.tar.gz"],
    )

    # lodepng - PNG image loading
    http_archive(
        name = "lodepng",
        build_file = "//:third_party/lodepng.BUILD",
        strip_prefix = "lodepng-17d08dd26cac4d63f43af217ebd70318bfb8189c",
        urls = ["https://github.com/lvandeve/lodepng/archive/17d08dd26cac4d63f43af217ebd70318bfb8189c.tar.gz"],
    )

    # tinyxml2 - XML parsing
    http_archive(
        name = "tinyxml2",
        build_file = "//:third_party/tinyxml2.BUILD",
        strip_prefix = "tinyxml2-e6caeae85799003f4ca74ff26ee16a789bc2af48",
        urls = ["https://github.com/leethomason/tinyxml2/archive/e6caeae85799003f4ca74ff26ee16a789bc2af48.tar.gz"],
    )

    # tinyobjloader - OBJ model loading
    http_archive(
        name = "tinyobjloader",
        build_file = "//:third_party/tinyobjloader.BUILD",
        strip_prefix = "tinyobjloader-1421a10d6ed9742f5b2c1766d22faa6cfbc56248",
        urls = ["https://github.com/tinyobjloader/tinyobjloader/archive/1421a10d6ed9742f5b2c1766d22faa6cfbc56248.tar.gz"],
    )

    # qhull - Convex hull computation
    http_archive(
        name = "qhull",
        build_file = "//:third_party/qhull.BUILD",
        strip_prefix = "qhull-62ccc56af071eaa478bef6ed41fd7a55d3bb2d80",
        urls = ["https://github.com/qhull/qhull/archive/62ccc56af071eaa478bef6ed41fd7a55d3bb2d80.tar.gz"],
    )

    # libccd - Collision detection
    http_archive(
        name = "ccd",
        build_file = "//:third_party/ccd.BUILD",
        strip_prefix = "libccd-7931e764a19ef6b21b443376c699bbc9c6d4fba8",
        urls = ["https://github.com/danfis/libccd/archive/7931e764a19ef6b21b443376c699bbc9c6d4fba8.tar.gz"],
    )

    # MarchingCubeCpp - Header-only marching cubes
    http_archive(
        name = "marchingcubecpp",
        build_file = "//:third_party/MarchingCubeCpp.BUILD",
        strip_prefix = "MarchingCubeCpp-f03a1b3ec29b1d7d865691ca8aea4f1eb2c2873d",
        urls = ["https://github.com/aparis69/MarchingCubeCpp/archive/f03a1b3ec29b1d7d865691ca8aea4f1eb2c2873d.tar.gz"],
    )

    # MuJoCo Menagerie robot models (BSD-3 licensed)
    http_archive(
        name = "mujoco_menagerie",
        build_file = "//:third_party/mujoco_menagerie.BUILD",
        strip_prefix = "mujoco_menagerie-a03e87bf13502b0b48ebbf2808928fd96ebf9cf3",
        urls = ["https://github.com/google-deepmind/mujoco_menagerie/archive/a03e87bf13502b0b48ebbf2808928fd96ebf9cf3.tar.gz"],
    )

    # TriangleMeshDistance - Header-only triangle mesh distance
    http_archive(
        name = "trianglemeshdistance",
        build_file = "//:third_party/TriangleMeshDistance.BUILD",
        strip_prefix = "TriangleMeshDistance-2cb643de1436e1ba8e2be49b07ec5491ac604457",
        urls = ["https://github.com/InteractiveComputerGraphics/TriangleMeshDistance/archive/2cb643de1436e1ba8e2be49b07ec5491ac604457.tar.gz"],
    )

mujoco_deps = module_extension(
    implementation = _mujoco_deps_impl,
)
