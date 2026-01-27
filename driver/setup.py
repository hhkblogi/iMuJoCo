"""
setup.py for imujoco-driver

Builds the C++ extension using pybind11.
Downloads FlatBuffers headers if not present.
"""

import os
import tarfile
import urllib.request
from pathlib import Path

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

# FlatBuffers version and URL
FLATBUFFERS_VERSION = "24.3.25"
FLATBUFFERS_URL = f"https://github.com/google/flatbuffers/archive/refs/tags/v{FLATBUFFERS_VERSION}.tar.gz"

HERE = Path(__file__).parent.resolve()
VENDOR_DIR = HERE / "vendor"
FLATBUFFERS_INCLUDE = VENDOR_DIR / "flatbuffers" / "include"


def ensure_flatbuffers():
    """Download FlatBuffers headers if not present."""
    if (FLATBUFFERS_INCLUDE / "flatbuffers" / "flatbuffers.h").exists():
        return  # Already downloaded

    print(f"Downloading FlatBuffers v{FLATBUFFERS_VERSION}...")
    VENDOR_DIR.mkdir(parents=True, exist_ok=True)

    tarball_path = VENDOR_DIR / f"flatbuffers-{FLATBUFFERS_VERSION}.tar.gz"

    # Download
    urllib.request.urlretrieve(FLATBUFFERS_URL, tarball_path)

    # Extract only the include directory
    with tarfile.open(tarball_path, "r:gz") as tar:
        extract_dir = VENDOR_DIR / "flatbuffers"
        extract_dir.mkdir(parents=True, exist_ok=True)

        prefix = f"flatbuffers-{FLATBUFFERS_VERSION}/include/"
        for member in tar.getmembers():
            if member.name.startswith(prefix):
                member.name = member.name[len(f"flatbuffers-{FLATBUFFERS_VERSION}/"):]
                tar.extract(member, extract_dir)

    tarball_path.unlink()
    print(f"FlatBuffers headers installed to {FLATBUFFERS_INCLUDE}")


ensure_flatbuffers()

cpp_sources = [
    "python/bindings.cc",
    "cc/src/driver.cc",
    "cc/src/fragment.cc",
]

include_dirs = [
    "cc/include",
    "cc/src",
    "generated",
    str(FLATBUFFERS_INCLUDE),
]

ext_modules = [
    Pybind11Extension(
        "imujoco_driver._imujoco_driver",
        sources=cpp_sources,
        include_dirs=include_dirs,
        cxx_std=20,
        define_macros=[("VERSION_INFO", '"0.1.0"')],
    ),
]

setup(
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)
