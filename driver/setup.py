"""
setup.py for imujoco-driver

Builds the C++ extension using pybind11.
Downloads FlatBuffers headers if not present.
"""

import hashlib
import sys
import tarfile
import urllib.request
from pathlib import Path

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

# FlatBuffers version, URL, and SHA256 checksum
FLATBUFFERS_VERSION = "24.3.25"
FLATBUFFERS_URL = f"https://github.com/google/flatbuffers/archive/refs/tags/v{FLATBUFFERS_VERSION}.tar.gz"
FLATBUFFERS_SHA256 = "4157c5cacdb59737c5d627e47ac26b140e9ee28b1102f812b36068aab728c1ed"

HERE = Path(__file__).parent.resolve()
VENDOR_DIR = HERE / "vendor"
FLATBUFFERS_INCLUDE = VENDOR_DIR / "flatbuffers" / "include"


def verify_checksum(filepath: Path, expected_sha256: str) -> bool:
    """Verify SHA256 checksum of a file."""
    sha256 = hashlib.sha256()
    with open(filepath, "rb") as f:
        for chunk in iter(lambda: f.read(8192), b""):
            sha256.update(chunk)
    return sha256.hexdigest() == expected_sha256


def ensure_flatbuffers():
    """Download FlatBuffers headers if not present."""
    if (FLATBUFFERS_INCLUDE / "flatbuffers" / "flatbuffers.h").exists():
        return  # Already downloaded

    print(f"Downloading FlatBuffers v{FLATBUFFERS_VERSION}...")
    VENDOR_DIR.mkdir(parents=True, exist_ok=True)

    tarball_path = VENDOR_DIR / f"flatbuffers-{FLATBUFFERS_VERSION}.tar.gz"

    try:
        # Download
        urllib.request.urlretrieve(FLATBUFFERS_URL, tarball_path)
    except Exception as e:
        print(f"Error downloading FlatBuffers: {e}", file=sys.stderr)
        print("Please check your network connection and try again.", file=sys.stderr)
        raise SystemExit(1)

    # Verify checksum
    if not verify_checksum(tarball_path, FLATBUFFERS_SHA256):
        tarball_path.unlink()
        print("Error: FlatBuffers checksum verification failed!", file=sys.stderr)
        print("The downloaded file may be corrupted or tampered with.", file=sys.stderr)
        raise SystemExit(1)

    try:
        # Extract only the include directory
        with tarfile.open(tarball_path, "r:gz") as tar:
            extract_dir = VENDOR_DIR / "flatbuffers"
            extract_dir.mkdir(parents=True, exist_ok=True)

            prefix = f"flatbuffers-{FLATBUFFERS_VERSION}/include/"
            for member in tar.getmembers():
                if member.name.startswith(prefix):
                    member.name = member.name[len(f"flatbuffers-{FLATBUFFERS_VERSION}/"):]
                    tar.extract(member, extract_dir)
    except Exception as e:
        print(f"Error extracting FlatBuffers: {e}", file=sys.stderr)
        raise SystemExit(1)
    finally:
        if tarball_path.exists():
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
