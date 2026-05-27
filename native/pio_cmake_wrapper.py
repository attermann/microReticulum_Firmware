# PlatformIO pre-build script for the native-linux env. Delegates the
# actual build to CMake (top-level CMakeLists.txt) and replaces the
# default PlatformIO compile pipeline with a single "run cmake" command.
#
# This exists because Portduino + libgpiod + Linux toolchain integration
# is awkward to express in platformio.ini directly. The CMake build is the
# canonical native path; this script keeps `pio run -e native-linux`
# working as a familiar entrypoint.

import os
import subprocess
import sys

Import("env")  # noqa: F821  (provided by PlatformIO)

PROJECT_DIR = env["PROJECT_DIR"]  # noqa: F821
BUILD_DIR = os.path.join(PROJECT_DIR, "build")


def cmake_build(*args, **kwargs):
    """Invoke cmake configure + build in lieu of PlatformIO's own compile."""
    os.makedirs(BUILD_DIR, exist_ok=True)
    configure = ["cmake", "-S", PROJECT_DIR, "-B", BUILD_DIR]
    build = ["cmake", "--build", BUILD_DIR, "-j"]
    for cmd in (configure, build):
        print(">", " ".join(cmd))
        result = subprocess.run(cmd, cwd=PROJECT_DIR)
        if result.returncode != 0:
            print("[pio_cmake_wrapper] cmake step failed", file=sys.stderr)
            sys.exit(result.returncode)


# Replace PlatformIO's default build sequence with the CMake delegation.
env.AddCustomTarget(  # noqa: F821
    name="cmake_build",
    dependencies=None,
    actions=[cmake_build],
    title="cmake build",
    description="Build the native microreticulum binary via CMake.",
    always_build=True,
)

# Make the default `pio run -e native-linux` invocation trigger the wrapper.
env.Default("cmake_build")  # noqa: F821
