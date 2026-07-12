# Copyright (C) 2026, Chad Attermann
#
# Pre-build script for [env:native-macos].
#
# Problem: PlatformIO's .ino → .cpp converter shells out with
#   $CXX -o <out> -x c++ -fpreprocessed -dD -E <tmp>
# Apple's clang rejects `-fpreprocessed`. We need to route that step
# through Homebrew's `g++-NN` (which supports the flag) without modifying
# anything in $CXX (which is used by the rest of the build).
#
# Earlier attempts to monkey-patch pioino.InoToCPPConverter._gcc_preprocess
# at the class level appeared to take effect (a fresh-instance method
# lookup resolved to the patched function) but PIO's actual conversion
# call still ran the original — suggests env.AddMethod captured a
# different binding or SCons replaces the registered method. So we
# override the env-level ConvertInoToCpp method directly: that's what
# the build script actually calls.

import os
import shutil
import subprocess
import sys
import tempfile

from platformio.builder.tools import pioino

Import("env")  # noqa: F821


def _find_homebrew_gxx():
    for cmd in ("g++-15", "g++-14", "g++-13", "g++-12"):
        if shutil.which(cmd):
            return cmd
    return None


_GXX = _find_homebrew_gxx()
if _GXX is None:
    print(
        "[macos-pre-build] FATAL: no Homebrew g++-NN found on PATH. "
        "Install with: brew install gcc",
        file=sys.stderr,
    )
    sys.exit(1)


class HomebrewGccConverter(pioino.InoToCPPConverter):
    """Subclass that overrides the preprocess step to use Homebrew g++."""

    def _gcc_preprocess(self, contents, out_file):
        tmp_path = tempfile.mkstemp(suffix=".cpp")[1]
        self.write_safe_contents(tmp_path, contents)
        try:
            result = subprocess.run(
                [_GXX, "-o", out_file, "-x", "c++",
                 "-fpreprocessed", "-dD", "-E", "-P", tmp_path],
                check=False,
            )
            return result.returncode == 0
        finally:
            try:
                os.remove(tmp_path)
            except OSError:
                pass


def ConvertInoToCpp_macos(env):
    """Drop-in replacement for the env method PIO registers from pioino.
    Uses our HomebrewGccConverter subclass instead of the stock one."""
    ino_nodes = env.FindInoNodes()
    if not ino_nodes:
        return
    c = HomebrewGccConverter(env)
    out_file = c.convert(ino_nodes)
    if out_file:
        import atexit
        atexit.register(pioino._delete_file, out_file)


# Re-register so the build script's env.ConvertInoToCpp() call uses ours.
# env.AddMethod replaces the prior binding cleanly.
env.AddMethod(ConvertInoToCpp_macos, "ConvertInoToCpp")  # noqa: F821

print(f"[macos-pre-build] ConvertInoToCpp -> HomebrewGccConverter (using {_GXX})")
