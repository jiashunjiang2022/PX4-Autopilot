#!/usr/bin/env bash
set -euo pipefail

# build-no-conda.sh
# Wrapper to run PX4 builds with Conda entries removed from PATH to avoid
# mixing conda-provided toolchain/libs (e.g. protoc/libprotobuf) with system
# packages. Usage:
#   ./Tools/scripts/build-no-conda.sh [<make-target> [<make-target> ...]]
# or via Makefile:
#   make no-conda px4_sitl_default

# determine repo root (two levels up from this script)
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

# Sanitize PATH by removing entries that likely come from Anaconda/Conda
ORIG_PATH="$PATH"
NEW_PATH=$(printf "%s" "$ORIG_PATH" | tr ':' '\n' | grep -v -E '(anaconda|miniconda|conda)' | paste -sd: -)
export PATH="$NEW_PATH"

# Unset common Conda runtime env vars that may leak in
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_EXE CONDA_PYTHON_EXE CONDA_SHLVL || true

echo "[build-no-conda] Using PATH: $PATH"

cd "$REPO_ROOT"

if [ "$#" -eq 0 ]; then
  TARGETS=(px4_sitl_default)
else
  TARGETS=("$@")
fi

# If the build directory for a target exists, prefer calling cmake --build
# directly to avoid recursive make issues and MAKEFLAGS leaking goals to ninja.
for t in "${TARGETS[@]}"; do
  BUILD_DIR="$REPO_ROOT/build/$t"
  if [ -d "$BUILD_DIR" ]; then
    echo "[build-no-conda] Building via cmake --build $BUILD_DIR"
    cmake --build "$BUILD_DIR" -- -j$(nproc)
  else
    echo "[build-no-conda] Build dir $BUILD_DIR not found, falling back to make $t"
    env PATH="$PATH" make "$t"
  fi
done

exit 0
