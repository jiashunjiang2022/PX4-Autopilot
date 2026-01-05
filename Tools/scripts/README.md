build-no-conda.sh — Wrapper to build without Conda paths

Usage:

- From repo root using Makefile target:

  make no-conda px4_sitl_default

- Or invoke the script directly:

  ./Tools/scripts/build-no-conda.sh px4_sitl_default

What it does:
- Removes entries that look like Anaconda/Conda from PATH.
- Unsets common CONDA_* environment variables.
- Invokes the normal Makefile target(s) with the sanitized PATH.

Why:
- Prevents mixing Conda-provided libraries (e.g., libprotobuf/protoc) with
  system packages, which can cause ABI or header mismatches during builds.
