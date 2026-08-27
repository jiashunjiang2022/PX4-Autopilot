#!/usr/bin/env python3
"""Phase 4.2 entry point for the shared RA-L preflight checker."""

import pathlib
import runpy


CHECKER = (pathlib.Path(__file__).resolve().parents[2]
           / "ral_revision_phase4_1/parameter_template/preflight_check.py")


if __name__ == "__main__":
    runpy.run_path(str(CHECKER), run_name="__main__")
