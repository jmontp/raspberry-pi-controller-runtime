"""Pytest configuration ensuring the project src directory is importable."""

import os
import sys
from pathlib import Path

os.environ.setdefault("JOBLIB_MULTIPROCESSING", "0")

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
