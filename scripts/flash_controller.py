#!/usr/bin/env python3
"""Deploy controller bundle to a Raspberry Pi."""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Flash controller runtime to Raspberry Pi")
    parser.add_argument("bundle", type=Path, help="Path to torque model bundle")
    parser.add_argument("host", help="Pi hostname or IP")
    parser.add_argument("--user", default="pi", help="SSH username")
    parser.add_argument("--target", default="~/controller", help="Remote path")
    args = parser.parse_args()

    subprocess.run([
        "scp",
        "-r",
        str(args.bundle),
        f"{args.user}@{args.host}:{args.target}",
    ], check=True)


if __name__ == "__main__":
    main()
