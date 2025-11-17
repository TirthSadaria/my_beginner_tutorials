"""
Minimal stub implementation of ament_cmake_test.main().

This is provided so that environments without the full
`ros-humble-ament-cmake-test` package installed can still execute
Catch2-based tests via the default `run_test.py` helper script.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import subprocess
import sys
from typing import List


def _write_result_file(path: pathlib.Path, command: List[str], rc: int) -> None:
    """Write a very small JUnit-style XML result file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    contents = f"""<?xml version="1.0"?>
<testsuite tests="1" failures="{int(rc != 0)}">
  <testcase classname="ament_cmake_test" name="{' '.join(command)}">
    {"<failure message='failure'/>" if rc != 0 else ""}
  </testcase>
</testsuite>
"""
    path.write_text(contents, encoding="utf-8")


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_file")
    parser.add_argument("--package-name", dest="package", default="")
    parser.add_argument("--generate-result-on-success",
                        action="store_true", dest="generate")
    parser.add_argument("--command", nargs=argparse.REMAINDER, required=True)
    args = parser.parse_args(argv)

    command = args.command
    if command and command[0] == "--":
        command = command[1:]

    env = os.environ.copy()
    process = subprocess.run(command, env=env)

    result_path = pathlib.Path(args.result_file)
    if process.returncode == 0 and args.generate:
        _write_result_file(result_path, command, process.returncode)
    elif process.returncode != 0:
        _write_result_file(result_path, command, process.returncode)

    return process.returncode


if __name__ == "__main__":
    sys.exit(main())

