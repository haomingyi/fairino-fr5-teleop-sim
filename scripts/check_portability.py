#!/usr/bin/env python3
"""Fail when first-party project files depend on another local checkout."""
from __future__ import annotations

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
REQUIRED = (
    "IPE-quest-hand-teleop/Makefile",
    "IPE-quest-hand-teleop/hand_tracking_streamer/ProjectSettings/ProjectSettings.asset",
    "IPE-quest-hand-teleop/ih01_bridge/src/hts_ih01/live.py",
    "IPE-quest-hand-teleop/runtime/ih01_runtime/simulation.py",
    "IPE-quest-hand-teleop/simulation/ih01_x1_dual.xml",
    "IPE-quest-hand-teleop/third_party/SOEM/CMakeLists.txt",
    "third_party/fairino-python-sdk/linux/fairino/Robot.py",
)
FORBIDDEN = (
    "/home/",
    "/Users/",
    "\\Users\\",
    "ih01-dexhands-teleop",
)
SKIP_PREFIXES = (
    ".git/",
    ".venv/",
    "IPE-quest-hand-teleop/.venv/",
    "IPE-quest-hand-teleop/build/",
    "IPE-quest-hand-teleop/hand_tracking_streamer/Library/",
    "IPE-quest-hand-teleop/hand_tracking_streamer/Temp/",
    "IPE-quest-hand-teleop/third_party/",
    "third_party/",
)


def project_files() -> list[Path]:
    try:
        result = subprocess.run(
            ["git", "ls-files", "-co", "--exclude-standard", "-z"], cwd=ROOT, check=True,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
        names = result.stdout.decode("utf-8").split("\0")
        return [ROOT / name for name in names if name]
    except (OSError, subprocess.CalledProcessError, UnicodeDecodeError):
        return [path for path in ROOT.rglob("*") if path.is_file()]


def main() -> int:
    problems: list[str] = []
    for relative in REQUIRED:
        if not (ROOT / relative).is_file():
            problems.append(f"missing bundled dependency: {relative}")

    for path in project_files():
        relative = path.relative_to(ROOT).as_posix()
        if relative == "scripts/check_portability.py":
            continue
        if relative.startswith(SKIP_PREFIXES) or not path.exists():
            continue
        if path.is_symlink():
            target = path.resolve()
            if ROOT != target and ROOT not in target.parents:
                problems.append(f"external symlink: {relative} -> {target}")
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        for marker in FORBIDDEN:
            if marker in text:
                problems.append(f"external local-path marker {marker!r}: {relative}")

    if problems:
        for problem in problems:
            print(f"FAIL portability: {problem}")
        return 1
    print("PASS portability: runtime and bundled dependencies stay inside project root")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
