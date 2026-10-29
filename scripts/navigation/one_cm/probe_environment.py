"""记录独立验证目录的环境，不启动 ROS 节点或访问生产话题。"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.root.resolve(strict=True)
    report = {
        "hostname": platform.node(),
        "machine": platform.machine(),
        "kernel": platform.release(),
        "python": sys.version,
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "root": str(root),
        "entries": sorted(p.name for p in root.iterdir())[:30],
        "cpu_count": os.cpu_count(),
        "load_average": list(os.getloadavg()) if hasattr(os, "getloadavg") else None,
        "packages": {
            name: bool(importlib.util.find_spec(name))
            for name in ("numpy", "scipy", "osqp", "casadi", "rclpy", "matplotlib", "pytest")
        },
        "tools": {name: shutil.which(name) for name in ("git", "g++", "cmake", "colcon", "ninja")},
        "eigen_headers": Path("/usr/include/eigen3/Eigen/Core").is_file(),
        "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    memory = Path("/proc/meminfo")
    if memory.is_file():
        report["memory"] = {
            key: value.strip()
            for line in memory.read_text().splitlines()
            for key, value in [line.split(":", 1)]
            if key in {"MemTotal", "MemAvailable", "SwapTotal", "SwapFree"}
        }
    model = Path("/proc/device-tree/model")
    if model.is_file():
        report["device_model"] = model.read_text().rstrip("\0\n")
    disk = shutil.disk_usage(root)
    report["disk_free_bytes"] = disk.free
    if shutil.which("git"):
        git = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "--show-toplevel"],
            capture_output=True, text=True, check=False,
        )
        report["git_root"] = git.stdout.strip() if git.returncode == 0 else None
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        output = args.output.resolve()
        if not output.is_relative_to(root):
            raise ValueError("环境记录必须写入指定验证目录")
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("x", encoding="utf-8", newline="\n") as stream:
            stream.write(text)
    print(text, end="")


if __name__ == "__main__":
    main()
