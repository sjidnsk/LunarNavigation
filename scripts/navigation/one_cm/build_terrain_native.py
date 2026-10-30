"""独立编译真实 core 地形桥；输出目录由调用者明确指定，默认不写缓存。

python scripts/navigation/one_cm/build_terrain_native.py --output /absolute/libnav1cm_terrain.so
可用 --print-command 查看等价 g++ 命令。ROS 包也可仅编译 terrain_bridge.cpp，
再链接已安装的 lunar_incremental_navigation_core；这不依赖 ROS 或 DRL。
"""

import argparse
import os
from pathlib import Path
import shlex
import shutil
import subprocess


def build_command(output, compiler="g++"):
    root = Path(__file__).resolve().parents[3]
    core = root / "ros2_ws/src/lunar_incremental_navigation_core"
    return [compiler, "-O2", "-std=c++20", "-shared", "-fPIC", "-pthread",
            "-I", str(core / "include"), "-I", str(core / "src"),
            str(Path(__file__).with_name("nav1cm") / "terrain_bridge.cpp"),
            *[str(core / "src/map" / name) for name in
              ("persistent_elevation_map.cpp", "platform_elevation_physics.cpp", "fine_traversability_builder.cpp")],
            *([] if os.name == "nt" else ["-Wl,-z,defs"]), "-o", str(Path(output).resolve())]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--compiler", default="g++")
    parser.add_argument("--print-command", action="store_true")
    args = parser.parse_args()
    command = build_command(args.output, args.compiler)
    print(shlex.join(command))
    if args.print_command:
        return
    if shutil.which(args.compiler) is None:
        parser.error("C++20 compiler unavailable; native terrain build NOT_RUN")
    args.output.resolve().parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
