#!/usr/bin/env bash
# 仅在用户授权的任务目录准备依赖，不安装系统包。
set -euo pipefail
root=/home/yanfa/P4_veri
sha=43354449192aa51557abda37d1bc17d907077354
export TMPDIR="$root/deps/tmp" PIP_CACHE_DIR="$root/deps/pip-cache"
mkdir -p "$TMPDIR" "$PIP_CACHE_DIR" "$root/build/humble" "$root/deps/solver-python"
if [[ ! -d "$root/deps/acados-v040" ]]; then
  git clone --branch v0.4.0 --depth 1 https://github.com/acados/acados.git "$root/deps/acados-v040"
fi
[[ "$(git -C "$root/deps/acados-v040" rev-parse HEAD)" == "$sha" ]]
if [[ ! -f "$root/deps/acados-v040/lib/libacados.so" ]]; then
  git -C "$root/deps/acados-v040" submodule update --init --depth 1 external/blasfeo external/hpipm
  cmake -S "$root/deps/acados-v040" -B "$root/build/humble/acados-v040" -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57 -DACADOS_WITH_OPENMP=OFF -DCMAKE_INSTALL_PREFIX="$root/deps/acados-v040"
  cmake --build "$root/build/humble/acados-v040" -j 2
  cmake --install "$root/build/humble/acados-v040"
fi
if [[ ! -x "$root/deps/solver-venv/bin/python" ]]; then
  python3 -m venv --without-pip "$root/deps/solver-venv"
fi
export PYTHONPATH=/usr/lib/python3/dist-packages PYTHONNOUSERSITE=1
python="$root/deps/solver-venv/bin/python"
site_dir="$($python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
printf "import site; site.addsitedir('%s/deps/solver-python'); site.addsitedir('%s/deps/acados-v040/interfaces/acados_template')\n" "$root" "$root" > "$site_dir/nav1cm.pth"
if ! "$python" -c 'import casadi, future_fstrings, Cython; assert casadi.__version__ == "3.6.6"'; then
  "$python" -m pip install --target "$root/deps/solver-python" --no-deps --only-binary=:all: 'casadi==3.6.6' 'future-fstrings==1.2.0' 'Cython==3.0.11'
fi
renderer="$root/deps/acados-v040/bin/t_renderer"
if [[ ! -f "$renderer" ]]; then
  mkdir -p "$(dirname "$renderer")"
  curl --fail --location https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-arm64 --output "$renderer"
fi
echo "a29f563c7f49368b1802864cdf2bad9d50fe0fb7180c9d210c188a30bb23f167  $renderer" | sha256sum --check
chmod +x "$renderer"
export ACADOS_SOURCE_DIR="$root/deps/acados-v040" LD_LIBRARY_PATH="$root/deps/acados-v040/lib"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
"$python" -c 'import casadi, acados_template; print("task-scoped solver ready",casadi.__version__)'
