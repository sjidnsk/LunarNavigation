"""原生距离场显式加载、注入与精确数学对照。"""
import os
from pathlib import Path
import shutil
import subprocess
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import GridSnapshot
from scripts.navigation.one_cm.nav1cm.maps import build_distance_field, DistanceFieldCache
from scripts.navigation.one_cm.nav1cm.field_native import NativeDistanceField


def test_missing_library_fails(tmp_path):
    with pytest.raises(OSError):
        NativeDistanceField(tmp_path / 'missing-library')


def test_explicit_backend_and_cache():
    class Backend:
        calls = 0
        def build(self, grid):
            self.calls += 1
            return build_distance_field(grid)
    backend = Backend()
    a = GridSnapshot(1, (0, 0), .01, np.zeros((7, 9), dtype=np.int8))
    b = GridSnapshot(2, (.003, -.007), .01, a.cells)
    cache = DistanceFieldCache(backend=backend)
    first, second = cache.get(a), cache.get(b)
    assert backend.calls == 1 and cache.last_reused
    assert second.epoch == 2 and second.origin_xy == b.origin_xy
    assert second.distance_m is first.distance_m
    assert build_distance_field(a, backend=backend).epoch == 1
    with pytest.raises(ValueError):
        second.distance_m.setflags(write=True)


@pytest.fixture(scope='module')
def native(tmp_path_factory):
    explicit = os.environ.get('NAV1CM_DISTANCE_FIELD_LIB')
    if explicit:
        return NativeDistanceField(explicit)
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('g++ unavailable; real native comparison NOT_RUN')
    output = tmp_path_factory.mktemp('native-field') / ('field.dll' if os.name == 'nt' else 'field.so')
    source = Path(__file__).resolve().parents[3] / 'scripts/navigation/one_cm/nav1cm/distance_field_native.cpp'
    subprocess.run([compiler, '-O3', '-std=c++17', '-shared', '-fPIC', str(source), '-o', str(output)], check=True)
    return NativeDistanceField(output)


@pytest.mark.parametrize('shape', [(2, 2), (13, 37), (500, 500)])
@pytest.mark.parametrize('kind', ['free', 'blocked', 'unknown', 'sparse', 'dense', 'wall'])
@pytest.mark.parametrize('allow_unknown', [False, True])
def test_native_matches_scipy(native, shape, kind, allow_unknown):
    rng = np.random.default_rng(745)
    cells = np.zeros(shape, dtype=np.int8)
    if kind == 'blocked': cells[:] = 1
    elif kind == 'unknown': cells[:] = -1
    elif kind in ('sparse', 'dense'):
        cells[:] = rng.choice([-1, 0, 1], size=shape, p=[.1, .89, .01] if kind == 'sparse' else [.2, .1, .7])
    elif kind == 'wall': cells[:, shape[1]//2] = 1
    grid = GridSnapshot(7, (.003, -1.017), .01, cells, allow_unknown)
    actual, expected = native.build(grid), build_distance_field(grid)
    assert actual.epoch == grid.epoch and actual.origin_xy == grid.origin_xy
    for name in ('distance_m', 'gradient_x', 'gradient_y'):
        value = getattr(actual, name)
        np.testing.assert_allclose(value, getattr(expected, name), atol=1e-10, rtol=0)
        assert np.isfinite(value).all() and not value.flags.writeable
        with pytest.raises(ValueError): value.setflags(write=True)


def test_native_rejects_single_axis(native):
    grid = GridSnapshot(0, (0, 0), .01, np.zeros((1, 5), dtype=np.int8))
    with pytest.raises(ValueError): native.build(grid)


@pytest.mark.parametrize('failure', ['abi', 'status', 'nonfinite'])
def test_library_contract_rejected(monkeypatch, tmp_path, failure):
    from scripts.navigation.one_cm.nav1cm import field_native
    class Function:
        def __init__(self, call): self.call = call
        def __call__(self, *args): return self.call(*args)
    class Library:
        nav1cm_field_abi_version = Function(lambda: 9 if failure == 'abi' else 1)
        @staticmethod
        def build(passable, h, w, resolution, lower, gx, gy):
            for a in (lower, gx, gy): a.fill(np.nan if failure == 'nonfinite' else 0.)
            return 2 if failure == 'status' else 0
        nav1cm_distance_field_v1 = Function(build)
    path = tmp_path / 'fake-library'
    path.write_bytes(b'fake ABI test only')
    monkeypatch.setattr(field_native.ctypes, 'CDLL', lambda _: Library())
    with pytest.raises((RuntimeError, ValueError)):
        native = NativeDistanceField(path)
        native.build(GridSnapshot(0, (0, 0), .01, np.zeros((3, 5), dtype=np.int8)))
