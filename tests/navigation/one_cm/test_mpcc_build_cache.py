"""缓存决策单测；编译边界使用替身，不代表真实 acados 验收。"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace
import sys
import multiprocessing
import time
import pytest
from scripts.navigation.one_cm.nav1cm import local_mpcc as mpcc


LIBRARY = ('' if sys.platform == 'win32' else 'lib') + 'acados_ocp_solver_example' + ('.dll' if sys.platform == 'win32' else ('.dylib' if sys.platform == 'darwin' else '.so'))


class Compiler:
    def __init__(self):
        self.builds = 0
        self.fail_build = False
        self.fail_load = False

    def __call__(self, ocp, *, json_file, verbose, generate=True, build=True):
        assert generate == build
        library = Path(ocp.code_export_directory) / getattr(ocp, 'library_name', LIBRARY)
        if build:
            self.builds += 1
            time.sleep(.01)
            library.parent.mkdir(parents=True, exist_ok=True)
            library.write_bytes(b'compiled solver')
            Path(json_file).write_text('{}', encoding='utf-8')
            if self.fail_build:
                raise RuntimeError('compile failed')
        elif self.fail_load:
            raise RuntimeError('load failed')
        assert library.read_bytes() == b'compiled solver'
        return 'solver'


def cache_case(tmp_path):
    ocp = SimpleNamespace(model=SimpleNamespace(name='example'),
                          code_export_directory=str(tmp_path / 'forward'))
    compiler = Compiler()
    def load(identity='sdk-1'):
        return mpcc._cached_acados_solver(compiler, ocp, tmp_path / 'forward.json', identity, library_name=LIBRARY)
    return ocp, compiler, load


def test_complete_cache_skips_generation_and_build(tmp_path):
    _, compiler, load = cache_case(tmp_path)
    assert load() == load() == 'solver'
    assert compiler.builds == 1


@pytest.mark.parametrize('damage', ['library', 'json', 'manifest', 'identity'])
def test_missing_or_changed_artifact_rebuilds(tmp_path, damage):
    ocp, compiler, load = cache_case(tmp_path)
    load()
    if damage == 'library':
        (Path(ocp.code_export_directory) / LIBRARY).write_bytes(b'broken')
    elif damage == 'json':
        (tmp_path / 'forward.json').write_text('{broken', encoding='utf-8')
    elif damage == 'manifest':
        (tmp_path / 'forward.cache.json').unlink()
    load('sdk-2' if damage == 'identity' else 'sdk-1')
    assert compiler.builds == 2


def test_failed_build_is_never_marked_complete(tmp_path):
    _, compiler, load = cache_case(tmp_path)
    compiler.fail_build = True
    with pytest.raises(RuntimeError, match='compile failed'):
        load()
    compiler.fail_build = False
    load()
    assert compiler.builds == 2


def test_load_failure_is_explicit_without_rebuilding_or_old_fallback(tmp_path):
    _, compiler, load = cache_case(tmp_path)
    load()
    compiler.fail_load = True
    with pytest.raises(RuntimeError, match='load failed'):
        load()
    assert compiler.builds == 1


def test_concurrent_constructors_build_once(tmp_path):
    _, compiler, load = cache_case(tmp_path)
    with ThreadPoolExecutor(max_workers=4) as pool:
        assert list(pool.map(lambda _: load(), range(4))) == ['solver'] * 4
    assert compiler.builds == 1


class ProcessCompiler(Compiler):
    def __call__(self, ocp, **kwargs):
        if kwargs.get('build', True):
            with (Path(ocp.code_export_directory).parent / 'build-count').open('ab') as stream:
                stream.write(b'1')
        return super().__call__(ocp, **kwargs)


def process_load(directory):
    directory = Path(directory)
    ocp = SimpleNamespace(model=SimpleNamespace(name='example'),
                          code_export_directory=str(directory / 'forward'))
    mpcc._cached_acados_solver(ProcessCompiler(), ocp, directory / 'forward.json', 'sdk-1', library_name=LIBRARY)


def test_independent_processes_share_completed_build(tmp_path):
    ctx = multiprocessing.get_context('spawn')
    processes = [ctx.Process(target=process_load, args=(str(tmp_path),)) for _ in range(3)]
    for process in processes:
        process.start()
    for process in processes:
        process.join(20)
        assert process.exitcode == 0
    assert (tmp_path / 'build-count').read_bytes() == b'1'


def test_sdk_and_abi_inputs_change_cache_identity(tmp_path):
    templates, libraries = tmp_path / 'templates', tmp_path / 'lib'
    templates.mkdir()
    libraries.mkdir()
    source = templates / 'acados_ocp_solver.py'
    native = libraries / 'libacados.so'
    source.write_bytes(b'sdk one')
    native.write_bytes(b'abi one')
    first = mpcc._acados_cache_identity(templates, libraries, 'casadi-one')
    source.write_bytes(b'sdk two')
    second = mpcc._acados_cache_identity(templates, libraries, 'casadi-one')
    native.write_bytes(b'abi two')
    third = mpcc._acados_cache_identity(templates, libraries, 'casadi-one')
    fourth = mpcc._acados_cache_identity(templates, libraries, 'casadi-two')
    assert len({first, second, third, fourth}) == 4


@pytest.mark.parametrize('library_name', ['acados_ocp_solver_example.dll', 'libacados_ocp_solver_example.so'])
def test_cache_uses_sdk_library_name(tmp_path, library_name):
    ocp = SimpleNamespace(model=SimpleNamespace(name='example'), library_name=library_name,
                          code_export_directory=str(tmp_path / 'forward'))
    compiler = Compiler()
    for _ in range(2):
        mpcc._cached_acados_solver(compiler, ocp, tmp_path / 'forward.json', 'sdk-1',
                                  library_name=library_name)
    assert compiler.builds == 1


def test_generator_source_change_invalidates_cache(tmp_path):
    templates, libraries = tmp_path / 'templates', tmp_path / 'lib'
    templates.mkdir()
    libraries.mkdir()
    generator = tmp_path / 'local_mpcc.py'
    generator.write_bytes(b'model-one')
    first = mpcc._acados_cache_identity(templates, libraries, 'casadi', generator_source=generator)
    generator.write_bytes(b'model-two')
    second = mpcc._acados_cache_identity(templates, libraries, 'casadi', generator_source=generator)
    assert first != second


def test_windows_runtime_dll_change_in_bin_invalidates_cache(tmp_path):
    templates, libraries, binaries = tmp_path / 'templates', tmp_path / 'lib', tmp_path / 'bin'
    for directory in (templates, libraries, binaries):
        directory.mkdir()
    runtime = binaries / 'acados.dll'
    runtime.write_bytes(b'abi-one')
    first = mpcc._acados_cache_identity(templates, libraries, 'casadi', runtime_dir='bin')
    runtime.write_bytes(b'abi-two')
    second = mpcc._acados_cache_identity(templates, libraries, 'casadi', runtime_dir='bin')
    assert first != second
