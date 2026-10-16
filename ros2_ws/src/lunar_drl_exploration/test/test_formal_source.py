"""Read-only verification of the exact deployed training package source."""
import hashlib
import importlib.util
import json
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[4]
SCRIPT=ROOT/'scripts/drl/verify_formal_source.py'

def verifier():
    assert SCRIPT.is_file(), 'formal source verifier is missing'
    spec=importlib.util.spec_from_file_location('verify_formal_source',SCRIPT)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.verify_source


def fixture(tmp_path):
    package=tmp_path/'package';package.mkdir()
    (package/'a.py').write_text('value = 1\n')
    manifest=tmp_path/'manifest.json'
    manifest.write_text(json.dumps(dict(schema='formal_drl_source_v1',files={'a.py':hashlib.sha256((package/'a.py').read_bytes()).hexdigest()})))
    return package,manifest


def test_recorded_formal_package_is_complete():
    report=verifier()(ROOT/'ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration',ROOT/'docs/drl/formal-runtime-source-manifest.json')
    assert report['ok'],report
    assert report['file_count']==78


@pytest.mark.parametrize('change',['changed','missing','extra'])
def test_drift_is_reported_without_writing_source(tmp_path,change):
    package,manifest=fixture(tmp_path)
    if change=='changed':(package/'a.py').write_text('value = 2\n')
    if change=='missing':(package/'a.py').unlink()
    if change=='extra':(package/'extra.py').write_text('x=0\n')
    before={p.name:p.read_bytes() for p in package.iterdir()}
    result=verifier()(package,manifest)
    assert not result['ok']
    assert result[change]
    assert {p.name:p.read_bytes() for p in package.iterdir()}==before


def test_valid_copy(tmp_path):
    package,manifest=fixture(tmp_path)
    assert verifier()(package,manifest)['ok']


def test_manifest_cannot_escape_package(tmp_path):
    package,manifest=fixture(tmp_path)
    manifest.write_text(json.dumps(dict(schema='formal_drl_source_v1',files={'../outside.py':'0'*64})))
    with pytest.raises(ValueError,match='filename'):verifier()(package,manifest)


def test_portable_config_preserves_formal_training_values():
    import yaml
    from lunar_drl_exploration.config import training_config_from_record
    record=yaml.safe_load((ROOT/'config/drl_candidate_geometry_v3.yaml').read_text())
    manifest=json.loads((ROOT/'docs/drl/formal-runtime-source-manifest.json').read_text())
    digest=hashlib.sha256(json.dumps(record,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    assert digest==manifest['portable_config_sha256']
    config=training_config_from_record(record)
    assert config.model.architecture_version=='candidate_geometry_v3'
    assert (config.model.width,config.model.heads,config.model.layers)==(128,8,6)
    assert (config.goal_position_tolerance_m,config.goal_yaw_tolerance_rad)==(.1,.05)
    assert not Path(config.artifact_root).is_absolute()
