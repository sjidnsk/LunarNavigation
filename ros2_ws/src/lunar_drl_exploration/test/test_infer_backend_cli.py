"""仅解析配置/参数，不调用 main、Actor、ROS、训练或评测。"""
import json
from dataclasses import replace
import pytest
from lunar_drl_exploration.cli import parser,load_config
from lunar_drl_exploration.config import TrainingConfig,config_record


@pytest.mark.parametrize('configured,flag,expected',[
    (None,None,'incremental_v2'),('nav1cm',None,'nav1cm'),
    (None,'nav1cm','nav1cm'),('nav1cm','nav1cm','nav1cm'),
    ('nav1cm','incremental_v2','incremental_v2'),('incremental_v2','nav1cm','nav1cm')])
def test_infer_backend_precedence_with_serialized_config(tmp_path,configured,flag,expected):
    config=TrainingConfig() if configured is None else replace(TrainingConfig(),navigation_backend=configured)
    record=config_record(config)
    if configured is None:assert 'navigation_backend' not in record
    path=tmp_path/'inference.json';path.write_text(json.dumps(record),encoding='utf-8')
    args=['infer','--actor','unused-readonly.pt','--config',str(path)]
    if flag is not None:args+=['--navigation-backend',flag]
    loaded=load_config(parser().parse_args(args))
    assert loaded.navigation_backend==expected
    assert json.loads(path.read_text())==record


@pytest.mark.parametrize('record,match',[
    ({'navigation_backend':'typo'},'navigation_backend'),
    ({'unrecognized_setting':True},'unknown training setting')])
def test_infer_still_rejects_invalid_configuration(tmp_path,record,match):
    path=tmp_path/'invalid.json';path.write_text(json.dumps(record),encoding='utf-8')
    args=parser().parse_args(['infer','--actor','unused.pt','--config',str(path),'--navigation-backend','nav1cm'])
    with pytest.raises(ValueError,match=match):load_config(args)


def test_training_config_loader_and_flags_do_not_accept_optional_inference_backend(tmp_path):
    path=tmp_path/'not-a-training-run.json'
    path.write_text(json.dumps({'navigation_backend':'nav1cm'}),encoding='utf-8')
    with pytest.raises(ValueError,match='unknown training setting'):
        load_config(parser().parse_args(['train','--config',str(path)]))
    with pytest.raises(SystemExit):parser().parse_args(['train','--navigation-backend','nav1cm'])


def test_default_infer_config_matches_legacy_serialization():
    loaded=load_config(parser().parse_args(['infer','--actor','unused.pt']))
    assert config_record(loaded)==config_record(TrainingConfig())
    assert loaded.navigation_backend=='incremental_v2'
