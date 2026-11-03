"""共享事实不再维护重复分类；观测变化与导航状态分别验证。"""
from dataclasses import fields
from scripts.navigation.one_cm.nav1cm.base_map import SourceEvidence


def test_shared_observation_contract_has_no_duplicate_classification_history():
    assert 'classification_fragments' not in {f.name for f in fields(SourceEvidence)}
