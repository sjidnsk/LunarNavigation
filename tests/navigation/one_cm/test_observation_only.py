"""共享事实不再维护重复分类；观测变化与导航状态分别验证。"""
from dataclasses import fields
from scripts.navigation.one_cm.nav1cm.base_map import SourceEvidence


def test_shared_observation_contract_has_no_duplicate_classification_history():
    assert 'classification_fragments' not in {f.name for f in fields(SourceEvidence)}


import numpy as np
from dataclasses import replace
from test_p3_terrain_native import compiled,_api
from scripts.navigation.one_cm.nav1cm import p3_evidence as kernel
from scripts.navigation.one_cm.nav1cm.base_map import MEASUREMENT_DTYPE


def incoming(height=0.,identity=1,x=0):
    return SourceEvidence(identity,.01,(0.,0.),0.,np.array([(x,0,height,0,np.nan,np.nan)],dtype=MEASUREMENT_DTYPE),())


def test_identical_observation_returns_same_sources_without_new_fragments(compiled,monkeypatch):
    with _api().NativeTerrain(compiled[0]) as ops:
        first=kernel.update_evidence((),incoming(),(0,),ops)
        calls=[];original=kernel.EvidenceFragment
        def create(*args):calls.append(1);return original(*args)
        monkeypatch.setattr(kernel,'EvidenceFragment',create)
        again=kernel.update_evidence(first,incoming(),(0,),ops)
        assert again is first
        assert calls==[]


def test_measurement_change_reuses_geometry_and_preserves_old_snapshot(compiled):
    with _api().NativeTerrain(compiled[0]) as ops:
        first=kernel.update_evidence((),incoming(),(0,),ops)
        second=kernel.update_evidence(first,incoming(.125),(0,),ops)
        assert second[0].observation_fragments[0] is first[0].observation_fragments[0]
        assert first[0].measurements['height_m'][0]==0.
        assert second[0].measurements['height_m'][0]==.125


def test_new_local_cell_preserves_unaffected_fragment_objects(compiled):
    with _api().NativeTerrain(compiled[0]) as ops:
        first=kernel.update_evidence((),incoming(),(0,),ops)
        second=kernel.update_evidence(first,incoming(.1,x=1),(0,),ops)
        assert second[0].observation_fragments[0] is first[0].observation_fragments[0]
        assert len(second[0].measurements)==2


def test_foreign_local_update_only_cuts_intersecting_fragments(compiled,monkeypatch):
    values=np.array([(x,0,0.,0,np.nan,np.nan) for x in range(128)],dtype=MEASUREMENT_DTYPE)
    with _api().NativeTerrain(compiled[0]) as ops:
        first=kernel.update_evidence((),SourceEvidence(1,.01,(0.,0.),0.,values,()),tuple(range(128)),ops)
        calls=[];cut=kernel._cut
        def counted(*args,**kwargs):calls.append(1);return cut(*args,**kwargs)
        monkeypatch.setattr(kernel,'_cut',counted)
        second=kernel.update_evidence(first,incoming(identity=2),(0,),ops)
        assert len(calls)==1
        previous=first[0].observation_fragments
        unchanged=sum(any(f is p for f in second[0].observation_fragments) for p in previous)
        assert unchanged>=126
