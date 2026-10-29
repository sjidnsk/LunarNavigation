"""native policy second-stage tests use controlled snapshots, never SDK success evidence."""
import hashlib
import importlib
import json
from pathlib import Path
import numpy as np
import pytest
from tools.isaac_viper.config import load_profile
from tools.isaac_viper.navigation_scene import SOURCE_SHA

ROOT=Path(__file__).resolve().parents[2]

def fixture(tmp_path):
    z=np.zeros((120,120),np.float32); relief=np.zeros_like(z)
    z[20:100,58:62]=1.; relief[20:100,58:62]=1.
    states=np.ones(z.shape,np.uint8); states[18:102,56:64]=2
    intrinsic=np.ones_like(states); intrinsic[20:100,58:62]=2
    profile=load_profile(ROOT/'config/isaac_viper_navigation_demo.yaml')
    bounds=[0.,120.,0.,120.]; resolution=1.; layers={'controlled-source':SOURCE_SHA}
    source=tmp_path/'map.npz'; np.savez_compressed(source,elevation=z,relief=relief,bounds_xy_m=bounds,resolution_m=resolution)
    source_receipt=dict(schema='lunar.viper-navigation-scene.v1',region='A',source_sha256=SOURCE_SHA,
        source_layer_sha256=layers,map_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),capability_limits=profile['capability'])
    (tmp_path/'navigation-scene.json').write_text(json.dumps(source_receipt),encoding='utf-8')
    native=tmp_path/'native.npz'
    np.savez_compressed(native,states=states,intrinsic_states=intrinsic,observed=intrinsic,elevation_m=z,
        source_relief_m=relief,bounds_xy_m=bounds,resolution_m=resolution,origin_xy=[0.,0.])
    response=tmp_path/'response.cdr'; response.write_bytes(b'controlled test artifact; not an actual ROS response')
    manifest=dict(response_cdr='response.cdr',response_sha256=hashlib.sha256(response.read_bytes()).hexdigest(),schema='native-policy-aligned500-v1',source_map=str(source),source_map_sha256=source_receipt['map_sha256'],
        native_npz='native.npz',native_sha256=hashlib.sha256(native.read_bytes()).hexdigest(),epoch='controlled-epoch',
        fine_revision=3,raw_elevation_revision=2,profile_hash='a'*16,frame_id='map',origin=[0.,0.,0.],resolution_m=resolution,
        bounds_xy_m=bounds,array_shape=[120,120],wire={'unknown':0,'free':1,'blocked':2},
        index_order='array[y,x] ascending world axes; cell centers origin+(index+0.5)*resolution; policy indices0..499 exactly source grid')
    path=tmp_path/'manifest.json'; path.write_text(json.dumps(manifest),encoding='utf-8')
    return path,profile,z,relief,bounds,resolution,layers,manifest

def module():
    assert importlib.util.find_spec('tools.isaac_viper.native_policy') is not None,'native policy validator missing'
    return importlib.import_module('tools.isaac_viper.native_policy')

def load(m,values):
    path,profile,z,relief,bounds,resolution,layers,_=values
    return m.load_native_policy(path,profile,z,relief,bounds,resolution,layers,'a'*16)

def test_native_free_corridor_and_endpoint_inflation_rejection(tmp_path):
    m=module(); values=fixture(tmp_path); policy=load(m,values)
    start,goal,proof=m.select_native_route(policy,values[2],values[3],values[4],1.,[5.5,5.5],[114.5,114.5],100.,values[1]['capability'])
    assert start==[5.5,5.5] and goal==[114.5,114.5]
    assert proof['path_length_m']>=100 and proof['line_of_sight_blocked']
    assert proof['native_endpoint_validation']=='PASS'
    assert proof['native_policy']['profile_hash']=='a'*16
    assert 'path' not in proof
    # inflated BLOCKED despite observed/intrinsic FREE must be rejected.
    with pytest.raises(ValueError,match='native.*FREE'):
        m.select_native_route(policy,values[2],values[3],values[4],1.,[56.5,19.5],[114.5,114.5],100.,values[1]['capability'])

@pytest.mark.parametrize('field,value',[('response_sha256','b'*64),('source_map_sha256','b'*64),('profile_hash','b'*16),('resolution_m',.2),('origin',[1.,0.,0.]),('wire',{'unknown':0,'free':2,'blocked':1})])
def test_native_snapshot_identity_mismatch_rejected(tmp_path,field,value):
    m=module(); values=fixture(tmp_path); manifest=values[-1]; manifest[field]=value
    values[0].write_text(json.dumps(manifest),encoding='utf-8')
    with pytest.raises(ValueError): load(m,values)

def test_native_height_tamper_and_disconnected_corridor_rejected(tmp_path):
    m=module(); values=fixture(tmp_path); policy=load(m,values)
    z=values[2].copy(); z[5,5]=99.
    changed=list(values); changed[2]=z
    with pytest.raises(ValueError,match='elevation'): load(m,changed)
    policy['free'][:,56:64]=False
    with pytest.raises(ValueError,match='connected'):
        m.select_native_route(policy,values[2],values[3],values[4],1.,[5.5,5.5],[114.5,114.5],100.,values[1]['capability'])
