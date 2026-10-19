import numpy as np
from lunar_drl_exploration.exploration_visuals import encode_delta, decode_delta


def test_map_delta_preserves_unknown_observed_and_navigation_separately():
    index=np.array([0,5,300],np.uint32)
    observed=np.array([1,1,0],np.uint8)
    intrinsic=np.array([1,2,0],np.uint8)
    navigation=np.array([0,2,0],np.uint8)
    data=encode_delta(index,observed,intrinsic,navigation)
    actual=decode_delta(data)
    for name,value in [('i',index),('o',observed),('b',intrinsic),('m',navigation)]:
        assert np.array_equal(actual[name],value)
    assert len(data)==7*3


def test_ros_numpy_session_ids_are_json_serializable():
    import json
    from lunar_drl_exploration.exploration_visuals import json_default
    assert json.loads(json.dumps({'session':[np.uint8(255)]},default=json_default)) == {'session':[255]}
