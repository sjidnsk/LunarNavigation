"""100 m at 0.2 m/cell must retain exact beams without huge prefix caches."""
import math
import numpy as np
import lunar_drl_terrain_native as native


def test_500_cell_radius_has_bounded_cache_and_preserves_full_range():
    b=np.zeros((1005,1005),np.uint8);out=np.empty_like(b)
    native.observe(b,502,502,500.,0.,2*math.pi,out)
    y,x=np.indices(b.shape);distance=np.hypot(x-502,y-502)
    assert np.all(out[distance<=500.])
    assert not np.any(out[distance>501.])
    assert out[502,1002] and not out[502,1003]
    info=native.visibility_cache_info();assert info['bytes']<=info['limit_bytes']


def test_long_range_queries_agree_with_observation_for_narrow_and_all_round_fov():
    b=np.zeros((71,601),np.uint8);b[:,100]=2;b[30:41,100]=0
    b[34,51]=2  # Neighbor corner contact; both simultaneous cells are emitted.
    source=(50,35);targets=np.array([(x,y) for x in (49,50,51,52,99,100,101,300,550,551,600)
        for y in (0,30,34,35,36,40,70)],np.int64)
    full=np.empty_like(b);native.observe(b,*source,500.,0.,2*math.pi,full)
    seen=np.empty((len(targets),1),np.uint8)
    native.visible_targets(b,*source,500.,targets,seen)
    assert np.array_equal(seen[:,0],full[targets[:,1],targets[:,0]])
    headings=np.linspace(-math.pi,math.pi,8,endpoint=False).reshape(8,1)
    for fov in (math.pi/3,2*math.pi):
        directed=np.empty((len(targets),8),np.uint8)
        native.directional_targets(b,*source,500.,targets,headings,fov,directed)
        for j,yaw in enumerate(headings[:,0]):
            mask=np.empty_like(b);native.observe(b,*source,500.,float(yaw),fov,mask)
            assert np.array_equal(directed[:,j],mask[targets[:,1],targets[:,0]])
    narrow=np.empty_like(b);native.observe(b,*source,500.,0.,1e-9,narrow)
    assert not np.any(narrow[:35]) and not np.any(narrow[36:])
    assert narrow[35,550] and not narrow[35,551]


def test_dense_long_range_frontier_query_finishes_with_exact_heading_masks():
    # Graph gain extraction asks about thousands of frontiers from one stance.
    # Give the entire child 10 s (including imports), far below a 180 s reset
    # watchdog after hundreds of stances, without a fragile subsecond gate.
    import subprocess
    import sys
    subprocess.run([sys.executable, '-c', '''
import numpy as np
import lunar_drl_terrain_native as n
rng=np.random.default_rng(902)
b=np.where(rng.random((301,301))<.1,2,0).astype(np.uint8);b[150,150]=0
heads=np.linspace(-np.pi,np.pi,8,endpoint=False).reshape(8,1)
targets=np.indices(b.shape)[::-1].reshape(2,-1).T.copy().astype(np.int64)
for i,fov in enumerate((np.pi/3,2*np.pi)*4):
    b[:,153]=2 if i%2==0 else 0  # Reused storage must not reuse stale occlusion.
    out=np.empty((len(targets),8),np.uint8)
    n.directional_targets(b,150,150,500.,targets,heads,fov,out)
    for j,yaw in enumerate(heads[:,0]):
        expected=np.empty_like(b)
        n.observe(b,150,150,500.,float(yaw),fov,expected)
        assert np.array_equal(out[:,j],expected.ravel())
'''], check=True, timeout=10)
