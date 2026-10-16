"""Structural split identity and geometric safety, independent of native bindings."""
import itertools
import numpy as np
import pytest
from lunar_drl_exploration.cave_topology import (
    CONSTRUCTION_KINDS, graph_signature, isomorphic, mother_catalog, manifest_hash,
    select_mother, build_geometry, validate_geometry, validate_graph,
)


def test_exact_identity_ignores_node_labels_but_distinguishes_cycle_structure():
    cycle=[(i,(i+1)%8) for i in range(8)]
    permutation=[5,2,6,0,7,1,4,3]
    relabel=[(permutation[a],permutation[b]) for a,b in cycle]
    assert graph_signature(8,cycle)==graph_signature(8,relabel)
    assert isomorphic(8,cycle,8,relabel)
    # Both cubic, 8-node connected graphs; WL/degree signatures alone are insufficient.
    cube=[(i,i^(1<<k)) for i in range(8) for k in range(3) if i<(i^(1<<k))]
    mobius=cycle+[(i,i+4) for i in range(4)]
    assert not isomorphic(8,cube,8,mobius)
    assert graph_signature(8,cube)!=graph_signature(8,mobius)


def test_catalog_is_fixed_and_split_groups_are_nonisomorphic():
    catalog=mother_catalog()
    assert catalog==mother_catalog()
    assert len(manifest_hash())==64
    assert {s:sum(r['split']==s for r in catalog) for s in ('train','validation','test')}==dict(train=64,validation=24,test=24)
    for a,b in itertools.combinations(catalog,2):
        assert not isomorphic(a['node_count'],a['edges'],b['node_count'],b['edges'])
    assert all('topology_family' not in row for row in catalog)


def test_deterministic_embedding_fits_minimum_extent_and_has_no_unrecorded_contacts():
    for row in mother_catalog():
        a=build_geometry(row,12,30.)
        assert a==build_geometry(row,12,30.)
        validate_geometry(a)
        assert all(abs(x)+r<=15 and abs(y)+r<=15 for x,y,r in a['rooms'])
        assert len(a['passages'])==len(row['edges'])
        assert a['anchor_node']==0


def test_geometric_validator_rejects_unrecorded_crossing_and_room_contact():
    geometry=dict(rooms=[(-6.,0.,2.),(6.,0.,2.),(0.,-6.,2.),(0.,6.,2.)],
        edges=[(0,1),(2,3)],passages=[(-6.,0.,6.,0.,1.3),(0.,-6.,0.,6.,1.3)])
    with pytest.raises(ValueError,match='contact|cross'):validate_geometry(geometry)
    geometry=dict(rooms=[(0.,0.,2.),(2.,0.,2.)],edges=[],passages=[])
    with pytest.raises(ValueError,match='room'):validate_geometry(geometry)


def test_raster_contraction_checks_actual_connections_and_rejects_shortcut():
    from lunar_drl_exploration.cave_topology import qualify_grid
    mother=select_mother('train',0)
    geom=build_geometry(mother,12,30.)
    resolution=.2;origin=(-16.,-16.);shape=(160,160)
    xx=origin[0]+(np.arange(shape[1])+.5)*resolution
    yy=origin[1]+(np.arange(shape[0])+.5)*resolution
    from lunar_drl_exploration.cave_topology import primitive_masks
    rooms,passages=primitive_masks(geom,shape,origin,resolution)
    floor=np.logical_or.reduce(rooms+passages)
    summary=qualify_grid(geom,floor,shape,origin,resolution)
    assert summary['realized_edges']==geom['edges']
    assert summary['grid_topology_validation']=='room_corridor_contraction_passed'
    floor[:]=True
    with pytest.raises(ValueError,match='unlabelled|shortcut|contact'):qualify_grid(geom,floor,shape,origin,resolution)


def test_curriculum_specs_restore_same_structural_sequence_and_split_exclusion():
    from types import SimpleNamespace
    from lunar_drl_exploration.course_two_simple import fresh_scene,cave_evaluation_specs
    from lunar_drl_exploration.cave_topology import VERSION
    config=SimpleNamespace(seed=20261014,curriculum_extents_m=((30.,100.),),
        ordered_budget_rule=dict(mode='linear_area',min_extent_m=50.,max_extent_m=100.,round_to=32,
            min_decisions=256,max_decisions=512),course_two_scene_profile='',
        cave_generator_version=VERSION,cave_split_manifest_hash=manifest_hash())
    train=[fresh_scene(config,'cave_inside',i) for i in range(80)]
    assert train==[fresh_scene(config,'cave_inside',i) for i in range(80)]
    validation=cave_evaluation_specs(config,'validation');test=cave_evaluation_specs(config,'test')
    quick=cave_evaluation_specs(config,'quick')
    assert (len(validation),len(test),len(quick))==(60,100,20)
    assert quick==validation[:20]
    assert not ({s['mother_graph_id'] for s in train}&{s['mother_graph_id'] for s in validation})
    assert not ({s['mother_graph_id'] for s in test}&{s['mother_graph_id'] for s in validation+train})
    assert all('topology_family' not in s for s in train+validation+test)
    config.cave_split_manifest_hash='wrong'
    with pytest.raises(ValueError,match='manifest'):fresh_scene(config,'cave_inside',0)


def test_all_fixed_mother_layouts_recover_exact_raster_graph_at_minimum_extent():
    from lunar_drl_exploration.cave_topology import primitive_masks,qualify_grid
    for mother in mother_catalog():
        geometry=build_geometry(mother,99,30.)
        shape=(160,160);origin=(-16.,-16.);resolution=.2
        rooms,passages=primitive_masks(geometry,shape,origin,resolution)
        floor=np.logical_or.reduce(rooms+passages)
        assert qualify_grid(geometry,floor,shape,origin,resolution)['realized_edges']==geometry['edges']


def test_topology_constructor_coverage_is_internal_and_valid():
    from lunar_drl_exploration.cave_topology import _lattice_candidate
    for kind in CONSTRUCTION_KINDS:
        rng=np.random.default_rng(19)
        candidate=None
        while candidate is None:candidate=_lattice_candidate(rng,kind)
        points,edges=candidate
        validate_graph(len(points),edges,kind)
