"""Identity-local region attention and strict v2 model contract."""
from dataclasses import asdict, replace
import copy
import numpy as np
import pytest
import torch
from lunar_drl_exploration.config import ModelConfig, canonical_model_config
from lunar_drl_exploration.batch import pack_observations
from lunar_drl_exploration.model import Actor, Critic
from test_model import observation, state, scene


def selected(current=0, nodes=(2,2,0,2)):
    obs = observation(4, 3)
    indices = np.asarray(nodes, dtype=np.int64)
    yaws = np.arange(len(indices)) * np.pi / 4
    return replace(obs, current_index=current, action_nodes=indices, action_yaws=yaws,
                   goals=np.column_stack((obs.positions[indices], yaws)))


def test_query_identity_and_action_mapping():
    index = pack_observations([selected()], 'cpu').region_queries
    assert index.nodes.tolist() == [2,0]
    assert index.actions.tolist() == [0,0,1,0]
    assert index.current.tolist() == [1]
    index = pack_observations([selected(1)], 'cpu').region_queries
    assert index.nodes.tolist() == [2,0,1]
    assert index.actions.tolist() == [0,0,1,0]
    assert index.current.tolist() == [2]


def test_query_pairs_are_sample_local():
    left = selected()
    right = replace(selected(1), polygon=left.polygon[:3], positions=np.zeros_like(left.positions))
    batch = pack_observations([left,right], 'cpu')
    index = batch.region_queries
    assert len(index.pair_edges) == 2*5 + 3*3
    assert index.nodes.tolist() == [2,0,6,4,5]
    assert torch.equal(index.groups[index.pair_queries], batch.polygon_groups[index.pair_edges])
    for value in vars(index).values(): assert value.dtype == torch.long


@pytest.mark.parametrize('nodes', [(), (0,), (2,0,2), (0,2,2)])
def test_query_empty_single_and_permuted_actions(nodes):
    batch = pack_observations([selected(nodes=nodes)], 'cpu')
    index = batch.region_queries
    assert index.actions.shape == (len(nodes),)
    assert index.nodes[index.current].tolist() == [0]
    assert index.nodes[index.actions].tolist() == list(nodes)
    assert batch.action_counts == (len(nodes),)


def test_region_read_matches_dense_reference():
    from lunar_drl_exploration.model import RegionRead
    torch.manual_seed(61)
    config = ModelConfig(width=16, heads=2, layers=1)
    read = RegionRead(config).double()
    reference = copy.deepcopy(read)
    batch = pack_observations([selected(),replace(selected(1),polygon=selected().polygon[:3])], 'cpu')
    index = batch.region_queries
    h = torch.randn(len(index.nodes),16,dtype=torch.double,requires_grad=True)
    xy = torch.randn(len(index.nodes),2,dtype=torch.double,requires_grad=True)
    tokens = torch.randn(len(batch.polygon_edges),16,dtype=torch.double,requires_grad=True)
    actual = read(h,xy,tokens,index)
    q = reference.query(torch.cat((reference.norm(h),xy),-1)).reshape(-1,2,8)
    k,v = reference.key_value(tokens).reshape(-1,2,2,8).unbind(1)
    rows=[]
    for row,group in enumerate(index.groups.tolist()):
        mask=batch.polygon_groups==group
        weights=torch.softmax((q[row]*k[mask]).sum(-1)/np.sqrt(8),0)
        rows.append(reference.project((weights[...,None]*v[mask]).sum(0).flatten()))
    expected=torch.stack(rows)
    torch.testing.assert_close(actual,expected)
    inputs=(h,xy,tokens)
    ga=torch.autograd.grad(actual.square().sum(),inputs+tuple(read.parameters()),retain_graph=True)
    ge=torch.autograd.grad(expected.square().sum(),inputs+tuple(reference.parameters()))
    for a,e in zip(ga,ge): torch.testing.assert_close(a,e)


def test_region_read_gradients_and_parameter_count():
    from lunar_drl_exploration.model import RegionRead
    read=RegionRead(ModelConfig())
    assert sum(p.numel() for p in read.parameters()) == 66560
    batch=pack_observations([selected()], 'cpu')
    h=torch.randn(2,128,requires_grad=True); xy=torch.randn(2,2,requires_grad=True)
    tokens=torch.randn(5,128,requires_grad=True)
    read(h,xy,tokens,batch.region_queries).square().sum().backward()
    for p in read.parameters():
        assert torch.isfinite(p.grad).all() and torch.count_nonzero(p.grad)>0


def test_measured_encoder_has_single_polygon_path():
    from lunar_drl_exploration.model import RegionRead
    actor,critic=Actor(),Critic()
    assert sum(p.numel() for p in actor.parameters()) == 1049393
    assert sum(p.numel() for p in critic.parameters()) == 1977953
    for network in (actor,critic):
        assert not hasattr(network.encoder,'region_query')
        assert network.encoder.context[0].in_features == 384
        assert sum(isinstance(m,RegionRead) for m in network.modules()) == 1
        assert network.encoder.polygon(torch.randn(5,4)).shape == (5,128)
    assert critic.head[0].in_features == 769


def test_current_and_candidates_share_relation():
    obs=selected(); batch=pack_observations([obs],'cpu')
    critic=Critic(ModelConfig(width=16,heads=2,layers=1))
    original=[]; originals=[]; enhanced=[]; queries=[]
    critic.encoder.graph.register_forward_hook(lambda m,a,o:(original.append(o.clone()), originals.append(o)) and None)
    critic.encoder.region_read.register_forward_hook(lambda m,a,o:enhanced.append(a[0]+o))
    critic.encoder.read.register_forward_pre_hook(lambda m,a:queries.append(a[0]))
    critic.truth_read.register_forward_pre_hook(lambda m,a:queries.append(a[0]))
    critic([obs],[state(obs=obs)],{'s':scene()})
    index=batch.region_queries
    for q in queries: torch.testing.assert_close(q,enhanced[0][index.current])
    actions,current=critic.encoder(batch)
    torch.testing.assert_close(actions[:,:16],enhanced[-1][index.actions])
    for before,after in zip(original,originals): torch.testing.assert_close(before,after,atol=0,rtol=0)
    torch.testing.assert_close(current,enhanced[-1][index.current])
    assert not torch.allclose(current,original[-1][batch.graph.current])


@pytest.mark.parametrize('record', [{}, {'architecture_version':'legacy'},
    {'architecture_version':'region_relation_v2','region_query_mode':'none'}])
def test_version_rejects_legacy_records(record):
    with pytest.raises(ValueError,match='architecture|region_query'):
        canonical_model_config(record)


def test_version_roundtrip_and_constructor():
    assert canonical_model_config(asdict(ModelConfig())) == asdict(ModelConfig())
    assert ModelConfig().architecture_version == 'region_relation_v2'
    with pytest.raises(ValueError,match='architecture'): ModelConfig(architecture_version='old')


@pytest.mark.parametrize('copies', ['learner','config','semantics','collector'])
@pytest.mark.parametrize('mutation', ['missing','old','residual'])
def test_version_four_checkpoint_copies_reject_without_writes(tmp_path,copies,mutation):
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.replay import ReplayBuffer
    from lunar_drl_exploration.schedule import UpdateSchedule
    from lunar_drl_exploration.checkpoint import TrainingState, UpdateBoundary
    from test_replay_checkpoint import capture_state, transition, scene as replay_scene
    config=replace(TrainingConfig(),model=ModelConfig(width=16,heads=2,layers=1),output_dir=str(tmp_path/'absent'))
    learner=SACLearner(config.model,config.learning)
    replay=ReplayBuffer(1000000); replay.add(transition(),{'s':replay_scene()})
    schedule=UpdateSchedule(); schedule.collected()
    saved=capture_state(learner,config,replay,schedule,UpdateBoundary())
    record=copy.deepcopy(saved.record)
    records=dict(learner=record['learner']['model_config'],config=record['config']['model'],
                 semantics=record['semantics']['model'],collector=record['collector_state']['actor_record']['model_config'])
    model=records[copies]
    if mutation=='missing': model.pop('architecture_version')
    elif mutation=='old': model['architecture_version']='region_query_v1'
    else: model['region_query_mode']='none'
    source=tmp_path/'source.pt'; torch.save(record,source); before=source.read_bytes()
    with pytest.raises(ValueError,match='architecture|region_query'):
        TrainingState(record).validate(config)
    assert source.read_bytes()==before and not (tmp_path/'absent').exists()


@pytest.mark.parametrize('mutation', ['missing','old','residual'])
def test_version_runtime_and_export_reject_before_replacing_output(tmp_path,mutation):
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.runtime import ActorPolicy
    from lunar_drl_exploration.evaluation import export_actor
    learner=SACLearner(ModelConfig(width=16,heads=2,layers=1))
    record=learner.actor_state()
    if mutation=='missing': record['model_config'].pop('architecture_version')
    elif mutation=='old': record['model_config']['architecture_version']='old'
    else: record['model_config']['region_query_mode']='none'
    source=tmp_path/'source.pt'; torch.save(record,source)
    output=tmp_path/'actor.pt'; output.write_bytes(b'preserved')
    with pytest.raises(ValueError,match='architecture|region_query'): ActorPolicy.from_record(record)
    with pytest.raises(ValueError,match='architecture|region_query'): export_actor(source,output)
    assert output.read_bytes()==b'preserved'


def test_single_token_relation_has_constant_query_weights():
    from lunar_drl_exploration.batch import RegionQueryIndex
    from lunar_drl_exploration.model import RegionRead
    read=RegionRead(ModelConfig(width=16,heads=2,layers=1))
    index=RegionQueryIndex(*(torch.tensor(x,dtype=torch.long) for x in
        ([0,1],[0,0],[0],[1],[0,1],[0,0])))
    output=read(torch.randn(2,16),torch.randn(2,2),torch.randn(1,16),index)
    torch.testing.assert_close(output[0],output[1])
    output.square().sum().backward()
    assert torch.count_nonzero(read.query.weight.grad)==0


def test_cave_dataset_identity_is_explicit_and_resume_bound():
    from lunar_drl_exploration.config import TrainingConfig,resume_semantics,config_record,training_config_from_record
    cfg=replace(TrainingConfig(),cave_generator_version='cave_topology_v2',cave_split_manifest_hash='a'*64)
    assert resume_semantics(cfg)['cave_dataset']==dict(generator_version='cave_topology_v2',split_manifest_hash='a'*64)
    assert training_config_from_record(config_record(cfg))==cfg
    for values in (dict(cave_generator_version='v2'),dict(cave_split_manifest_hash='A'*64),
        dict(cave_generator_version='cave_topology_v2',cave_split_manifest_hash='bad')):
        with pytest.raises(ValueError): replace(TrainingConfig(),**values)


@pytest.mark.skipif(not torch.cuda.is_available(),reason='CUDA unavailable')
def test_region_read_cpu_gpu_forward_and_gradients():
    from lunar_drl_exploration.model import RegionRead
    cfg=ModelConfig(width=16,heads=2,layers=1)
    cpu=RegionRead(cfg); gpu=copy.deepcopy(cpu).cuda()
    observations=[selected(),replace(selected(1),polygon=selected().polygon[:3])]
    a=pack_observations(observations,'cpu'); b=pack_observations(observations,'cuda')
    h=torch.randn(len(a.region_queries.nodes),16,requires_grad=True)
    xy=torch.randn(len(h),2,requires_grad=True)
    tokens=torch.randn(len(a.polygon_edges),16,requires_grad=True)
    inputs=[h,xy,tokens]; gpu_inputs=[t.detach().cuda().requires_grad_() for t in inputs]
    left=cpu(*inputs,a.region_queries); right=gpu(*gpu_inputs,b.region_queries)
    torch.testing.assert_close(left,right.cpu(),atol=2e-5,rtol=2e-4)
    left.square().sum().backward(); right.square().sum().backward()
    for p,q in zip(inputs+list(cpu.parameters()),gpu_inputs+list(gpu.parameters())):
        torch.testing.assert_close(p.grad,q.grad.cpu(),atol=2e-5,rtol=2e-4)
