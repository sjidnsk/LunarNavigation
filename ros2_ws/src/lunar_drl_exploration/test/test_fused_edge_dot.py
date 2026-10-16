import pytest
import torch
from lunar_drl_exploration.edge_dot import edge_dot_eager,edge_dot_cuda


def tensors(device):
    torch.manual_seed(15)
    q=torch.randn(23,8,16,device=device,requires_grad=True)
    k=torch.randn(23,8,16,device=device,requires_grad=True)
    source=torch.tensor([0,1,2,5,5,7,20,22,22,3],device=device)
    target=torch.tensor([1,2,2,7,20,3,0,22,2,3],device=device)
    return q,k,source,target


def test_cpu_eager_is_original_formula_and_gradient():
    q,k,s,t=tensors('cpu');a=edge_dot_eager(q,k,s,t)
    b=(q[t]*k[s]).sum(-1)/(q.shape[-1]**.5)
    torch.testing.assert_close(a,b,rtol=0,atol=0)
    ga=torch.autograd.grad(a.square().sum(),(q,k),retain_graph=True)
    gb=torch.autograd.grad(b.square().sum(),(q,k))
    for x,y in zip(ga,gb):torch.testing.assert_close(x,y,rtol=0,atol=0)


@pytest.mark.skipif(not torch.cuda.is_available(),reason='CUDA unavailable')
def test_cuda_fusion_matches_forward_and_backward_with_repeated_indices():
    q,k,s,t=tensors('cuda');a=edge_dot_eager(q,k,s,t);b=edge_dot_cuda(q,k,s,t)
    torch.testing.assert_close(a,b,rtol=1e-4,atol=1e-5)
    ga=torch.autograd.grad(a.square().sum(),(q,k),retain_graph=True)
    gb=torch.autograd.grad(b.square().sum(),(q,k))
    for x,y in zip(ga,gb):torch.testing.assert_close(x,y,rtol=1e-4,atol=1e-5)
    # One callable handles variable node/edge counts with the same head dimensions.
    q2=q[:17].detach().requires_grad_();k2=k[:17].detach().requires_grad_()
    torch.testing.assert_close(edge_dot_cuda(q2,k2,s[:5],t[:5].clamp_max(16)),edge_dot_eager(q2,k2,s[:5],t[:5].clamp_max(16)),rtol=1e-4,atol=1e-5)


def test_sparse_layer_routes_dot_without_new_parameters(monkeypatch):
    import lunar_drl_exploration.model as model
    from lunar_drl_exploration.batch import pack_observations
    from lunar_drl_exploration.config import ModelConfig
    from test_model import observation
    calls=[]
    def record(q,k,s,t):calls.append((q.device.type,len(s)));return edge_dot_eager(q,k,s,t)
    monkeypatch.setattr(model,'edge_dot_cuda',record)
    layer=model.SparseAttentionLayer(ModelConfig(width=16,heads=2,layers=1))
    keys=tuple(layer.state_dict());batch=pack_observations([observation()],device='cpu')
    result=layer(torch.randn(len(batch.graph.features),16),batch.graph)
    assert calls and result.shape==(len(batch.graph.features),16)
    assert tuple(layer.state_dict())==keys


@pytest.mark.skipif(not torch.cuda.is_available(),reason='CUDA unavailable')
def test_compiler_failure_falls_back_once_without_swallowing_cuda_oom(monkeypatch,caplog):
    import lunar_drl_exploration.edge_dot as module
    from torch._dynamo.exc import Unsupported
    q,k,s,t=tensors('cuda');calls=[]
    # Torch/ClearML may configure propagation independently of pytest's root
    # capture. Attach capture to the exact logger whose warning is contracted.
    import logging
    logger=logging.getLogger(module.__name__)
    monkeypatch.setattr(logger,'handlers',[caplog.handler])
    monkeypatch.setattr(logger,'propagate',False)
    def broken(*args):calls.append(1);raise Unsupported('test compiler unavailable')
    monkeypatch.setattr(module,'_disabled',False)
    monkeypatch.setattr(module,'_compiled',lambda:broken)
    expected=edge_dot_eager(q,k,s,t)
    torch.testing.assert_close(module.edge_dot_cuda(q,k,s,t),expected,rtol=0,atol=0)
    torch.testing.assert_close(module.edge_dot_cuda(q,k,s,t),expected,rtol=0,atol=0)
    assert len(calls)==1 and 'eager' in caplog.text
    monkeypatch.setattr(module,'_disabled',False)
    def oom(*args):raise torch.OutOfMemoryError('test oom')
    monkeypatch.setattr(module,'_compiled',lambda:oom)
    with pytest.raises(torch.OutOfMemoryError):module.edge_dot_cuda(q,k,s,t)

    from torch._dynamo.exc import BackendCompilerFailed
    def wrapped_oom(*args):raise BackendCompilerFailed(oom,torch.OutOfMemoryError('wrapped test oom'),None)
    monkeypatch.setattr(module,'_compiled',lambda:wrapped_oom)
    with pytest.raises(BackendCompilerFailed):module.edge_dot_cuda(q,k,s,t)
