"""The fused edge reduction preserves the existing sparse message and gradients."""
import importlib
from types import SimpleNamespace

import pytest
import torch


def message_module():
    try:
        return importlib.import_module('lunar_drl_exploration.edge_message')
    except ModuleNotFoundError as error:
        if error.name != 'lunar_drl_exploration.edge_message':
            raise
        pytest.fail('The same-formula fused edge message implementation is missing')


def original_message(weights, values, source, target, count):
    # Frozen pre-change expression, independent of the implementation under test.
    messages = weights[..., None] * values[source]
    output = messages.new_zeros((count,) + messages.shape[1:])
    return output.index_add_(0, target, messages)


def inputs(device, nodes=23, edges=51):
    generator = torch.Generator(device=device).manual_seed(20261013 + nodes)
    qkv = torch.randn(nodes, 3, 8, 16, device=device, generator=generator,
                      requires_grad=True)
    values = qkv.unbind(1)[2]
    assert not values.is_contiguous()
    weights = torch.randn(edges, 8, device=device, generator=generator, requires_grad=True)
    # Repeated sources and destinations, with the final three nodes isolated.
    source = torch.randint(nodes - 3, (edges,), device=device, generator=generator)
    target = torch.randint(nodes - 3, (edges,), device=device, generator=generator)
    return qkv, weights, values, source, target


def test_literal_repeated_edges_and_isolated_destinations_have_correct_gradients():
    module = message_module()
    values = torch.tensor([[[2., 3.]], [[5., 7.]], [[11., 13.]]], requires_grad=True)
    weights = torch.tensor([[.25], [.5], [2.], [-1.]], requires_grad=True)
    source = torch.tensor([0, 0, 1, 2])
    target = torch.tensor([1, 1, 1, 3])
    actual = module.edge_message_eager(weights, values, source, target, 5)
    expected = torch.tensor([[[0., 0.]], [[11.5, 16.25]], [[0., 0.]],
                             [[-11., -13.]], [[0., 0.]]])
    torch.testing.assert_close(actual, expected, rtol=0, atol=0)
    actual.sum().backward()
    torch.testing.assert_close(weights.grad, torch.tensor([[5.], [5.], [12.], [24.]]), rtol=0, atol=0)
    torch.testing.assert_close(values.grad, torch.tensor([[[.75, .75]], [[2., 2.]],
                                                         [[-1., -1.]]]), rtol=0, atol=0)


@pytest.mark.parametrize('nodes,edges', [(7, 1), (23, 51), (41, 127)])
def test_cpu_noncontiguous_qkv_view_matches_original_forward_and_backward(nodes, edges, monkeypatch):
    module = message_module()
    def forbidden_compile():
        pytest.fail('CPU execution must not invoke the CUDA compiler')
    monkeypatch.setattr(module, '_compiled', forbidden_compile)
    qkv, weights, values, source, target = inputs('cpu', nodes, edges)
    actual = module.edge_message_cuda(weights, values, source, target, nodes)
    expected = original_message(weights, values, source, target, nodes)
    torch.testing.assert_close(actual, expected, rtol=0, atol=0)
    actual_grad = torch.autograd.grad(actual.square().sum(), (qkv, weights), retain_graph=True)
    expected_grad = torch.autograd.grad(expected.square().sum(), (qkv, weights))
    for actual_value, expected_value in zip(actual_grad, expected_grad):
        torch.testing.assert_close(actual_value, expected_value, rtol=0, atol=0)


@pytest.mark.parametrize('count', [0, 5])
def test_empty_edges_preserve_empty_or_isolated_output_and_zero_gradients(count):
    module = message_module()
    values = torch.randn(count, 2, 3, requires_grad=True)
    weights = torch.empty(0, 2, requires_grad=True)
    indices = torch.empty(0, dtype=torch.long)
    output = module.edge_message_cuda(weights, values, indices, indices, count)
    torch.testing.assert_close(output, torch.zeros(count, 2, 3), rtol=0, atol=0)
    output.sum().backward()
    torch.testing.assert_close(values.grad, torch.zeros_like(values), rtol=0, atol=0)
    assert weights.grad.shape == (0, 2)


def test_sparse_layer_routes_message_without_changing_checkpoint_or_input_gradients(monkeypatch):
    module = message_module()
    import lunar_drl_exploration.model as model
    from lunar_drl_exploration.batch import pack_observations
    from lunar_drl_exploration.config import ModelConfig
    from test_model import observation
    torch.manual_seed(17)
    layer = model.SparseAttentionLayer(ModelConfig(width=16, heads=2, layers=1))
    # These keys are the existing checkpoint contract, including all 13 tensors.
    expected_keys = {'norm1.weight', 'norm1.bias', 'qkv.weight', 'qkv.bias',
                     'edge_bias.weight', 'project.weight', 'project.bias',
                     'norm2.weight', 'norm2.bias', 'ff.0.weight', 'ff.0.bias',
                     'ff.2.weight', 'ff.2.bias'}
    assert set(layer.state_dict()) == expected_keys
    graph = pack_observations([observation()], device='cpu').graph
    x = torch.randn(len(graph.features), 16, requires_grad=True)
    calls = []
    def record(weights, values, source, target, count):
        calls.append((values.is_contiguous(), count))
        return module.edge_message_cuda(weights, values, source, target, count)
    monkeypatch.setattr(model, 'edge_message_cuda', record)
    actual = layer(x, graph)
    actual_grad = torch.autograd.grad(actual.square().sum(), (x, *layer.parameters()))
    monkeypatch.setattr(model, 'edge_message_cuda', original_message)
    expected = layer(x, graph)
    expected_grad = torch.autograd.grad(expected.square().sum(), (x, *layer.parameters()))
    assert calls == [(False, len(x))]
    torch.testing.assert_close(actual, expected, rtol=0, atol=0)
    for actual_value, expected_value in zip(actual_grad, expected_grad):
        torch.testing.assert_close(actual_value, expected_value, rtol=0, atol=0)


@pytest.mark.skipif(not torch.cuda.is_available(), reason='CUDA unavailable')
def test_cuda_fusion_handles_dynamic_shapes_noncontiguous_values_and_gradients(monkeypatch):
    module = message_module()
    monkeypatch.setattr(module, '_disabled', False)
    for nodes, edges in [(23, 51), (41, 127), (7, 1), (7, 0)]:
        qkv, weights, values, source, target = inputs('cuda', nodes, edges)
        # Compiled autograd may donate saved buffers on a normal backward.
        # Exercise that production path with separate but identical input graphs,
        # rather than asking it to retain a graph shared with the reference.
        reference_qkv = qkv.detach().clone().requires_grad_()
        reference_values = reference_qkv.unbind(1)[2]
        reference_weights = weights.detach().clone().requires_grad_()
        assert not reference_values.is_contiguous()
        actual = module.edge_message_cuda(weights, values, source, target, nodes)
        expected = original_message(reference_weights, reference_values, source, target, nodes)
        torch.testing.assert_close(actual, expected, rtol=1e-4, atol=1e-5)
        actual_grad = torch.autograd.grad(actual.square().sum(), (qkv, weights))
        expected_grad = torch.autograd.grad(expected.square().sum(), (reference_qkv, reference_weights))
        for actual_value, expected_value in zip(actual_grad, expected_grad):
            torch.testing.assert_close(actual_value, expected_value, rtol=1e-4, atol=1e-5)
        assert not module._disabled, 'CUDA equivalence must exercise the compiled path'


@pytest.mark.skipif(not torch.cuda.is_available(), reason='CUDA unavailable')
def test_cuda_deterministic_mode_bypasses_compiler_without_disabling_later_fusion(monkeypatch):
    module = message_module()
    _, weights, values, source, target = inputs('cuda')
    monkeypatch.setattr(module, '_disabled', False)
    def forbidden_compile():
        pytest.fail('Deterministic mode must retain the original reduction')
    monkeypatch.setattr(module, '_compiled', forbidden_compile)
    previous = torch.are_deterministic_algorithms_enabled()
    warn_only = torch.is_deterministic_algorithms_warn_only_enabled()
    try:
        torch.use_deterministic_algorithms(True)
        actual = module.edge_message_cuda(weights, values, source, target, len(values))
        expected = original_message(weights, values, source, target, len(values))
        torch.testing.assert_close(actual, expected, rtol=0, atol=0)
        assert not module._disabled
    finally:
        torch.use_deterministic_algorithms(previous, warn_only=warn_only)


@pytest.mark.skipif(not torch.cuda.is_available(), reason='CUDA unavailable')
def test_compiler_compatibility_failure_falls_back_once(monkeypatch, caplog):
    module = message_module()
    from torch._dynamo.exc import Unsupported
    _, weights, values, source, target = inputs('cuda')
    calls = []
    def broken(*args):
        calls.append(1)
        raise Unsupported('test compiler unavailable')
    import logging
    logger=logging.getLogger(module.__name__)
    monkeypatch.setattr(logger,'handlers',[caplog.handler])
    monkeypatch.setattr(logger,'propagate',False)
    monkeypatch.setattr(module, '_disabled', False)
    monkeypatch.setattr(module, '_compiled', lambda: broken)
    expected = original_message(weights, values, source, target, len(values))
    for _ in range(2):
        actual = module.edge_message_cuda(weights, values, source, target, len(values))
        torch.testing.assert_close(actual, expected, rtol=1e-4, atol=1e-5)
    assert len(calls) == 1
    assert len([r for r in caplog.records if 'eager' in r.message]) == 1


@pytest.mark.skipif(not torch.cuda.is_available(), reason='CUDA unavailable')
@pytest.mark.parametrize('kind', ['oom', 'wrapped_oom', 'runtime', 'wrapped_runtime'])
def test_resource_and_runtime_errors_are_not_swallowed(monkeypatch, kind):
    module = message_module()
    from torch._dynamo.exc import BackendCompilerFailed
    _, weights, values, source, target = inputs('cuda')
    def broken(*args):
        if kind == 'oom':
            raise torch.OutOfMemoryError('test oom')
        if kind == 'wrapped_oom':
            raise BackendCompilerFailed(broken, torch.OutOfMemoryError('wrapped test oom'), None)
        if kind == 'wrapped_runtime':
            raise BackendCompilerFailed(broken, RuntimeError('CUDA error: illegal memory access'), None)
        raise RuntimeError('test CUDA execution failure')
    monkeypatch.setattr(module, '_disabled', False)
    monkeypatch.setattr(module, '_compiled', lambda: broken)
    expected = {'oom': torch.OutOfMemoryError, 'wrapped_oom': BackendCompilerFailed,
                'runtime': RuntimeError, 'wrapped_runtime': BackendCompilerFailed}[kind]
    with pytest.raises(expected):
        module.edge_message_cuda(weights, values, source, target, len(values))
    assert not module._disabled


@pytest.mark.parametrize('message', ['CUDA error: illegal memory access',
                                     'CUDA error: device-side assert triggered',
                                     'CUDA out of memory.',
                                     'CUDA driver error: invalid argument'])
def test_wrapped_runtime_device_failure_never_enters_eager_fallback_without_using_gpu(monkeypatch, message):
    module = message_module()
    from torch._dynamo.exc import BackendCompilerFailed
    # Only exercise error dispatch here: no tensor allocation or CUDA API call.
    # The compiler test double raises before it can consume any tensor inputs.
    values = SimpleNamespace(device=SimpleNamespace(type='cuda'))
    def broken(*args):
        raise BackendCompilerFailed(broken, RuntimeError(message), None)
    def forbidden_fallback(*args):
        pytest.fail('A wrapped CUDA execution failure must not enter eager fallback')
    monkeypatch.setattr(module, '_disabled', False)
    monkeypatch.setattr(module, '_compiled', lambda: broken)
    monkeypatch.setattr(module, 'edge_message_eager', forbidden_fallback)
    with pytest.raises(BackendCompilerFailed):
        module.edge_message_cuda(None, values, None, None, 0)
    assert not module._disabled
