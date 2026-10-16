"""Fuse the existing sparse message reduction without changing graph or weights."""
import logging
from functools import lru_cache

import torch


_disabled = False


def edge_message_eager(weights, values, source, target, count):
    messages = weights[..., None] * values[source]
    output = messages.new_zeros((count,) + messages.shape[1:])
    return output.index_add_(0, target, messages)


@lru_cache(maxsize=1)
def _compiled():
    return torch.compile(edge_message_eager, fullgraph=True, dynamic=True,
                         options={'compile_threads': 1})


def edge_message_cuda(weights, values, source, target, count):
    global _disabled
    # Keep the original PyTorch deterministic reduction when explicitly enabled.
    if (values.device.type != 'cuda' or _disabled or
            torch.are_deterministic_algorithms_enabled()):
        return edge_message_eager(weights, values, source, target, count)
    try:
        return _compiled()(weights, values, source, target, count)
    except Exception as error:
        # Compatibility failures may fall back once; GPU/runtime failures remain
        # visible to the normal training stop/recovery path.
        from torch._dynamo.exc import (BackendCompilerFailed, Unsupported,
                                      FailOnRecompileLimitHit, RecompileLimitExceeded)
        if not isinstance(error, (BackendCompilerFailed, Unsupported,
                                  FailOnRecompileLimitHit, RecompileLimitExceeded)):
            raise
        if isinstance(error, BackendCompilerFailed):
            inner = error.inner_exception
            device_failures = ('cuda error:', 'cuda out of memory', 'cuda driver error:',
                               'device-side assert', 'illegal memory access',
                               'cublas_status_', 'cudnn_status_execution_failed')
            if (isinstance(inner, torch.OutOfMemoryError) or
                    any(marker in str(inner).lower() for marker in device_failures)):
                raise
        _disabled = True
        logging.getLogger(__name__).warning(
            'fused edge message compiler failed; using eager for this process: %s', error)
        return edge_message_eager(weights, values, source, target, count)
