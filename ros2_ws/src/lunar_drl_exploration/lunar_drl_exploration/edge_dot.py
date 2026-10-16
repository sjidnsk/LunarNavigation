"""Fuse the existing sparse edge dot product without changing graph or weights."""
import math
import logging
from functools import lru_cache
import torch

_disabled=False


def edge_dot_eager(q,k,source,target):
    return (q[target]*k[source]).sum(-1)/math.sqrt(q.shape[-1])


@lru_cache(maxsize=1)
def _compiled():
    return torch.compile(edge_dot_eager,fullgraph=True,dynamic=True,
                         options={'compile_threads':1})


def edge_dot_cuda(q,k,source,target):
    global _disabled
    if q.device.type!='cuda' or _disabled:return edge_dot_eager(q,k,source,target)
    try:
        return _compiled()(q,k,source,target)
    except Exception as error:
        # Only compiler compatibility failures may fall back. CUDA/resource
        # errors and invalid input remain visible to the normal stop/recovery.
        from torch._dynamo.exc import (BackendCompilerFailed,Unsupported,
                                      FailOnRecompileLimitHit,RecompileLimitExceeded)
        if not isinstance(error,(BackendCompilerFailed,Unsupported,FailOnRecompileLimitHit,RecompileLimitExceeded)):
            raise
        if isinstance(error,BackendCompilerFailed) and isinstance(error.inner_exception,torch.OutOfMemoryError):
            raise
        _disabled=True
        logging.getLogger(__name__).warning('fused edge dot compiler failed; using eager for this process: %s',error)
        return edge_dot_eager(q,k,source,target)
