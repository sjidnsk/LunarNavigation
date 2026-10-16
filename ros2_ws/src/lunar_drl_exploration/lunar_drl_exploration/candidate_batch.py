"""Pack immutable v5 inputs without reconstructing geometry or action features."""
from dataclasses import dataclass
import numpy as np
import torch
from .batch import GraphBatch, _graph, _tensor


@dataclass
class CandidateBatch:
    graph: GraphBatch
    real_nodes: torch.Tensor
    node_views: torch.Tensor
    heading_features: torch.Tensor
    polygon_edges: torch.Tensor
    polygon_groups: torch.Tensor
    context: torch.Tensor
    current_real: torch.Tensor
    candidate_nodes: torch.Tensor
    candidate_real: torch.Tensor
    action_candidates: torch.Tensor
    action_nodes: torch.Tensor
    action_views: torch.Tensor
    action_groups: torch.Tensor
    action_counts: tuple
    schema: str = 'task_graph_v5'

    @property
    def size(self):
        return len(self.action_counts)


def pack_candidates(observations, device):
    graph = _graph([o.node_geometry for o in observations], [o.edges for o in observations],
        [o.edge_lengths for o in observations], [o.current_index for o in observations], device)
    offsets = np.cumsum((0,) + graph.counts[:-1])
    real = np.flatnonzero(np.concatenate([o.node_is_real for o in observations]))
    real_index = np.full(sum(graph.counts), -1, np.int64)
    real_index[real] = np.arange(len(real))
    action_nodes = np.concatenate([o.action_nodes+offset for o, offset in zip(observations, offsets)])
    candidates, inverse = np.unique(action_nodes, return_inverse=True)
    action_headings = np.concatenate([o.action_pairs[:, 1] for o in observations])
    counts = tuple(len(o.goals) for o in observations)
    polygons = [np.concatenate((o.polygon, np.roll(o.polygon, -1, axis=0)), axis=1) for o in observations]
    def index(value):
        return torch.as_tensor(value, dtype=torch.long, device=device)
    return CandidateBatch(graph, index(real),
        _tensor([o.node_views[o.node_is_real] for o in observations], device),
        torch.as_tensor(np.stack([o.heading_features for o in observations]), device=device),
        _tensor(polygons, device), index(np.repeat(np.arange(len(observations)), [len(p) for p in polygons])),
        torch.as_tensor(np.stack([o.context for o in observations]), device=device),
        index(real_index[offsets+np.array([o.current_index for o in observations])]),
        index(candidates), index(real_index[candidates]), index(inverse), index(action_nodes),
        index(real_index[action_nodes]*8+action_headings),
        index(np.repeat(np.arange(len(observations)), counts)), counts)
