"""Six-layer candidate network with geometry bypass and indexed score reuse."""
from dataclasses import dataclass
import torch
from torch import nn
from torch.nn import functional as F
from torch.utils.checkpoint import checkpoint
from .model import SparseAttentionLayer, CurrentRead, segment_sum


@dataclass
class CandidateEncoding:
    positions: torch.Tensor
    views: torch.Tensor
    nodes: torch.Tensor
    context: torch.Tensor
    current: torch.Tensor

    def score_parts(self, batch):
        return (self.positions[batch.candidate_real], self.nodes[batch.candidate_nodes],
                self.views.flatten(0, 1)[batch.action_views], self.context)


def actor_score_indices(batch):
    return (batch.action_candidates, batch.action_candidates, None, batch.action_groups)


class SymmetricPolygonEncoder(nn.Module):
    def __init__(self, width):
        super().__init__()
        self.edge = nn.Sequential(nn.Linear(4, width), nn.GELU(), nn.Linear(width, width))

    def forward(self, edges, groups, count):
        reversed_edges = torch.cat((edges[:, 2:], edges[:, :2]), dim=-1)
        encoded = (self.edge(edges)+self.edge(reversed_edges))*.5
        totals = segment_sum(encoded, groups, count)
        sizes = segment_sum(edges.new_ones((len(edges), 1)), groups, count)
        return totals/sizes.clamp_min(1)


class CandidateMeasuredEncoder(nn.Module):
    def __init__(self, config):
        super().__init__()
        width = config.width
        self.position = nn.Sequential(nn.Linear(5, width), nn.GELU(), nn.Linear(width, width))
        self.view = nn.Sequential(nn.Linear(5, width), nn.GELU(), nn.Linear(width, width))
        self.node_fusion = nn.Sequential(nn.Linear(width*2, width), nn.GELU())
        self.virtual = nn.Parameter(torch.zeros(width))
        self.layers = nn.ModuleList([SparseAttentionLayer(config) for _ in range(config.layers)])
        self.norm = nn.LayerNorm(width)
        self.read = CurrentRead(config)
        self.polygon = SymmetricPolygonEncoder(width)
        # Four dynamic scalars enter fusion directly, with no separate expansion.
        self.context = nn.Sequential(nn.Linear(width*4+4, width), nn.GELU())

    def forward(self, batch):
        if batch.schema != 'task_graph_v5':
            raise ValueError('candidate encoder requires task_graph_v5 schema')
        real = batch.real_nodes
        positions = self.position(batch.graph.features[real])
        labels = batch.heading_features[batch.graph.groups[real]]
        views = self.view(torch.cat((batch.node_views, labels), dim=-1).flatten(0, 1))
        views = views.reshape(len(real), 8, -1)
        features = self.node_fusion(torch.cat((positions, views.mean(dim=1)), dim=-1))
        nodes = self.virtual.expand(len(batch.graph.features), -1).index_copy(0, real, features)
        for layer in self.layers:
            if self.training and torch.is_grad_enabled():
                nodes = checkpoint(layer, nodes, batch.graph, use_reentrant=False, preserve_rng_state=False)
            else:
                nodes = layer(nodes, batch.graph)
        nodes = self.norm(nodes)
        current = nodes[batch.graph.current]
        context = self.context(torch.cat((positions[batch.current_real], current,
            self.read(current, nodes[real], batch.graph.groups[real]),
            self.polygon(batch.polygon_edges, batch.polygon_groups, batch.size), batch.context), dim=-1))
        return CandidateEncoding(positions, views, nodes, context, current)


class FactoredScoreHead(nn.Module):
    """The same concat-Linear-GELU-Linear, with reusable first-layer projections.

    Bias is added once, and no per-part nonlinearity is introduced. Indices map
    each position/state/view projection to the same frozen action ordering.
    """
    def __init__(self, widths, hidden):
        super().__init__()
        self.widths = tuple(widths)
        self.first = nn.Linear(sum(widths), hidden)
        self.output = nn.Linear(hidden, 1)

    def forward(self, parts, indices):
        if len(parts) != len(self.widths) or len(indices) != len(parts):
            raise ValueError('score parts and indices must align')
        value, offset = None, 0
        for part, index, width in zip(parts, indices, self.widths):
            projection = F.linear(part, self.first.weight[:, offset:offset+width])
            projection = projection if index is None else projection[index]
            value = projection if value is None else value+projection
            offset += width
        return self.output(F.gelu(value+self.first.bias)).squeeze(-1)
