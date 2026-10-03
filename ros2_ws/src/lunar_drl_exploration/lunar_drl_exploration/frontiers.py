"""Frozen measured FREE-side frontiers and shared, classification-safe removal."""
from dataclasses import dataclass
import hashlib
from typing import TYPE_CHECKING
import numpy as np
from .contracts import MapSnapshot, TaskSpec, freeze_array
from .geometry import lattice_coordinates, lattice_displacement, pack_cell_ids
from .maps import FREE

if TYPE_CHECKING:
    from .task_analysis import MeasuredWorkspace


@dataclass(frozen=True)
class FrontierSet:
    cell_ids: np.ndarray
    centers: np.ndarray
    demand_offsets: np.ndarray
    demand_ids: np.ndarray

    def __post_init__(self):
        for name, dtype in (("cell_ids", np.int64), ("centers", np.float64),
                            ("demand_offsets", np.int64), ("demand_ids", np.int64)):
            object.__setattr__(self, name, freeze_array(getattr(self, name), dtype=dtype))
        n = len(self.cell_ids)
        if (self.cell_ids.shape != (n,) or self.centers.shape != (n, 2)
                or not np.all(np.isfinite(self.centers))
                or self.demand_ids.ndim != 1 or self.demand_offsets.shape != (n+1,)
                or self.demand_offsets[0] != 0
                or self.demand_offsets[-1] != len(self.demand_ids)
                or np.any(np.diff(self.demand_offsets) <= 0)
                or len(np.unique(self.cell_ids)) != n):
            raise ValueError("frontiers require unique centers and nonempty CSR demands")


@dataclass(frozen=True)
class FrontierSnapshot:
    epoch: str
    resolution_m: float
    lattice_origin: tuple
    task_id: str
    polygon_digest: str
    revision: int
    inside: FrontierSet
    outside: FrontierSet
    known_ids: np.ndarray
    free_ids: np.ndarray

    def __post_init__(self):
        object.__setattr__(self, "lattice_origin", tuple(self.lattice_origin))
        for name in ("known_ids", "free_ids"):
            object.__setattr__(self, name, freeze_array(
                np.unique(getattr(self, name)), dtype=np.int64))


def build_frontiers(snapshot: MapSnapshot, workspace: 'MeasuredWorkspace',
                    task: TaskSpec) -> FrontierSnapshot:
    """Use cumulative effective K and current measured B, over the full workspace."""
    w = workspace
    if w.lattice_origin is None:
        phase, offset = lattice_coordinates(snapshot.origin, snapshot.resolution_m)
    else:
        phase, offset = w.lattice_origin, np.asarray(w.lattice_offset, np.int64)
    shift = np.asarray(w.bounds[:2], np.int64) + offset
    ys, xs = np.nonzero(w.known & (w.intrinsic == FREE))
    free_ids = pack_cell_ids(xs+shift[0], ys+shift[1])
    ky, kx = np.nonzero(w.known)
    known_ids = pack_cell_ids(kx+shift[0], ky+shift[1])

    def domain_frontiers(domain):
        selected = w.task_mask[ys, xs] == domain
        fy, fx = ys[selected], xs[selected]
        rows, demand_x, demand_y = [], [], []
        height, width = w.known.shape
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == dy == 0:
                    continue
                nx, ny = fx+dx, fy+dy
                valid = (nx >= 0) & (ny >= 0) & (nx < width) & (ny < height)
                ids = np.flatnonzero(valid)
                ids = ids[(~w.known[ny[ids], nx[ids]]) &
                          (w.task_mask[ny[ids], nx[ids]] == domain)]
                rows.append(ids); demand_x.append(nx[ids]); demand_y.append(ny[ids])
        rows = np.concatenate(rows)
        dx = np.concatenate(demand_x); dy = np.concatenate(demand_y)
        counts = np.bincount(rows, minlength=len(fx))
        fronts = counts > 0
        order = np.argsort(rows, kind='stable')
        centers = np.column_stack((
            w.origin[0]+(fx[fronts]+.5)*snapshot.resolution_m,
            w.origin[1]+(fy[fronts]+.5)*snapshot.resolution_m))
        return FrontierSet(
            pack_cell_ids(fx[fronts]+shift[0], fy[fronts]+shift[1]), centers,
            np.r_[0, np.cumsum(counts[fronts])],
            pack_cell_ids(dx[order]+shift[0], dy[order]+shift[1]))

    digest = hashlib.sha256(np.asarray(task.polygon, dtype='<f4').tobytes()).hexdigest()
    return FrontierSnapshot(snapshot.epoch, snapshot.resolution_m, phase,
                            task.task_id, digest, snapshot.revision,
                            domain_frontiers(True), domain_frontiers(False),
                            known_ids, free_ids)


def count_removed(before: FrontierSnapshot, after: FrontierSnapshot, *, domain: str) -> int:
    """Count solved old fronts, independent of newly created front counts.

    Stateless: identical revisions contribute zero; callers advance their frozen
    before snapshot after an action rather than replaying the same revision pair.
    """
    if domain not in ('inside', 'outside'):
        raise ValueError("frontier domain must be inside or outside")
    for field in ('epoch', 'resolution_m', 'task_id', 'polygon_digest'):
        if getattr(before, field) != getattr(after, field):
            raise ValueError(f"incompatible frontier {field}")
    shift = lattice_displacement(after.lattice_origin, before.lattice_origin,
                                 before.resolution_m)
    if shift is None:
        raise ValueError("incompatible frontier lattice_origin")
    if after.revision < before.revision:
        raise ValueError("stale frontier revision")
    if after.revision == before.revision:
        return 0
    old, new = getattr(before, domain), getattr(after, domain)
    if not len(old.cell_ids):
        return 0
    def aligned(ids):
        # Independent initial phases may straddle 0/resolution because of
        # modulo noise. Rebase signed XY IDs as well as accepting the phase.
        if not np.any(shift):
            return ids
        return pack_cell_ids((ids >> 32)+shift[0], ((ids << 32) >> 32)+shift[1])

    solved = np.logical_and.reduceat(np.isin(old.demand_ids, aligned(after.known_ids)),
                                     old.demand_offsets[:-1])
    removed = (~np.isin(old.cell_ids, aligned(new.cell_ids)) &
               np.isin(old.cell_ids, aligned(after.free_ids)) & solved)
    return int(np.count_nonzero(removed))
