"""Task-owned effective observation union K, stored on a fixed world lattice.

Consumes applied native observations only. Current B/M classifications never
come from this history. Public projections use the current native map origin.
"""
import numpy as np
from .geometry import lattice_coordinates, lattice_displacement, polygon_mask


class CoverageHistory:
    def __init__(self, task):
        self.task = task
        self.identity = self.coordinate_identity = None
        self.revision = -1
        self._tiles = {}; self._sources = {}; self._task_bits = {}
        self._known_count = 0
        self._bounds = None
        self._offset = np.zeros(2, np.int64)
        self._reference_origin = None
        self._reference_offset = None

    @property
    def cell_offset(self):
        """Current native origin in the retained lattice coordinate system."""
        return tuple(map(int, self._offset))

    @property
    def known_area_m2(self):
        return self._known_count*self.identity[1]**2 if self.identity is not None else 0.

    @property
    def bounds(self):
        if self._bounds is None:
            return None
        dx, dy = map(int, self._offset)
        x0, y0, x1, y1 = self._bounds
        return x0-dx, y0-dy, x1-dx, y1-dy

    def consume(self, snapshot):
        identity = (snapshot.epoch, snapshot.resolution_m, tuple(snapshot.origin))
        displacement = None
        if (self.coordinate_identity is not None and
                self.coordinate_identity[:2] == identity[:2]):
            displacement = lattice_displacement(snapshot.origin, self._reference_origin,
                                                snapshot.resolution_m)
        reset = displacement is None
        if reset:
            phase, offset = lattice_coordinates(snapshot.origin, snapshot.resolution_m)
            coordinate_identity = (snapshot.epoch, snapshot.resolution_m, phase)
        else:
            coordinate_identity = self.coordinate_identity
            phase = coordinate_identity[2]
            offset = self._reference_offset + displacement
        if not reset and snapshot.revision < self.revision:
            raise ValueError('stale coverage snapshot')
        moved = self.identity != identity
        if reset:
            self._reference_origin = tuple(snapshot.origin)
            self._reference_offset = offset.copy()
            self._tiles.clear(); self._task_bits.clear()
            self._known_count = 0; self._bounds = None; self.revision = -1
        if moved:
            self._sources.clear()
        self.identity = identity
        self.coordinate_identity = coordinate_identity
        self._offset = offset
        if snapshot.revision == self.revision and not moved:
            return
        self.revision = snapshot.revision
        for key, tile in snapshot.tiles.items():
            if self._sources.get(key) is tile:
                continue
            self._sources[key] = tile
            incoming = (tile.observed != 0).reshape(256, 256)
            if not np.any(incoming):
                continue
            # A native tile can overlap up to four fixed-lattice storage tiles.
            x0, y0 = np.asarray(key)*256 + offset
            for ty in range(int(y0//256), int((y0+255)//256)+1):
                for tx in range(int(x0//256), int((x0+255)//256)+1):
                    left, right = max(x0, tx*256), min(x0+256, (tx+1)*256)
                    top, bottom = max(y0, ty*256), min(y0+256, (ty+1)*256)
                    block = np.zeros((256, 256), bool)
                    block[top-ty*256:bottom-ty*256, left-tx*256:right-tx*256] = (
                        incoming[top-y0:bottom-y0, left-x0:right-x0])
                    self._merge((tx, ty), block, phase, snapshot.resolution_m)

    def _merge(self, key, block, phase, resolution):
        incoming = np.packbits(block, bitorder='little')
        previous = self._tiles.get(key)
        new = incoming if previous is None else incoming & ~previous
        if not np.any(new):
            return
        self._tiles[key] = incoming.copy() if previous is None else previous | incoming
        if key not in self._task_bits:
            origin = (phase[0]+key[0]*256*resolution, phase[1]+key[1]*256*resolution)
            self._task_bits[key] = np.packbits(
                polygon_mask((256, 256), origin, resolution, self.task.polygon),
                bitorder='little')
        self._known_count += int(np.unpackbits(new & self._task_bits[key]).sum())
        ids = np.flatnonzero(np.unpackbits(new, bitorder='little'))
        xs = key[0]*256+ids%256; ys = key[1]*256+ids//256
        box = (int(xs.min()), int(ys.min()), int(xs.max())+1, int(ys.max())+1)
        self._bounds = box if self._bounds is None else (
            min(box[0], self._bounds[0]), min(box[1], self._bounds[1]),
            max(box[2], self._bounds[2]), max(box[3], self._bounds[3]))

    def mask(self, bounds):
        """Project fixed history into bounds relative to the current native origin."""
        x0, y0, x1, y1 = map(int, bounds)
        if x1 < x0 or y1 < y0:
            raise ValueError('inverted projection bounds')
        result = np.zeros((y1-y0, x1-x0), bool)
        dx, dy = map(int, self._offset)
        x0 += dx; x1 += dx; y0 += dy; y1 += dy
        for (tx, ty), bits in self._tiles.items():
            left, right = max(x0, tx*256), min(x1, (tx+1)*256)
            top, bottom = max(y0, ty*256), min(y1, (ty+1)*256)
            if right <= left or bottom <= top:
                continue
            tile = np.unpackbits(bits, bitorder='little').reshape(256, 256)
            result[top-y0:bottom-y0, left-x0:right-x0] = (
                tile[top-ty*256:bottom-ty*256, left-tx*256:right-tx*256])
        return result
