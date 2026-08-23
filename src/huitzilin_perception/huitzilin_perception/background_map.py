"""
background_map.py — persistent voxel-hashed background model (W4).

Replaces the 5-frame rolling background deque that made the detector go blind
whenever the drone translated. Measured ("the
detector goes blind while patrolling"): the deque's oldest frame is only 4 cloud
frames = 0.267 s old, which cannot cover scene the camera newly *sees* as patrol
translates and yaws, so the foreground ramped 48 -> 3475 -> ... -> 45023 points,
the ball got connected to that flood at cluster_tolerance_m, and every on-target
patrol throw produced 0-1 centroids against a min_track_updates=3 gate.

The fix is coverage, not a threshold: keep every voxel the camera has *ever*
seen (within a TTL) in the fixed odom frame, so re-entering a region the drone
looked at ten seconds ago is background, not novelty.

Why a voxel hash and not a growing cKDTree: foreground_mask() rebuilds a tree
over the background every frame, and a persistent background is 10^5-10^6
points. Tree build would dominate the 66 ms frame budget. Voxel-key lookup is
a searchsorted, and memory is bounded by scene *volume* rather than frame count.

Distance semantics differ slightly from foreground_mask()'s metric threshold: a
point counts as background if its own voxel OR any of the 26 neighbours is
occupied, i.e. an effective tolerance of leaf..leaf*sqrt(3). Set leaf_m to
diff_threshold_m to keep the two comparable. The dilation is not optional
cleverness — without it a static surface point that jitters across a voxel
boundary reads as foreground every time it crosses.

No ROS imports: unit-testable anywhere (test/test_background_map.py).
"""

from __future__ import annotations

import numpy as np

# Voxel key packing
# 21 bits per axis, signed via a bias, packed into one int64 (63 bits used).
# Collision-free by construction — a hash with mixing would not be, and a false
# "already seen" silently deletes a projectile.
_BITS = 21
_BIAS = 1 << (_BITS - 1)          # 2^20 voxels ~= 105 km at a 0.10 m leaf
_AXIS_MAX = (1 << _BITS) - 1

_NEIGHBOURS = np.array(
    [(dx, dy, dz)
     for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
     if (dx, dy, dz) != (0, 0, 0)],
    dtype=np.int64,
)


def _pack(ijk: np.ndarray) -> np.ndarray:
    """(N, 3) int64 voxel indices -> (N,) int64 keys."""
    b = np.clip(ijk + _BIAS, 0, _AXIS_MAX)
    return (b[:, 0] << (2 * _BITS)) | (b[:, 1] << _BITS) | b[:, 2]


class VoxelBackgroundMap:
    """
    Occupancy set of voxels the sensor has observed, keyed in a fixed frame.

    Usage per cloud frame, in this order — insert AFTER querying, or the frame
    becomes its own background and nothing is ever foreground:

        fg_mask = bg.foreground(pts)
        bg.insert(pts, stamp_s)
        bg.prune(stamp_s)

    An empty map reports nothing as foreground (the first frame has no basis for
    novelty), which mirrors the old `len(buffer) < 2` early return.
    """

    def __init__(self, leaf_m: float, ttl_s: float,
                 max_voxels: int = 400_000, prune_every: int = 15) -> None:
        if leaf_m <= 0.0:
            raise ValueError(f"leaf_m must be > 0, got {leaf_m}")
        self.leaf_m = float(leaf_m)
        self.ttl_s = float(ttl_s)
        self.max_voxels = int(max_voxels)
        self.prune_every = max(1, int(prune_every))
        self._keys = np.empty(0, dtype=np.int64)     # sorted, unique
        self._seen = np.empty(0, dtype=np.float64)   # last-seen time, parallel
        self._inserts = 0
        self._last_t: float | None = None

    # introspection

    def __len__(self) -> int:
        return int(self._keys.size)

    # core

    def clear(self) -> None:
        self._keys = np.empty(0, dtype=np.int64)
        self._seen = np.empty(0, dtype=np.float64)
        self._last_t = None

    def _ijk(self, pts: np.ndarray) -> np.ndarray:
        return np.floor(np.asarray(pts, dtype=np.float64) / self.leaf_m).astype(np.int64)

    def _contains(self, keys: np.ndarray) -> np.ndarray:
        if self._keys.size == 0:
            return np.zeros(keys.shape[0], dtype=bool)
        idx = np.searchsorted(self._keys, keys)
        np.clip(idx, 0, self._keys.size - 1, out=idx)
        return self._keys[idx] == keys

    def foreground(self, pts: np.ndarray) -> np.ndarray:
        """
        Bool mask of points in no observed voxel and no neighbour of one.

        Two-stage on purpose: in a warm map almost every point hits its own
        voxel, so the 26-neighbour dilation only runs on the handful of misses.
        """
        pts = np.asarray(pts)
        if pts.shape[0] == 0:
            return np.zeros(0, dtype=bool)
        fg = np.zeros(pts.shape[0], dtype=bool)
        if self._keys.size == 0:
            return fg

        ijk = self._ijk(pts)
        miss_at = np.flatnonzero(~self._contains(_pack(ijk)))
        if miss_at.size == 0:
            return fg

        sub = ijk[miss_at]
        novel = np.ones(miss_at.size, dtype=bool)
        for off in _NEIGHBOURS:
            live = np.flatnonzero(novel)
            if live.size == 0:
                break
            hit = self._contains(_pack(sub[live] + off))
            novel[live[hit]] = False
        fg[miss_at[novel]] = True
        return fg

    def insert(self, pts: np.ndarray, stamp_s: float) -> None:
        """Mark every voxel containing a point as observed at `stamp_s`."""
        pts = np.asarray(pts)
        if pts.shape[0] == 0:
            return
        # Sim time running backwards means the stack restarted under us; a map
        # carried across that would be geometry from a previous flight.
        if self._last_t is not None and stamp_s < self._last_t:
            self.clear()
        self._last_t = float(stamp_s)
        self._inserts += 1

        keys = np.unique(_pack(self._ijk(pts)))
        if self._keys.size == 0:
            self._keys = keys
            self._seen = np.full(keys.size, float(stamp_s), dtype=np.float64)
            return

        idx = np.searchsorted(self._keys, keys)
        clipped = np.clip(idx, 0, self._keys.size - 1)
        known = self._keys[clipped] == keys
        self._seen[clipped[known]] = stamp_s
        fresh = ~known
        if fresh.any():
            # `keys` is sorted and `idx` indexes the pre-insert array, so
            # np.insert preserves the sort order the lookups depend on.
            self._keys = np.insert(self._keys, idx[fresh], keys[fresh])
            self._seen = np.insert(self._seen, idx[fresh],
                                   np.full(int(fresh.sum()), float(stamp_s)))

    def prune(self, stamp_s: float) -> None:
        """
        Drop voxels unseen for longer than ttl_s, then cap total size.

        Runs every `prune_every` inserts: pruning is an O(M) rebuild and the map
        only needs to stay bounded, not be exact.
        """
        if self._keys.size == 0 or self._inserts % self.prune_every:
            return
        keep = (float(stamp_s) - self._seen) <= self.ttl_s
        if not keep.all():
            self._keys = self._keys[keep]
            self._seen = self._seen[keep]
        if self._keys.size > self.max_voxels:
            # Keep the most recently seen; argpartition avoids a full sort.
            newest = np.argpartition(self._seen,
                                     self._keys.size - self.max_voxels
                                     )[-self.max_voxels:]
            newest.sort()
            self._keys = self._keys[newest]
            self._seen = self._seen[newest]
