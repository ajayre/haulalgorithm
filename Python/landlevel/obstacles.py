"""Stage 5b - exclusion zones and obstacle-aware routing (spec §12.3).

An **exclusion zone** is an irregular polygon (e.g. a well head) the machine must
neither level nor drive across. Two things follow from one zone:

1. **No leveling.** The bins whose group bounding box contains the polygon are
   dropped from the field mask (:func:`landlevel.grid.apply_exclusions`), so no
   cut/fill is ever planned inside the zone even if the AGD has bins there.
2. **No driving through it.** Every blade-up travel leg (deadhead RETURN, loaded
   HAUL, multi-drop MOVE) must detour around the zone. This module provides the
   geometry: each zone is buffered outward by the machine's clearance (half the
   equipment width) and a **visibility graph** over the buffered vertices is
   solved with **Dijkstra** to give the shortest path that avoids every zone.

The straight-line case (the overwhelming majority of legs, because a well is tiny
relative to the field) is rejected in O(1) by an axis-aligned bounding-box test,
so :class:`ObstacleField` is cheap to call inside the O(n^2) sequencer. Only legs
that actually graze a zone bbox run the precise test, and only legs that are
genuinely blocked solve Dijkstra.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

XY = Tuple[float, float]


# ---------------------------------------------------------------------------
# Polygon helpers
# ---------------------------------------------------------------------------


def _signed_area(ring: Sequence[XY]) -> float:
    a = 0.0
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return 0.5 * a


def _as_ccw(ring: Sequence[XY]) -> List[XY]:
    """Return the ring ordered counter-clockwise (positive signed area)."""
    r = list(ring)
    if len(r) >= 3 and _signed_area(r) < 0.0:
        r = list(reversed(r))
    return r


def point_in_ring(x: float, y: float, ring: Sequence[XY]) -> bool:
    """Strict-ish point-in-polygon (ray cast); boundary is treated as outside."""
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if (yi > y) != (yj > y):
            x_at = (xj - xi) * (y - yi) / ((yj - yi) + 1e-18) + xi
            if x < x_at:
                inside = not inside
        j = i
    return inside


def _miter_buffer(ring: List[XY], clearance: float) -> List[XY]:
    """Offset a convex CCW ring outward by ``clearance`` (miter joins).

    For each edge the outward normal (to the right of the CCW direction) is shifted
    out by ``clearance``; each vertex moves along the average of its two adjacent
    edge normals, scaled so the perpendicular clearance to every edge is exactly
    ``clearance``. Convex polygons (a well is convex) give a clean larger ring;
    the miter scale is clamped to avoid spikes at very sharp corners.
    """
    n = len(ring)
    if n < 3 or clearance <= 0.0:
        return list(ring)

    # Outward edge normals (right of the CCW edge direction).
    normals: List[XY] = []
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        ln = math.hypot(ex, ey) or 1e-18
        normals.append((ey / ln, -ex / ln))

    out: List[XY] = []
    for i in range(n):
        n_prev = normals[(i - 1) % n]
        n_cur = normals[i]
        mx, my = n_prev[0] + n_cur[0], n_prev[1] + n_cur[1]
        mlen = math.hypot(mx, my)
        if mlen < 1e-9:
            mx, my, mlen = n_cur[0], n_cur[1], 1.0
        mx, my = mx / mlen, my / mlen
        # cos(theta/2) = miter . edge-normal; scale = clearance / cos(theta/2).
        cos_half = mx * n_cur[0] + my * n_cur[1]
        scale = clearance / max(cos_half, 0.35)  # clamp sharp corners
        px, py = ring[i]
        out.append((px + mx * scale, py + my * scale))
    return out


def _seg_intersects_aabb(a: XY, b: XY, box: Tuple[float, float, float, float]) -> bool:
    """Cheap reject: does segment a-b intersect the axis-aligned box (minx,miny,maxx,maxy)?"""
    minx, miny, maxx, maxy = box
    ax, ay = a
    bx, by = b
    # Trivial reject: both endpoints strictly on one side of a slab.
    if (ax < minx and bx < minx) or (ax > maxx and bx > maxx):
        return False
    if (ay < miny and by < miny) or (ay > maxy and by > maxy):
        return False
    return True


# ---------------------------------------------------------------------------
# Exclusion zone
# ---------------------------------------------------------------------------


@dataclass
class ExclusionZone:
    """One no-go polygon (projected metres) with a clearance-buffered ring."""

    ring: List[XY]                 # original polygon (CCW, projected xy)
    buffer_ring: List[XY]          # ring offset outward by the clearance
    bbox: Tuple[float, float, float, float]  # buffered-ring AABB (minx,miny,maxx,maxy)

    @classmethod
    def from_ring(cls, ring: Sequence[XY], clearance: float) -> "ExclusionZone":
        ccw = _as_ccw(ring)
        buf = _miter_buffer(ccw, clearance)
        xs = [p[0] for p in buf]
        ys = [p[1] for p in buf]
        return cls(ring=ccw, buffer_ring=buf, bbox=(min(xs), min(ys), max(xs), max(ys)))

    def blocks(self, a: XY, b: XY, samples: int = 24) -> bool:
        """Does straight segment a-b pass through the buffered interior?

        Cheap bbox reject first; then sample interior points of the segment and
        test strict containment. Endpoints are excluded so a leg that merely
        starts/ends on the boundary (a perimeter edge of the visibility graph) is
        not considered blocked.
        """
        if not _seg_intersects_aabb(a, b, self.bbox):
            return False
        ax, ay = a
        bx, by = b
        for k in range(1, samples):
            t = k / samples
            if point_in_ring(ax + (bx - ax) * t, ay + (by - ay) * t, self.buffer_ring):
                return True
        return False


# ---------------------------------------------------------------------------
# Obstacle field: visibility graph + Dijkstra
# ---------------------------------------------------------------------------


@dataclass
class ObstacleField:
    """A set of exclusion zones plus a visibility graph for routing around them.

    All public queries are no-ops (straight-line / clear) when there are no zones,
    so the field can be threaded everywhere unconditionally.
    """

    zones: List[ExclusionZone] = field(default_factory=list)
    _nodes: List[XY] = field(default_factory=list)
    _adj: List[List[Tuple[int, float]]] = field(default_factory=list)

    @classmethod
    def from_rings(cls, rings: Sequence[Sequence[XY]], clearance: float) -> "ObstacleField":
        zones = [ExclusionZone.from_ring(r, clearance) for r in rings if len(r) >= 3]
        of = cls(zones=zones)
        of._build_graph()
        return of

    @classmethod
    def from_zones(cls, zones: Sequence["ExclusionZone"]) -> "ObstacleField":
        """Build a field from already-buffered zones (e.g. mixing clearances)."""
        of = cls(zones=list(zones))
        of._build_graph()
        return of

    @property
    def active(self) -> bool:
        return bool(self.zones)

    def _build_graph(self) -> None:
        """Visibility graph over all buffered-ring vertices."""
        self._nodes = [v for z in self.zones for v in z.buffer_ring]
        m = len(self._nodes)
        self._adj = [[] for _ in range(m)]
        for i in range(m):
            for j in range(i + 1, m):
                if self._visible(self._nodes[i], self._nodes[j]):
                    d = math.dist(self._nodes[i], self._nodes[j])
                    self._adj[i].append((j, d))
                    self._adj[j].append((i, d))

    def _visible(self, a: XY, b: XY) -> bool:
        """True if straight a-b crosses no zone interior (perimeter edges allowed)."""
        for z in self.zones:
            if z.blocks(a, b):
                return False
        return True

    def blocked(self, a: XY, b: XY) -> bool:
        for z in self.zones:
            if z.blocks(a, b):
                return True
        return False

    def path(self, a: XY, b: XY) -> List[XY]:
        """Shortest polyline from a to b avoiding every zone (inclusive of a, b)."""
        if not self.zones or not self.blocked(a, b):
            return [a, b]

        # Temporary graph: buffered vertices + a + b. Connect a/b to visible nodes.
        nodes = self._nodes
        m = len(nodes)
        ai, bi = m, m + 1
        adj: List[List[Tuple[int, float]]] = [list(e) for e in self._adj]
        adj.append([])  # a
        adj.append([])  # b
        for idx, pt in ((ai, a), (bi, b)):
            for k in range(m):
                if self._visible(pt, nodes[k]):
                    d = math.dist(pt, nodes[k])
                    adj[idx].append((k, d))
                    adj[k].append((idx, d))
        if self._visible(a, b):  # rare: bbox hit but not truly blocked
            adj[ai].append((bi, math.dist(a, b)))
            adj[bi].append((ai, math.dist(a, b)))

        prev, dist = self._dijkstra(adj, ai, bi)
        if dist[bi] == math.inf:
            return [a, b]  # unreachable (degenerate); fall back to straight
        chain = [bi]
        while chain[-1] != ai:
            p = prev[chain[-1]]
            if p < 0:
                return [a, b]
            chain.append(p)
        chain.reverse()
        all_pts = nodes + [a, b]
        return [all_pts[i] for i in chain]

    def path_length(self, a: XY, b: XY) -> float:
        """Length of the obstacle-avoiding shortest path from a to b."""
        if not self.zones or not self.blocked(a, b):
            return math.dist(a, b)
        pts = self.path(a, b)
        return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))

    @staticmethod
    def _dijkstra(adj: List[List[Tuple[int, float]]], src: int, dst: int):
        n = len(adj)
        dist = [math.inf] * n
        prev = [-1] * n
        dist[src] = 0.0
        pq: List[Tuple[float, int]] = [(0.0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            if u == dst:
                break
            for v, w in adj[u]:
                nd = d + w
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        return prev, dist


# A shared empty field so callers can default to "no obstacles" cheaply.
EMPTY_OBSTACLES = ObstacleField()
