"""Stage 5 - sequencing and routing (spec §12).

Orders the passes to minimize total non-productive (deadhead) return travel
between the end of one pass's fill segment and the start of the next pass's cut
segment. A nearest-neighbour constructive heuristic is followed by a bounded
2-opt improvement. Because passes are generated layer-by-layer (shallower
layers first), the resulting order respects layer ordering by construction.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

from .passes import Pass


@dataclass
class Route:
    order: List[int]              # indices into the original pass list
    ordered_passes: List[Pass]
    total_deadhead_m: float


def _deadhead(a: Pass, b: Pass) -> float:
    return math.dist(a.fill_end_xy, b.cut_start_xy)


def sequence_passes(passes: List[Pass], two_opt_cap: int = 1500) -> Route:
    n = len(passes)
    if n == 0:
        return Route(order=[], ordered_passes=[], total_deadhead_m=0.0)
    if n == 1:
        return Route(order=[0], ordered_passes=list(passes), total_deadhead_m=0.0)

    # Nearest-neighbour construction starting from pass 0 (deterministic).
    unvisited = set(range(n))
    order = [0]
    unvisited.remove(0)
    current = 0
    while unvisited:
        cur_pass = passes[current]
        nxt = min(unvisited, key=lambda k: _deadhead(cur_pass, passes[k]))
        order.append(nxt)
        unvisited.remove(nxt)
        current = nxt

    # Bounded 2-opt: only run on modest pass counts to stay fast + deterministic.
    if n <= two_opt_cap:
        order = _two_opt(passes, order)

    total = sum(_deadhead(passes[order[k]], passes[order[k + 1]]) for k in range(len(order) - 1))
    ordered = [passes[k] for k in order]
    return Route(order=order, ordered_passes=ordered, total_deadhead_m=total)


def _two_opt(passes: List[Pass], order: List[int], max_iter: int = 30) -> List[int]:
    n = len(order)
    improved = True
    it = 0
    while improved and it < max_iter:
        improved = False
        it += 1
        for i in range(n - 1):
            a_end = passes[order[i]].fill_end_xy
            b_start = passes[order[i + 1]].cut_start_xy
            d_ab = math.dist(a_end, b_start)
            for k in range(i + 2, n):
                c = passes[order[k]]
                d_next = (
                    math.dist(passes[order[k - 1]].fill_end_xy, c.cut_start_xy)
                )
                # cost of reversing segment order[i+1 .. k]
                new_d_a = math.dist(a_end, passes[order[k]].cut_start_xy)
                if k + 1 < n:
                    old_tail = math.dist(c.fill_end_xy, passes[order[k + 1]].cut_start_xy)
                    new_tail = math.dist(
                        passes[order[i + 1]].fill_end_xy, passes[order[k + 1]].cut_start_xy
                    )
                else:
                    old_tail = 0.0
                    new_tail = 0.0
                delta = (new_d_a + new_tail) - (d_ab + old_tail)
                if delta < -1e-6:
                    order[i + 1: k + 1] = reversed(order[i + 1: k + 1])
                    improved = True
                    break
            if improved:
                break
    return order
