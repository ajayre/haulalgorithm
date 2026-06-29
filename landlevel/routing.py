"""Stage 5 - sequencing and routing (spec §12).

Orders the passes to minimize total non-productive travel between passes. The
cost of going from one pass to the next is *direction-aware*: it combines the
deadhead drive time with the time lost turning to leave the current pass and
swing into the next cut. Both terms are in seconds, using the same turning-arc
model as the guidance time estimate (machine slows to ``turn_speed`` and travels
``R * angle``), so they trade off without an arbitrary weight. A nearest-
neighbour constructive heuristic builds the route; a bounded 2-opt improvement
refines it (distance-only, and only when direction-aware sequencing is off, to
avoid the asymmetry that segment reversal introduces with directed turn costs).
Because passes are generated layer-by-layer (shallower layers first), the
resulting order respects layer ordering by construction.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from . import (
    DEFAULT_RETURN_SPEED_MPH,
    DEFAULT_TURNING_CIRCLE_FT,
    DEFAULT_TURN_SPEED_MPH,
    FEET_TO_METERS,
    MPH_TO_MPS,
)
from .passes import Pass


@dataclass
class Route:
    order: List[int]              # indices into the original pass list
    ordered_passes: List[Pass]
    total_deadhead_m: float


def _exit_xy(p: Pass, grade_return: bool):
    """Where the machine actually is when a pass finishes.

    With the grade/compact return pass on, the machine drives back over the fill
    and ends at ``fill_start`` (the side nearest the cut ridge), so the next cut
    should be chained from there — that way the grade return doubles as travel
    toward the next cut instead of being a detour. Without it, the pass ends at
    ``fill_end``.
    """
    return p.fill_start_xy if grade_return else p.fill_end_xy


def _deadhead(a: Pass, b: Pass, grade_return: bool = True) -> float:
    return math.dist(_exit_xy(a, grade_return), b.cut_start_xy)


def _unit(p0: Tuple[float, float], p1: Tuple[float, float]) -> Optional[Tuple[float, float]]:
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return None
    return (dx / n, dy / n)


def _cut_dir(p: Pass) -> Optional[Tuple[float, float]]:
    """Heading the machine drives while cutting (cut_start -> cut_end)."""
    return _unit(p.cut_start_xy, p.cut_end_xy)


def _exit_dir(p: Pass, grade_return: bool) -> Optional[Tuple[float, float]]:
    """Heading the machine is travelling when the pass finishes.

    With the grade/compact return pass on, the last motion is the return over the
    fill (fill_end -> fill_start). Without it, the machine ends heading along the
    spread direction (fill_start -> fill_end).
    """
    if grade_return:
        return _unit(p.fill_end_xy, p.fill_start_xy)
    return _unit(p.fill_start_xy, p.fill_end_xy)


def _turn_angle(u: Optional[Tuple[float, float]],
                v: Optional[Tuple[float, float]]) -> float:
    if u is None or v is None:
        return 0.0
    d = max(-1.0, min(1.0, u[0] * v[0] + u[1] * v[1]))
    return math.acos(d)


@dataclass
class _TransitionModel:
    """Time-based cost of going from one pass to the next.

    ``return_v`` covers the deadhead drive; ``turn_v`` and ``R`` (turning-circle
    radius) convert heading changes into arc travel time, matching the guidance
    time estimate. When disabled, the cost is plain deadhead distance.
    """
    grade_return: bool
    direction_aware: bool
    return_v: float
    turn_v: float
    R: float

    def cost(self, a: Pass, b: Pass) -> float:
        exit_pt = _exit_xy(a, self.grade_return)
        cut_start = b.cut_start_xy
        dist = math.dist(exit_pt, cut_start)
        if not self.direction_aware:
            return dist
        drive_t = dist / self.return_v if self.return_v > 0 else dist
        if dist < 1e-6 or self.turn_v <= 0:
            return drive_t
        dead_dir = _unit(exit_pt, cut_start)
        # Turn to leave the current heading onto the deadhead, then to swing from
        # the deadhead into the next cut.
        angle = (_turn_angle(_exit_dir(a, self.grade_return), dead_dir)
                 + _turn_angle(dead_dir, _cut_dir(b)))
        turn_t = self.R * angle / self.turn_v
        return drive_t + turn_t


def sequence_passes(passes: List[Pass], two_opt_cap: int = 1500,
                    grade_return: bool = True, direction_aware: bool = True,
                    return_speed_mph: float = DEFAULT_RETURN_SPEED_MPH,
                    turn_speed_mph: float = DEFAULT_TURN_SPEED_MPH,
                    turning_circle_ft: float = DEFAULT_TURNING_CIRCLE_FT) -> Route:
    n = len(passes)
    if n == 0:
        return Route(order=[], ordered_passes=[], total_deadhead_m=0.0)
    if n == 1:
        return Route(order=[0], ordered_passes=list(passes), total_deadhead_m=0.0)

    model = _TransitionModel(
        grade_return=grade_return,
        direction_aware=direction_aware,
        return_v=return_speed_mph * MPH_TO_MPS,
        turn_v=turn_speed_mph * MPH_TO_MPS,
        R=(turning_circle_ft * FEET_TO_METERS) / 2.0,
    )

    # Nearest-neighbour construction starting from pass 0 (deterministic).
    unvisited = set(range(n))
    order = [0]
    unvisited.remove(0)
    current = 0
    while unvisited:
        cur_pass = passes[current]
        nxt = min(unvisited, key=lambda k: model.cost(cur_pass, passes[k]))
        order.append(nxt)
        unvisited.remove(nxt)
        current = nxt

    # Bounded 2-opt refines the distance objective. We only run it for the
    # symmetric (distance-only) case: with directed turn costs a segment reversal
    # flips every internal heading, which the simple endpoint delta cannot model,
    # so it could silently reintroduce U-turns. The turn-aware NN already orders
    # passes against the right (time) objective.
    if not direction_aware and n <= two_opt_cap:
        order = _two_opt(passes, order, grade_return=grade_return)

    total = sum(_deadhead(passes[order[k]], passes[order[k + 1]], grade_return)
                for k in range(len(order) - 1))
    ordered = [passes[k] for k in order]
    return Route(order=order, ordered_passes=ordered, total_deadhead_m=total)


def _cost_or0(model: _TransitionModel, a: Optional[Pass], b: Optional[Pass]) -> float:
    """Transition cost between two passes, or 0 when either end is missing.

    A missing end is the start (before the first pass) or finish (after the last
    pass) of the route, where there is no inter-pass transition to charge.
    """
    if a is None or b is None:
        return 0.0
    return model.cost(a, b)


def _route_objective(model: _TransitionModel, passes: List[Pass]) -> float:
    """Total order-dependent cost of a route: the sum of every consecutive
    pass-to-pass transition (return drive + turn-into-cut when direction-aware,
    else plain deadhead distance). This is exactly the part of the overall job
    time that reordering passes can change - each pass's own cut/haul/fill/grade
    sweep is fixed by its geometry - so minimizing it minimizes overall time.
    """
    return sum(model.cost(passes[k], passes[k + 1]) for k in range(len(passes) - 1))


def _relocate_delta(model: _TransitionModel, seq: List[Pass], s: int, d: int) -> float:
    """Change in :func:`_route_objective` from moving ``seq[s]`` to new index ``d``.

    ``d`` is the index the element will occupy in the list *after* it is removed
    from ``s`` (0..len-1; len reproduces an append). Only the few transitions
    around the source and destination change, so the delta is O(1): all other
    edges are untouched. A negative result is an improvement.
    """
    n = len(seq)
    x = seq[s]
    prev_s = seq[s - 1] if s > 0 else None
    next_s = seq[s + 1] if s < n - 1 else None
    removed = _cost_or0(model, prev_s, x) + _cost_or0(model, x, next_s)
    bridge = _cost_or0(model, prev_s, next_s)        # prev/next join once x leaves

    def orig(p: int) -> Optional[Pass]:
        # element at new position p in the list with index s removed
        if p < 0 or p > n - 2:
            return None
        return seq[p] if p < s else seq[p + 1]

    left = orig(d - 1)
    right = orig(d)
    removed_ins = _cost_or0(model, left, right)      # edge broken to insert x
    added_ins = _cost_or0(model, left, x) + _cost_or0(model, x, right)
    return (bridge - removed) + (added_ins - removed_ins)


def refine_route_order(
    route: Route,
    window: int = 50,
    grade_return: bool = True,
    direction_aware: bool = True,
    return_speed_mph: float = DEFAULT_RETURN_SPEED_MPH,
    turn_speed_mph: float = DEFAULT_TURN_SPEED_MPH,
    turning_circle_ft: float = DEFAULT_TURNING_CIRCLE_FT,
    max_rounds: int = 12,
) -> Tuple[float, float, int]:
    """Local-search refinement of the routed pass order to cut overall time.

    The constructive nearest-neighbour sequencer leaves no improvement step for
    the direction-aware (asymmetric) cost, because the 2-opt that would refine it
    reverses segments - flipping every internal heading - which is unsafe with
    directed turn costs. This walks the route one pass at a time and, for each
    pass, considers the window of the next ``window`` passes ahead, relocating the
    pass to the best slot within that window. Relocation (or-opt: lift one pass
    and re-insert it) never reverses a run, so it preserves each pass's internal
    heading and is safe with the directed turn cost. Every accepted move strictly
    lowers :func:`_route_objective` (the order-dependent time), so the search
    converges; it repeats until a full sweep finds no improvement (or ``max_rounds``).

    Mutates ``route`` in place (``ordered_passes``, ``order`` and
    ``total_deadhead_m``). Returns ``(objective_before, objective_after, n_moves)``;
    the objective is route transition time in seconds when ``direction_aware``,
    else deadhead distance in metres.
    """
    seq = route.ordered_passes
    n = len(seq)
    model = _TransitionModel(
        grade_return=grade_return,
        direction_aware=direction_aware,
        return_v=return_speed_mph * MPH_TO_MPS,
        turn_v=turn_speed_mph * MPH_TO_MPS,
        R=(turning_circle_ft * FEET_TO_METERS) / 2.0,
    )
    before = _route_objective(model, seq)
    if n < 3 or window < 2:
        return before, before, 0

    # Track the original pass index alongside each pass so ``route.order`` stays
    # consistent after the permutation.
    tagged = list(zip(seq, route.order))
    moves = 0
    eps = 1e-7
    for _round in range(max_rounds):
        improved = False
        k = 0
        while k < len(tagged):
            cur = [t[0] for t in tagged]
            lo = max(0, k - window)
            hi = min(len(tagged) - 1, k + window)
            best_d, best_delta = k, -eps
            for d in range(lo, hi + 1):
                if d == k:
                    continue
                delta = _relocate_delta(model, cur, k, d)
                if delta < best_delta:
                    best_delta, best_d = delta, d
            if best_d != k:
                item = tagged.pop(k)
                tagged.insert(best_d, item)
                moves += 1
                improved = True
            k += 1
        if not improved:
            break

    route.ordered_passes = [t[0] for t in tagged]
    route.order = [t[1] for t in tagged]
    route.total_deadhead_m = sum(
        _deadhead(route.ordered_passes[k], route.ordered_passes[k + 1], grade_return)
        for k in range(len(route.ordered_passes) - 1)
    )
    after = _route_objective(model, route.ordered_passes)
    return before, after, moves


def _two_opt(passes: List[Pass], order: List[int], max_iter: int = 30,
             grade_return: bool = True) -> List[int]:
    n = len(order)
    improved = True
    it = 0
    while improved and it < max_iter:
        improved = False
        it += 1
        for i in range(n - 1):
            a_end = _exit_xy(passes[order[i]], grade_return)
            b_start = passes[order[i + 1]].cut_start_xy
            d_ab = math.dist(a_end, b_start)
            for k in range(i + 2, n):
                c = passes[order[k]]
                # cost of reversing segment order[i+1 .. k]
                new_d_a = math.dist(a_end, passes[order[k]].cut_start_xy)
                if k + 1 < n:
                    old_tail = math.dist(_exit_xy(c, grade_return),
                                         passes[order[k + 1]].cut_start_xy)
                    new_tail = math.dist(
                        _exit_xy(passes[order[i + 1]], grade_return),
                        passes[order[k + 1]].cut_start_xy
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
