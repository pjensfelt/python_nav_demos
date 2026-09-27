"""Path planners: A* on the grid, RRT and RRT* in the continuous plane.

Written to be read. Each planner returns a PlanResult with the path and
a record of how the search went (which cells A* expanded, in what order;
which edges the trees grew and rewired), so the demo can replay the
search step by step afterwards. The recording is one line in each loop
and is the only thing here that isn't the algorithm itself.

A* is based on the structure of the WASP assignment 3 template
(KTH-RPL/wasp_autonomous_systems, src/wasp_as_ass_3), completed. RRT
follows the same file. RRT* here has both halves of the algorithm --
choosing the best parent among the near nodes, then rewiring the near
nodes through the new one, with the cost change passed on to their
descendants -- where the assignment version only rewires.
"""

import heapq
import time
from dataclasses import dataclass, field
from math import hypot, sqrt

import numpy as np


@dataclass
class PlanResult:
    path: list                       # [(x, y), ...] start to goal; [] if none found
    cost: float = np.inf             # path length [m]
    expanded: list = field(default_factory=list)   # A*: cells in expansion order
    nodes: np.ndarray = None         # RRT/RRT*: node coordinates, N x 2
    events: list = field(default_factory=list)     # RRT/RRT*: (child, parent) in order
    iterations: int = 0
    seconds: float = 0.0
    message: str = ""


def path_length(path):
    return sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


# ==========================================================================
# A*
# ==========================================================================

def neighbours(free, ix, iy, eight):
    """Free neighbouring cells, and the cost of stepping to each.

    With 8-connectivity a diagonal step is only allowed if both cells it
    cuts past are free too -- otherwise the path would squeeze diagonally
    between two obstacles touching at a corner.
    """
    out = []
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        if free(ix + dx, iy + dy):
            out.append(((ix + dx, iy + dy), 1.0))
    if eight:
        for dx, dy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            if free(ix + dx, iy + dy) and free(ix + dx, iy) and free(ix, iy + dy):
                out.append(((ix + dx, iy + dy), sqrt(2)))
    return out


def heuristic(a, b, eight):
    """Distance from cell a to cell b if there were no obstacles:
    Manhattan for 4-connectivity, octile for 8. Never overestimates, so A*
    with weight 1 finds the shortest path on the grid."""
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    if not eight:
        return dx + dy
    return max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)


def smoothed_cost(occ, res, sigma):
    """A cost for being near obstacles: the planning map blurred with a
    Gaussian of standard deviation `sigma` [m], so 0 far from anything,
    rising towards 1 at the (inflated) obstacles. Made from the map alone,
    as for inflation grid first: no model of the world needed."""
    s = max(sigma / res, 1e-6)                      # in cells
    r = int(np.ceil(3 * s))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / s) ** 2)
    k /= k.sum()
    c = np.pad(occ.astype(float), r, mode="edge")   # beyond the map: as at its edge
    c = np.apply_along_axis(np.convolve, 0, c, k, mode="valid")
    c = np.apply_along_axis(np.convolve, 1, c, k, mode="valid")
    return np.clip(c, 0.0, 1.0)


def astar(grid, start, goal, eight=True, weight=1.0, free=None, cost=None, cost_weight=0.0):
    """A* from `start` to `goal` (world coordinates) over the grid's cells.

    f = g + weight * h.  weight = 0 is Dijkstra (no heuristic at all),
    weight = 1 is A*, weight > 1 is weighted A*: fewer expansions, but the
    path is no longer guaranteed to be the shortest.

    With a `cost` map (see smoothed_cost), stepping into a cell costs
    step * (1 + cost_weight * cost[cell]): A* then trades path length for
    clearance. The heuristic is still the plain distance, which can only
    underestimate such a cost, so the path is still the cheapest.

    `free(ix, iy)` decides which cells can be entered; grid.free by
    default (see GridChecker for why you might want another).
    """
    t0 = time.perf_counter()
    free = grid.free if free is None else free
    s, g = grid.cell(*start), grid.cell(*goal)
    if not free(*s) or not free(*g):
        which = "start" if not free(*s) else "goal"
        return PlanResult([], message=f"{which} is in an occupied cell")

    came_from = {}
    cost_so_far = {s: 0.0}
    closed = set()
    expanded = []
    # Heap entries: (f, h, counter, cell). Ties in f go to the smaller h,
    # i.e. the cell closer to the goal; the counter keeps the order stable.
    h0 = heuristic(s, g, eight)
    open_heap = [(weight * h0, h0, 0, s)]
    counter = 1

    while open_heap:
        _, _, _, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue          # a stale entry: we already found a cheaper way here
        closed.add(cur)
        expanded.append(cur)

        if cur == g:
            cells = [cur]
            while cells[-1] in came_from:
                cells.append(came_from[cells[-1]])
            cells.reverse()
            # Cell centres from start to goal, with the real start and goal
            # points at the ends instead of their cells' centres.
            path = [tuple(start)] + [grid.coord(*c) for c in cells[1:-1]] + [tuple(goal)]
            return PlanResult(path, path_length(path), expanded=expanded,
                              seconds=time.perf_counter() - t0)

        for nb, step in neighbours(free, *cur, eight):
            if cost is not None and cost_weight > 0:
                step *= 1.0 + cost_weight * cost[nb]
            new_cost = cost_so_far[cur] + step
            if new_cost < cost_so_far.get(nb, np.inf):
                cost_so_far[nb] = new_cost
                came_from[nb] = cur
                h = heuristic(nb, g, eight)
                heapq.heappush(open_heap, (new_cost + weight * h, h, counter, nb))
                counter += 1

    return PlanResult([], expanded=expanded, seconds=time.perf_counter() - t0,
                      message="no path: the goal is not reachable on this grid")


# ==========================================================================
# RRT and RRT*
# ==========================================================================

def steer(p, q, step):
    """Move from p toward q, at most `step`."""
    d = hypot(q[0] - p[0], q[1] - p[1])
    if d <= step:
        return q
    return (p[0] + (q[0] - p[0]) * step / d, p[1] + (q[1] - p[1]) * step / d)


class Tree:
    """Nodes in a preallocated array, so nearest-neighbour search is one
    numpy expression rather than a loop -- brute force, but fast enough for
    a few thousand nodes and much easier to read than a k-d tree."""

    def __init__(self, root, capacity):
        self.xy = np.empty((capacity, 2))
        self.xy[0] = root
        self.parent = np.full(capacity, -1)
        self.cost = np.zeros(capacity)       # path length from the root
        self.n = 1

    def add(self, p, parent, cost):
        i = self.n
        self.xy[i], self.parent[i], self.cost[i] = p, parent, cost
        self.n += 1
        return i

    def dist(self, p):
        return np.hypot(*(self.xy[:self.n] - p).T)

    def nearest(self, p):
        return int(np.argmin(self.dist(p)))

    def near(self, p, radius):
        return np.flatnonzero(self.dist(p) <= radius)

    def path_to(self, i):
        path = []
        while i >= 0:
            path.append(tuple(self.xy[i]))
            i = self.parent[i]
        return path[::-1]


def _sample(rng, bounds, goal, goal_bias):
    """A uniformly random point in the world -- or, with probability
    goal_bias, the goal itself, which pulls the tree toward it."""
    if rng.random() < goal_bias:
        return goal
    xmin, xmax, ymin, ymax = bounds
    return (rng.uniform(xmin, xmax), rng.uniform(ymin, ymax))


def rrt(checker, bounds, start, goal, iterations=2000, step=0.5, goal_bias=0.05,
        stop_at_goal=True, rng=None):
    """Rapidly-exploring random tree.

    Each iteration: sample a point, find the nearest node in the tree, take
    one step of at most `step` toward the sample, and keep the new node if
    that edge is collision free. The goal is reached once a node lands
    within `step` of it with a free straight line to it.
    """
    rng = np.random.default_rng() if rng is None else rng
    t0 = time.perf_counter()
    if not checker.point_free(start) or not checker.point_free(goal):
        return PlanResult([], message="start or goal is in collision")

    tree = Tree(start, iterations + 2)
    events = []
    goal_node = None

    for it in range(1, iterations + 1):
        q_rand = _sample(rng, bounds, goal, goal_bias)
        i_near = tree.nearest(q_rand)
        q_new = steer(tuple(tree.xy[i_near]), q_rand, step)
        if not checker.segment_free(tuple(tree.xy[i_near]), q_new):
            continue
        i_new = tree.add(q_new, i_near, tree.cost[i_near] + hypot(
            q_new[0] - tree.xy[i_near, 0], q_new[1] - tree.xy[i_near, 1]))
        events.append((i_new, i_near))

        if goal_node is None and hypot(goal[0] - q_new[0], goal[1] - q_new[1]) <= step \
                and checker.segment_free(q_new, goal):
            goal_node = tree.add(goal, i_new, tree.cost[i_new] + hypot(
                goal[0] - q_new[0], goal[1] - q_new[1]))
            events.append((goal_node, i_new))
            if stop_at_goal:
                break

    return _tree_result(tree, goal_node, events, it, t0)


def rrtstar(checker, bounds, start, goal, iterations=2000, step=0.5, goal_bias=0.05,
            radius=1.0, rng=None):
    """RRT*: RRT plus two improvements that make the path converge toward
    the shortest one as the iterations go on.

    1. Choose parent: connect the new node not to the nearest node, but to
       whichever node within `radius` gives it the shortest path from the
       start.
    2. Rewire: for every node within `radius`, if going through the new
       node would make *its* path shorter, make the new node its parent.

    Unlike RRT it does not stop at the first path found -- running longer
    is what improves it.
    """
    rng = np.random.default_rng() if rng is None else rng
    t0 = time.perf_counter()
    if not checker.point_free(start) or not checker.point_free(goal):
        return PlanResult([], message="start or goal is in collision")

    tree = Tree(start, iterations + 2)
    children = {0: set()}
    events = []

    for it in range(1, iterations + 1):
        q_rand = _sample(rng, bounds, goal, goal_bias)
        i_nearest = tree.nearest(q_rand)
        q_new = steer(tuple(tree.xy[i_nearest]), q_rand, step)
        if not checker.segment_free(tuple(tree.xy[i_nearest]), q_new):
            continue

        # 1. choose the best parent among the near nodes (the nearest node
        #    is always a candidate: we already know its edge is free)
        near = np.union1d(tree.near(q_new, radius), [i_nearest])
        d_near = np.hypot(*(tree.xy[near] - q_new).T)
        through = tree.cost[near] + d_near       # path length via each candidate
        for k in np.argsort(through):            # cheapest first: first free one wins
            j = near[k]
            if j == i_nearest or checker.segment_free(tuple(tree.xy[j]), q_new):
                best, best_cost = j, through[k]
                break
        i_new = tree.add(q_new, best, best_cost)
        children[i_new] = set()
        children[best].add(i_new)
        events.append((i_new, best))

        # 2. rewire the near nodes through the new one
        for j, d in zip(near, d_near):
            if j == best:
                continue
            c = best_cost + d
            if c < tree.cost[j] - 1e-12 and checker.segment_free(q_new, tuple(tree.xy[j])):
                children[tree.parent[j]].discard(j)
                tree.parent[j] = i_new
                children[i_new].add(j)
                _propagate_cost(tree, children, j, c - tree.cost[j])
                events.append((j, i_new))

    # Best way to finish: any node within `step` of the goal with a free
    # straight line to it, picking the one with the shortest total.
    goal_node = None
    cand = tree.near(goal, step)
    totals = tree.cost[cand] + np.hypot(*(tree.xy[cand] - goal).T)
    for k in np.argsort(totals):
        j = cand[k]
        if checker.segment_free(tuple(tree.xy[j]), goal):
            goal_node = tree.add(goal, j, totals[k])
            events.append((goal_node, j))
            break

    return _tree_result(tree, goal_node, events, it, t0)


def _propagate_cost(tree, children, i, delta):
    """A rewired node's cost changed by `delta`, and so has every node
    below it in the tree. Forgetting this is a classic RRT* bug: the
    costs silently go stale and later choices are made on wrong numbers."""
    stack = [i]
    while stack:
        k = stack.pop()
        tree.cost[k] += delta
        stack.extend(children.get(k, ()))


def _tree_result(tree, goal_node, events, iterations, t0):
    nodes = tree.xy[:tree.n].copy()
    if goal_node is None:
        return PlanResult([], nodes=nodes, events=events, iterations=iterations,
                          seconds=time.perf_counter() - t0,
                          message=f"no path after {iterations} iterations")
    path = tree.path_to(goal_node)
    return PlanResult(path, path_length(path), nodes=nodes, events=events,
                      iterations=iterations, seconds=time.perf_counter() - t0)


# ==========================================================================
# Post-processing
# ==========================================================================

def shortcut(checker, path):
    """Greedy shortcutting (optimize_path in the WASP GridMap): from each
    point, jump straight to the furthest later point that is still in
    free line of sight."""
    if len(path) < 3:
        return list(path)
    out, i = [path[0]], 0
    while i < len(path) - 1:
        for j in range(len(path) - 1, i, -1):
            if j == i + 1 or checker.segment_free(path[i], path[j]):
                out.append(path[j])
                i = j
                break
    return out


def resample(path, spacing):
    """Points along the polyline `path`, `spacing` apart (plus both ends)."""
    P = np.asarray(path, dtype=float)
    seg = np.hypot(*np.diff(P, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(int(np.ceil(s[-1] / spacing)), 1)
    t = np.linspace(0.0, s[-1], n + 1)
    return np.column_stack([np.interp(t, s, P[:, 0]), np.interp(t, s, P[:, 1])])


def _bspline_segments(ctrl, step=0.02):
    """The pieces of a uniform cubic B-spline with control points `ctrl`,
    each sampled about `step` apart. Piece i depends on control points
    i-2 .. i+1 only. The end points are tripled, so the curve starts and
    ends exactly on the path's ends."""
    P = np.asarray(ctrl, dtype=float)
    P = np.vstack([P[:1], P[:1], P, P[-1:], P[-1:]])
    h = np.hypot(*np.diff(P, axis=0).T).max()
    t = np.linspace(0.0, 1.0, max(int(np.ceil(h / step)), 2) + 1)[:, None]
    b = np.hstack([(1 - t) ** 3, 3 * t ** 3 - 6 * t ** 2 + 4,
                   -3 * t ** 3 + 3 * t ** 2 + 3 * t + 1, t ** 3]) / 6.0
    return [b @ P[i:i + 4] for i in range(len(P) - 3)]


def bspline(ctrl, step=0.02):
    """A uniform cubic B-spline with `ctrl` as its control points, sampled
    about `step` apart. It does not pass through the control points but
    rounds off between them -- smooth (continuous curvature), and inside
    the polygon they form, so it cannot overshoot."""
    segs = _bspline_segments(ctrl, step)
    pts = np.vstack([segs[0]] + [g[1:] for g in segs[1:]])
    keep = np.concatenate([[True], np.hypot(*np.diff(pts, axis=0).T) > 1e-9])
    return pts[keep]


def max_curvature(pts, step=0.02):
    """The largest curvature [1/m] along a path, measured at even steps
    along it: the turn from one step to the next over the step length."""
    P = resample(pts, step)
    d = np.diff(P, axis=0)
    if len(d) < 2:
        return 0.0
    turn = np.abs(np.angle(np.exp(1j * np.diff(np.arctan2(d[:, 1], d[:, 0])))))
    return float(turn.max() / step)


def spline(checker, path, spacing, min_spacing=0.05, max_rounds=20):
    """Smooth `path` with a B-spline, its control points `spacing` apart
    along the path.

    Rounding a corner cuts inside it, towards the obstacle the path goes
    round, so the curve has to be checked again. Where a piece of it hits
    the map, the control points around that piece are put closer together
    (halving the gaps there), so the curve hugs the path more tightly at
    that corner only, and it is checked again. Returns (points, the
    smallest spacing needed), or (None, None) if a piece still collides
    with its control points `min_spacing` apart."""
    P = np.asarray(path, dtype=float)
    seg = np.hypot(*np.diff(P, axis=0).T)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(int(np.ceil(cum[-1] / spacing)), 1)
    s_ctrl = list(np.linspace(0.0, cum[-1], n + 1))

    for _ in range(max_rounds):
        ctrl = np.column_stack([np.interp(s_ctrl, cum, P[:, 0]), np.interp(s_ctrl, cum, P[:, 1])])
        segs = _bspline_segments(ctrl)
        bad = [i for i, g in enumerate(segs)
               if not all(checker.segment_free(a, b) for a, b in zip(g[:-1], g[1:]))]
        if not bad:
            pts = bspline(ctrl)
            gaps = np.diff(s_ctrl)
            return [tuple(p) for p in pts], float(gaps.min())
        # piece i depends on control points i-2 .. i+1: halve the gaps
        # between them (and one either side)
        new = set()
        for i in bad:
            for k in range(max(i - 3, 0), min(i + 2, len(s_ctrl) - 1)):
                if s_ctrl[k + 1] - s_ctrl[k] > 2 * min_spacing - 1e-9:
                    new.add(0.5 * (s_ctrl[k] + s_ctrl[k + 1]))
        if not new:
            return None, None
        s_ctrl = sorted(set(s_ctrl) | new)
    return None, None


# ==========================================================================
# Potential field
# ==========================================================================

def field_force(x, goal, samples, k_rep, d0, clearance, spacing, d_goal=1.0):
    """The force at x: pulled towards the goal, pushed away from every
    sample point within d0.

    Attraction: towards the goal with magnitude 1, falling off linearly
    within d_goal of it (a cone far away, a bowl near the goal).
    Repulsion, the classic (Khatib) form per sample point p, with rho the
    distance left before the robot's `clearance` is used up:

        k_rep * spacing * (1/rho - 1/d0) / rho^2     away from p, for rho < d0

    Scaled by the sample spacing so that a denser survey of the same wall
    doesn't push harder."""
    g = goal - x
    dg = np.hypot(*g)
    f = g / max(dg, d_goal)
    if len(samples):
        d_vec = x - samples
        d = np.hypot(d_vec[:, 0], d_vec[:, 1])
        rho = np.maximum(d - clearance, 1e-3)
        near = rho < d0
        if near.any():
            mag = k_rep * spacing * (1 / rho[near] - 1 / d0) / rho[near] ** 2
            f = f + (mag / np.maximum(d[near], 1e-9)) @ d_vec[near]
    return f


def field_potential(pts, goal, samples, k_rep, d0, clearance, spacing, d_goal=1.0):
    """The potential whose (negative) gradient field_force is, at points
    `pts` (N x 2) -- for drawing its contours."""
    dg = np.hypot(*(pts - goal).T)
    u = np.where(dg < d_goal, 0.5 * dg ** 2 / d_goal, dg - 0.5 * d_goal)
    for p in samples:
        d = np.hypot(*(pts - p).T)
        rho = np.maximum(d - clearance, 1e-3)
        u += np.where(rho < d0, 0.5 * k_rep * spacing * (1 / rho - 1 / d0) ** 2, 0.0)
    return u


def potential_field(samples, start, goal, bounds, k_rep, d0, clearance, spacing,
                    step=0.02, stuck_window=100, stuck_dist=0.1):
    """Follow the force downhill from `start`, a fixed `step` at a time, in
    the direction of the force. Reaches the goal -- or gets stuck: if the
    last `stuck_window` steps got less than `stuck_dist` anywhere, it is in
    a local minimum (typically in front of a wall, or oscillating in a
    narrow passage) and gives up there.

    The trace is returned as `nodes`/`events`, a chain, so it can be
    replayed like an RRT tree."""
    t0 = time.perf_counter()
    samples = np.asarray(samples, dtype=float).reshape(-1, 2)
    x = np.array(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    (x0, x1), (y0, y1) = (bounds[0], bounds[1]), (bounds[2], bounds[3])
    max_steps = int(4 * ((x1 - x0) + (y1 - y0)) / step)
    trace = [x.copy()]
    message = ""
    for i in range(max_steps):
        if np.hypot(*(goal - x)) < step:
            trace.append(goal.copy())
            break
        f = field_force(x, goal, samples, k_rep, d0, clearance, spacing)
        n = np.hypot(*f)
        if n < 1e-9:
            message = "stuck: the forces cancel exactly (a local minimum)"
            break
        x = x + step * f / n
        trace.append(x.copy())
        if i >= stuck_window and np.hypot(*(x - trace[-1 - stuck_window])) < stuck_dist:
            message = "stuck in a local minimum"
            break
    else:
        message = "gave up: no progress towards the goal"
    nodes = np.array(trace)
    events = [(j, j - 1) for j in range(1, len(nodes))]
    reached = not message
    path = [tuple(p) for p in nodes] if reached else []
    return PlanResult(path, path_length(path) if reached else np.inf, nodes=nodes,
                      events=events, iterations=len(nodes) - 1,
                      seconds=time.perf_counter() - t0, message=message)


# What 's' cycles through: the plan as it came from the planner, a spline
# of it, its shortcut, and a spline of the shortcut.
PATH_MODES = ("as planned", "spline", "shortcut", "shortcut + spline")


PLANNERS = ("A*", "RRT", "RRT*", "potential field")
