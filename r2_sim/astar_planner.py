"""
A* path planner for the Robocon 2026 Kung Fu Quest field.

Builds an occupancy grid from known field geometry, inflates obstacles
by the robot radius, and runs A* to find collision-free paths.

Usage:
    planner = FieldPlanner(resolution=0.05, robot_radius=0.35)
    path = planner.plan(start_x, start_y, goal_x, goal_y)
    # path = [(x1,y1), (x2,y2), ...] in world frame, or [] if no path
"""

import heapq
import math
from typing import List, Tuple, Optional

# ── Field dimensions ─────────────────────────────────────────────────────────
FIELD_W = 12.18   # metres
FIELD_H = 12.18
FIELD_X_MIN = -FIELD_W / 2   # -6.09
FIELD_X_MAX =  FIELD_W / 2   #  6.09
FIELD_Y_MIN = -FIELD_H / 2   # -6.09
FIELD_Y_MAX =  FIELD_H / 2   #  6.09

# ── Obstacles (world frame) ─────────────────────────────────────────────────
# Each obstacle: (cx, cy, half_w, half_h)  — axis-aligned bounding box
#
# Meihua Forest platforms — 1.2×1.2m per platform, touching edge-to-edge.
# The 3×4 grid forms two solid rectangular blocks:
#   Left forest:  x ∈ [-4.8, -1.2]  y ∈ [-2.4, 2.4]  → center (-3.0, 0)
#   Right forest: x ∈ [ 1.2,  4.8]  y ∈ [-2.4, 2.4]  → center ( 3.0, 0)
PLATFORMS = [
    (-3.0,  0.0,  1.80,  2.40),   # left forest block
    ( 3.0,  0.0,  1.80,  2.40),   # right forest block
]

# Staff racks (800×50mm footprint, but treat as wider for safety)
STAFF_RACKS = [
    (-3.44, 6.065, 0.45, 0.10),
    ( 3.44, 6.065, 0.45, 0.10),
]

# Spearhead rack (300×1200mm)
SPEARHEAD_RACK = [
    (0.0, 4.69, 0.20, 0.65),
]

# Tic-tac-toe rack (approximate)
TICTACTOE = [
    (0.0, -4.5, 0.80, 0.30),
]

# Arena slope / platform
ARENA_OBST = [
    (0.0, -5.0, 1.5, 0.40),
]

# Fence walls (100mm thick boundary)
_FENCE_T = 0.10
FENCES = [
    (0.0,  FIELD_Y_MAX, FIELD_W / 2, _FENCE_T),   # north
    (0.0,  FIELD_Y_MIN, FIELD_W / 2, _FENCE_T),   # south
    (FIELD_X_MIN, 0.0, _FENCE_T, FIELD_H / 2),    # west
    (FIELD_X_MAX, 0.0, _FENCE_T, FIELD_H / 2),    # east
]

ALL_OBSTACLES = PLATFORMS + STAFF_RACKS + SPEARHEAD_RACK + TICTACTOE + ARENA_OBST + FENCES


# ── Occupancy grid ───────────────────────────────────────────────────────────
class OccupancyGrid:
    """2-D boolean grid.  True = occupied / inflated obstacle."""

    def __init__(self, resolution: float = 0.05):
        self.res = resolution
        self.cols = int(math.ceil(FIELD_W / resolution))
        self.rows = int(math.ceil(FIELD_H / resolution))
        self.grid = [[False] * self.cols for _ in range(self.rows)]

    # coordinate ↔ grid conversions
    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        c = int((wx - FIELD_X_MIN) / self.res)
        r = int((wy - FIELD_Y_MIN) / self.res)
        c = max(0, min(self.cols - 1, c))
        r = max(0, min(self.rows - 1, r))
        return r, c

    def grid_to_world(self, r: int, c: int) -> Tuple[float, float]:
        wx = FIELD_X_MIN + (c + 0.5) * self.res
        wy = FIELD_Y_MIN + (r + 0.5) * self.res
        return wx, wy

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.rows and 0 <= c < self.cols

    def mark_rect(self, cx: float, cy: float, hw: float, hh: float):
        """Mark a rectangle as occupied."""
        r_min, c_min = self.world_to_grid(cx - hw, cy - hh)
        r_max, c_max = self.world_to_grid(cx + hw, cy + hh)
        for r in range(r_min, r_max + 1):
            for c in range(c_min, c_max + 1):
                if self.in_bounds(r, c):
                    self.grid[r][c] = True

    def inflate(self, radius: float):
        """Inflate all occupied cells by *radius* (in metres)."""
        pad = int(math.ceil(radius / self.res))
        old = [row[:] for row in self.grid]
        for r in range(self.rows):
            for c in range(self.cols):
                if old[r][c]:
                    for dr in range(-pad, pad + 1):
                        for dc in range(-pad, pad + 1):
                            if dr * dr + dc * dc <= pad * pad:
                                nr, nc = r + dr, c + dc
                                if self.in_bounds(nr, nc):
                                    self.grid[nr][nc] = True

    def is_free(self, r: int, c: int) -> bool:
        return self.in_bounds(r, c) and not self.grid[r][c]


# ── A* search ────────────────────────────────────────────────────────────────
# 8-connected grid neighbours
_DIRS = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414),
]


def _heuristic(r1: int, c1: int, r2: int, c2: int) -> float:
    """Octile distance heuristic."""
    dr = abs(r1 - r2)
    dc = abs(c1 - c2)
    return max(dr, dc) + 0.414 * min(dr, dc)


def _astar(grid: OccupancyGrid,
           start: Tuple[int, int],
           goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    """A* on an 8-connected grid.  Returns list of (row, col) or None."""
    sr, sc = start
    gr, gc = goal

    if not grid.is_free(sr, sc) or not grid.is_free(gr, gc):
        return None

    open_set = [(0.0, sr, sc)]
    g_score = {(sr, sc): 0.0}
    came_from = {}

    while open_set:
        _, cr, cc = heapq.heappop(open_set)

        if (cr, cc) == (gr, gc):
            # reconstruct
            path = [(gr, gc)]
            while (cr, cc) in came_from:
                cr, cc = came_from[(cr, cc)]
                path.append((cr, cc))
            path.reverse()
            return path

        for dr, dc, cost in _DIRS:
            nr, nc = cr + dr, cc + dc
            if not grid.is_free(nr, nc):
                continue
            ng = g_score[(cr, cc)] + cost
            if ng < g_score.get((nr, nc), float('inf')):
                g_score[(nr, nc)] = ng
                f = ng + _heuristic(nr, nc, gr, gc)
                came_from[(nr, nc)] = (cr, cc)
                heapq.heappush(open_set, (f, nr, nc))

    return None   # no path


# ── Path smoothing ───────────────────────────────────────────────────────────
def _line_of_sight(grid: OccupancyGrid,
                   r1: int, c1: int, r2: int, c2: int) -> bool:
    """Bresenham-style check: is the straight line free of obstacles?"""
    dr = abs(r2 - r1)
    dc = abs(c2 - c1)
    sr = 1 if r1 < r2 else -1
    sc = 1 if c1 < c2 else -1
    err = dr - dc
    r, c = r1, c1

    while True:
        if not grid.is_free(r, c):
            return False
        if r == r2 and c == c2:
            return True
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc


def _smooth_path(grid: OccupancyGrid,
                 path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """Remove redundant waypoints where line-of-sight exists."""
    if len(path) <= 2:
        return path

    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        # find the furthest point visible from path[i]
        furthest = i + 1
        for j in range(i + 2, len(path)):
            if _line_of_sight(grid, path[i][0], path[i][1],
                              path[j][0], path[j][1]):
                furthest = j
        smoothed.append(path[furthest])
        i = furthest

    return smoothed


# ── Public planner ───────────────────────────────────────────────────────────
class FieldPlanner:
    """
    A* planner for the Robocon field.

    Parameters
    ----------
    resolution : float
        Grid cell size in metres (default 0.05 m = 5 cm).
    robot_radius : float
        Inflation radius in metres (default 0.25 m ≈ half robot width + margin).
    """

    def __init__(self, resolution: float = 0.05, robot_radius: float = 0.25):
        self.grid = OccupancyGrid(resolution)

        # mark all known obstacles
        for cx, cy, hw, hh in ALL_OBSTACLES:
            self.grid.mark_rect(cx, cy, hw, hh)

        # inflate by robot radius
        self.grid.inflate(robot_radius)

    def _nearest_free(self, r: int, c: int) -> Optional[Tuple[int, int]]:
        """BFS outward from (r, c) to find the nearest free cell."""
        if self.grid.is_free(r, c):
            return (r, c)
        from collections import deque
        visited = {(r, c)}
        q = deque([(r, c)])
        while q:
            cr, cc = q.popleft()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = cr + dr, cc + dc
                    if (nr, nc) in visited:
                        continue
                    visited.add((nr, nc))
                    if not self.grid.in_bounds(nr, nc):
                        continue
                    if self.grid.is_free(nr, nc):
                        return (nr, nc)
                    q.append((nr, nc))
        return None

    def plan(self, sx: float, sy: float,
             gx: float, gy: float) -> List[Tuple[float, float]]:
        """
        Plan a path from (sx, sy) to (gx, gy) in world coordinates.

        If start or goal is inside an inflated obstacle, the nearest
        free cell is used instead (the robot approaches the obstacle edge).

        Returns a list of (x, y) waypoints (world frame), or empty list
        if no path exists.
        """
        start = self.grid.world_to_grid(sx, sy)
        goal = self.grid.world_to_grid(gx, gy)

        # Snap to nearest free cell if inside an obstacle
        start = self._nearest_free(*start) or start
        goal = self._nearest_free(*goal) or goal

        raw = _astar(self.grid, start, goal)
        if raw is None:
            return []

        smoothed = _smooth_path(self.grid, raw)

        # convert back to world coordinates
        return [self.grid.grid_to_world(r, c) for r, c in smoothed]
