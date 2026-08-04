"""A tiny 2D grid world standing in for the map Nav2 would normally serve
(occupancy grid) plus the victims a search-and-rescue swarm must find.
BFS on this grid plays the role of Nav2's global planner + controller pair:
each robot asks it for a path to a goal, then walks the path one cell at a
time. Victim positions are known to the World object (ground truth) but a
robot only "perceives" a victim once it is within its own sensor_range -
the swarm has no shared global map of victims, only what each robot senses.
"""

from collections import deque

FREE, WALL = 0, 1


class World:
    def __init__(self, width=15, height=15):
        self.width = width
        self.height = height
        self.grid = [[FREE] * width for _ in range(height)]
        self._build_walls()
        self.victims = {}  # vid -> {"pose": (x, y), "rescued": bool, "rescued_by": int|None}

    def _build_walls(self):
        if self.width == 15 and self.height == 15:
            # Hand-placed maze: a vertical wall with a gap, a horizontal
            # wall with a gap. Guaranteed connected (verified in sim.py at
            # startup). Kept as its own literal branch, rather than folded
            # into the proportional formula below, so the default demo's
            # map/tick-counts never shift as a side effect of scenario work.
            for y in list(range(0, 6)) + list(range(8, 15)):
                self.grid[y][6] = WALL
            for x in list(range(0, 4)) + [x for x in range(6, 15) if x != 10]:
                self.grid[9][x] = WALL
            return

        # Generalized cross-shaped maze, scaled proportionally to whatever
        # size is requested: a vertical wall with one gap (connects the two
        # top quadrants) and a horizontal wall with *two* gaps, one on each
        # side of the vertical wall (connects top-left to bottom-left, and
        # top-right to bottom-right) - same shape as the hand-placed 15x15
        # maze above, needed so all four quadrants stay reachable from one
        # another instead of one being walled off entirely. Connectivity
        # for any given (width, height) is still checked at scenario-build
        # time in sim.py, not assumed here.
        vx = max(1, min(self.width - 2, round(self.width * 0.4)))
        gy0 = max(0, round(self.height * 0.4))
        gy1 = min(self.height, gy0 + max(2, round(self.height * 0.13)))
        for y in range(self.height):
            if not (gy0 <= y < gy1):
                self.grid[y][vx] = WALL

        hy = max(1, min(self.height - 2, round(self.height * 0.6)))
        gx0 = max(0, round(vx * 0.6))
        gx1 = min(vx, gx0 + max(2, round(vx * 0.25)))
        gx2 = min(self.width - 1, vx + max(1, round((self.width - vx) * 0.4)))
        for x in range(self.width):
            if x == vx or (gx0 <= x < gx1) or x == gx2:
                continue  # column vx: left to the vertical-wall loop above
            self.grid[hy][x] = WALL

    def is_free(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height and self.grid[y][x] == FREE

    def add_victim(self, vid, pos):
        assert self.is_free(*pos), f"victim {vid} placed on a wall/out of bounds: {pos}"
        self.victims[vid] = {"pose": pos, "rescued": False, "rescued_by": None}

    def neighbors(self, pos):
        x, y = pos
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if self.is_free(nx, ny):
                yield (nx, ny)

    def all_free_cells(self):
        return [
            (x, y)
            for y in range(self.height)
            for x in range(self.width)
            if self.grid[y][x] == FREE
        ]


def bfs_path(world, start, goal):
    """Shortest path start -> goal on the grid, or None if unreachable."""
    if start == goal:
        return [start]
    visited = {start: None}
    q = deque([start])
    while q:
        cur = q.popleft()
        if cur == goal:
            break
        for nxt in world.neighbors(cur):
            if nxt not in visited:
                visited[nxt] = cur
                q.append(nxt)
    if goal not in visited:
        return None
    path = [goal]
    while path[-1] != start:
        path.append(visited[path[-1]])
    path.reverse()
    return path


def reachable_cells(world, start):
    """Every free cell reachable from start - used to validate a scenario's
    map/robot/victim placement at build time (see sim.py's build_scenario)
    rather than assuming a hand- or formula-placed wall layout stays
    connected."""
    seen = {start}
    q = deque([start])
    while q:
        cur = q.popleft()
        for nxt in world.neighbors(cur):
            if nxt not in seen:
                seen.add(nxt)
                q.append(nxt)
    return seen


def perimeter_cells(world):
    """Free cells on the outer ring of the grid, in clockwise walking order
    starting from the top-left corner - used by the patrol mission to build
    a loop route. Border walls aren't part of any generated maze here (the
    walls in _build_walls are all interior), so this is normally the full
    ring, but is_free is still checked for robustness."""
    top, bottom = 0, world.height - 1
    left, right = 0, world.width - 1
    cells = [(x, top) for x in range(left, right + 1)]
    cells += [(right, y) for y in range(top + 1, bottom + 1)]
    cells += [(x, bottom) for x in range(right - 1, left - 1, -1)]
    cells += [(left, y) for y in range(bottom - 1, top, -1)]
    return [c for c in cells if world.is_free(*c)]


def bfs_nearest(world, start, targets):
    """Shortest path from start to the nearest cell in `targets` (a set)."""
    if start in targets:
        return [start]
    visited = {start: None}
    q = deque([start])
    while q:
        cur = q.popleft()
        if cur in targets:
            path = [cur]
            while path[-1] != start:
                path.append(visited[path[-1]])
            path.reverse()
            return path
        for nxt in world.neighbors(cur):
            if nxt not in visited:
                visited[nxt] = cur
                q.append(nxt)
    return None
