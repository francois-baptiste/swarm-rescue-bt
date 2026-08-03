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
        self.victims = {}  # vid -> {"pos": (x, y), "rescued": bool, "rescued_by": int|None}

    def _build_walls(self):
        # Hand-placed maze: a vertical wall with a gap, a horizontal wall with
        # a gap. Guaranteed connected (verified in main.py at startup).
        for y in list(range(0, 6)) + list(range(8, 15)):
            self.grid[y][6] = WALL
        for x in list(range(0, 4)) + [x for x in range(6, 15) if x != 10]:
            self.grid[9][x] = WALL

    def is_free(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height and self.grid[y][x] == FREE

    def add_victim(self, vid, pos):
        assert self.is_free(*pos), f"victim {vid} placed on a wall/out of bounds: {pos}"
        self.victims[vid] = {"pos": pos, "rescued": False, "rescued_by": None}

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
