"""Border patrol: N robots split a shared perimeter loop into contiguous
arcs and each walks its own arc back and forth, forever. Unlike the
decentralized branches, the split isn't something each robot computes for
itself: a single Coordinator owns the loop and the arc assignment, and
rebalances it across whichever robots are still heartbeating whenever
that set changes - including recovering full coverage after a robot
dies, once its silence is noticed (see FAILURE_TIMEOUT, same detection
delay the rescue mission's Coordinator uses - a real Coordinator can't
know a robot died the instant it happens, only that it stopped reporting
in).
"""

import py_trees

from swarm.world import World, reachable_cells, perimeter_cells, bfs_path
from missions.common import tree_snapshot, PALETTE

TITLE = "Centralized Border Patrol"
Status = py_trees.common.Status

FAILURE_TIMEOUT = 15  # ticks of silence before the Coordinator drops a robot from the split

SCENARIOS = {
    "default": dict(
        description="4 robots split a 15x15 map's border into 4 arcs, no failures - full "
                     "coverage expected.",
        width=15, height=15,
        starts=[(0, 0), (14, 0), (14, 14), (0, 14)],
        failures=[],
        max_ticks=80,
        expect_full_coverage=True,
    ),
    "robot_down": dict(
        description="Same as default but robot 1 fails at tick 3 - the Coordinator notices "
                     "the silence (after FAILURE_TIMEOUT) and rebalances the loop across the "
                     "3 survivors, recovering full coverage instead of leaving a permanent gap.",
        width=15, height=15,
        starts=[(0, 0), (14, 0), (14, 14), (0, 14)],
        failures=[(1, 3)],
        max_ticks=80,
        expect_full_coverage=True,
    ),
    "double_failure": dict(
        description="Same map, but robots 1 and 2 both fail early (ticks 3 and 30) - the "
                     "Coordinator rebalances down to the 2 survivors each time, recovering full "
                     "coverage twice over instead of losing half the loop permanently (just "
                     "takes longer: 2 robots covering the whole loop take a while).",
        width=15, height=15,
        starts=[(0, 0), (14, 0), (14, 14), (0, 14)],
        failures=[(1, 3), (2, 30)],
        max_ticks=130,
        expect_full_coverage=True,
    ),
    "large_map": dict(
        description="25x25 map, 6 robots, no failures - tests scaling to a bigger loop and swarm.",
        width=25, height=25,
        starts=[(0, 0), (24, 0), (24, 24), (0, 24), (12, 0), (12, 24)],
        failures=[],
        max_ticks=150,
        expect_full_coverage=True,
    ),
}


class Coordinator:
    """Owns the perimeter loop and the arc split. Recomputes the split
    only when the alive set actually changes (not every tick), so a
    robot's own progress along its arc is never reset for no reason."""

    def __init__(self, loop):
        self.loop = loop
        self.events = []
        self._last_seen = {}
        self._current_alive = None

    def heartbeat(self, robot_id, t):
        self._last_seen[robot_id] = t

    def rebalance(self, robots, t):
        alive_ids = tuple(sorted(
            r.id for r in robots if t - self._last_seen.get(r.id, t) <= FAILURE_TIMEOUT))
        if alive_ids == self._current_alive:
            return
        self._current_alive = alive_ids
        if not alive_ids:
            return
        by_id = {r.id: r for r in robots}
        arcs = _split_route(self.loop, by_id[alive_ids[0]].pos, len(alive_ids))
        for rid, arc in zip(alive_ids, arcs):
            r = by_id[rid]
            r.route = arc
            r.route_idx = arc.index(r.pos) if r.pos in arc else 0
            r.direction = 1
        self.events.append((t, len(alive_ids), alive_ids))


class PickNextWaypoint(py_trees.behaviour.Behaviour):
    """Back-and-forth (ping-pong) along the robot's own arc - see
    missions/patrol.py on the decentralized branches for why (a loop
    wraparound could BFS-shortcut through another robot's arc)."""

    def __init__(self, robot):
        super().__init__("PickNextWaypoint")
        self.robot = robot

    def update(self):
        r = self.robot
        if len(r.route) > 1 and r.pos == r.route[r.route_idx]:
            nxt = r.route_idx + r.direction
            if nxt < 0 or nxt >= len(r.route):
                r.direction *= -1
                nxt = r.route_idx + r.direction
            r.route_idx = nxt
        return Status.SUCCESS


class NavigateToPose(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("NavigateToPose")
        self.robot = robot

    def update(self):
        r = self.robot
        goal = r.route[r.route_idx]
        if not r.path or r.path[0] != r.pos or r.path[-1] != goal:
            r.path = bfs_path(r.world, r.pos, goal)
        if r.path and len(r.path) >= 2:
            r.pos = r.path[1]
            r.path = r.path[1:]
        r.state = f"-> {goal}"
        return Status.RUNNING


class Robot:
    def __init__(self, rid, start, world, coordinator, color="C0"):
        self.id = rid
        self.pos = start
        self.world = world
        self.coordinator = coordinator
        self.color = color

        self.route = [start]
        self.route_idx = 0
        self.direction = 1
        self.path = []
        self.state = "PATROL"
        self.failed = False
        self.t = 0
        self.trail = [start]

        self.tree = self._build_tree()

    def _build_tree(self):
        return py_trees.composites.Selector("MissionRoot", memory=False, children=[
            py_trees.composites.Sequence("Patrol", memory=False, children=[
                PickNextWaypoint(self),
                NavigateToPose(self),
            ]),
        ])

    def tick(self, t):
        self.t = t
        if self.failed:
            self.state = "FAILED"
            return
        self.coordinator.heartbeat(self.id, t)
        self.tree.tick_once()
        self.trail.append(self.pos)


def _split_route(loop, start_cell, n):
    """Rotates loop so it starts at (or near) start_cell, then splits it
    into n contiguous, near-equal arcs."""
    if start_cell in loop:
        i = loop.index(start_cell)
        loop = loop[i:] + loop[:i]
    base, extra = divmod(len(loop), n)
    arcs, idx = [], 0
    for r in range(n):
        size = base + (1 if r < extra else 0)
        arcs.append(loop[idx:idx + size])
        idx += size
    return arcs


def build_scenario(name):
    spec = SCENARIOS[name]
    world = World(spec["width"], spec["height"])
    loop = perimeter_cells(world)

    free = set(world.all_free_cells())
    reach = reachable_cells(world, spec["starts"][0])
    for pos in spec["starts"]:
        assert pos in free, f"scenario {name!r}: {pos} is on a wall/out of bounds"
        assert pos in reach, f"scenario {name!r}: {pos} is unreachable from {spec['starts'][0]}"

    coordinator = Coordinator(loop)
    robots = [
        Robot(i, start, world, coordinator, color=PALETTE[i % len(PALETTE)])
        for i, start in enumerate(spec["starts"])
    ]
    # Seed an initial split before tick 0, rather than waiting a full
    # FAILURE_TIMEOUT for the Coordinator to "notice" robots that were
    # there from the start.
    for r in robots:
        coordinator.heartbeat(r.id, 0)
    coordinator.rebalance(robots, 0)

    return world, coordinator, robots, spec


def run(world, coordinator, robots, spec):
    """Unlike the rescue mission, patrol has no natural end - it always
    runs the full max_ticks."""
    frames = []
    covered = {r.pos for r in robots}
    for t in range(spec["max_ticks"]):
        for rid, fail_tick in spec["failures"]:
            if t == fail_tick:
                robots[rid].failed = True

        for r in robots:
            r.tick(t)
        coordinator.rebalance(robots, t)
        covered |= {r.pos for r in robots if not r.failed}

        frames.append({
            "t": t,
            "positions": [r.pos for r in robots],
            "states": [r.state for r in robots],
            "trails": [list(r.trail) for r in robots],
            "bt_snapshot": [tree_snapshot(r) for r in robots],
            "visited": set(covered),
        })
    return frames


def print_log(coordinator, world):
    print("=== Event log (centralized: the Coordinator owns the arc split) ===")
    print(f"Perimeter has {len(coordinator.loop)} free cells.")
    for t, n, alive_ids in coordinator.events:
        print(f"t={t:3d}  coordinator splits the loop into {n} arc(s) among robots {list(alive_ids)}")
    print()
    print("=== Summary ===")


def summarize(world, coordinator, spec, frames):
    covered = frames[-1]["visited"] & set(coordinator.loop)
    total = len(coordinator.loop)
    full = len(covered) == total
    ok = full if spec["expect_full_coverage"] else True
    label = f"{len(covered):3d}/{total:<3d} cells covered"
    return ok, label


def draw_extra(ax, frame, world):
    xs = [c[0] for c in frame["visited"]]
    ys = [c[1] for c in frame["visited"]]
    if xs:
        ax.plot(xs, ys, marker=".", color="black", linestyle="", markersize=4, zorder=2)


def legend_handles():
    from matplotlib.lines import Line2D
    return [
        Line2D([0], [0], marker=".", color="black", linestyle="",
               markersize=6, label="Currently patrolled cell"),
        Line2D([0], [0], marker="o", color="gray", linestyle="",
               markeredgecolor="black", markersize=11, label="Robot (active)"),
        Line2D([0], [0], marker="X", color="gray", linestyle="",
               markeredgecolor="black", markersize=11, label="Robot (failed)"),
    ]
