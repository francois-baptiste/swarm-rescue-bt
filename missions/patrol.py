"""Border patrol: N robots split a shared perimeter loop into contiguous
arcs and each walks its own arc back and forth, forever, with no central
coordinator. Each robot computes its own arc from nothing but its own id
and the total robot count - there is no negotiation and no need for one,
since the split is a fixed function every robot can compute identically.

The cost of that simplicity shows up on failure: if a robot dies, nobody
reassigns its arc - it just goes unpatrolled for the rest of the run. See
missions/patrol.py on the centralized-swarm branch, where the Coordinator
notices and reflows the dead robot's arc across the survivors instead.
"""

import py_trees

from swarm.world import World, reachable_cells, perimeter_cells, bfs_path
from missions.common import tree_snapshot, PALETTE

TITLE = "Decentralized Border Patrol"
Status = py_trees.common.Status

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
        description="Same as default but robot 1 fails at tick 3, almost immediately - since arcs "
                     "are statically assigned, robot 1's arc is never patrolled again (this is the "
                     "expected, demonstrated limitation, not a bug: partial coverage is correct).",
        width=15, height=15,
        starts=[(0, 0), (14, 0), (14, 14), (0, 14)],
        failures=[(1, 3)],
        max_ticks=80,
        expect_full_coverage=False,
    ),
    "double_failure": dict(
        description="Same map, but robots 1 and 2 both fail early (ticks 3 and 30) - roughly half "
                     "the loop goes permanently unpatrolled instead of a quarter.",
        width=15, height=15,
        starts=[(0, 0), (14, 0), (14, 14), (0, 14)],
        failures=[(1, 3), (2, 30)],
        max_ticks=80,
        expect_full_coverage=False,
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


class PickNextWaypoint(py_trees.behaviour.Behaviour):
    """Back-and-forth (ping-pong) along the robot's own arc, not a loop
    wraparound: a wraparound's "last waypoint -> first waypoint" leg could
    BFS-shortcut clean through another robot's arc via the interior,
    which would both look wrong and quietly cover a dead neighbor's
    territory. Ping-ponging only ever asks for the next/previous cell in
    the robot's own list, so it can never leave its own arc."""

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
    """Always RUNNING (patrol never "completes") - arrival is detected next
    tick by PickNextWaypoint, same NavigateToPose-stays-RUNNING-until-a-
    condition-notices-arrival pattern as the rescue mission's NavigateToPose."""

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
    def __init__(self, rid, start, world, route, color="C0"):
        self.id = rid
        self.pos = start
        self.world = world
        self.route = route
        self.route_idx = route.index(start) if start in route else 0
        self.direction = 1
        self.color = color

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
        self.tree.tick_once()
        self.trail.append(self.pos)


def _split_route(loop, start_cell, n):
    """Rotates loop so it starts at start_cell, then splits it into n
    contiguous, near-equal arcs (one per robot), each robot ping-ponging
    within its own arc only (see PickNextWaypoint) - so a dead robot's
    entire arc, not just scattered cells, goes unpatrolled."""
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

    n = len(spec["starts"])
    arcs = _split_route(loop, spec["starts"][0], n)
    robots = [
        Robot(i, start, world, arcs[i], color=PALETTE[i % len(PALETTE)])
        for i, start in enumerate(spec["starts"])
    ]

    return world, {"loop": loop, "events": []}, robots, spec


def run(world, extra, robots, spec):
    """Unlike the rescue mission, patrol has no natural end - it always
    runs the full max_ticks."""
    frames = []
    covered = {r.pos for r in robots}
    for t in range(spec["max_ticks"]):
        for rid, fail_tick in spec["failures"]:
            if t == fail_tick:
                robots[rid].failed = True
                extra["events"].append((t, rid))

        for r in robots:
            r.tick(t)
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


def print_log(extra, world):
    print("=== Event log (decentralized: each robot patrols a statically-assigned arc) ===")
    print(f"Perimeter has {len(extra['loop'])} free cells.")
    for t, rid in extra["events"]:
        print(f"t={t:3d}  robot {rid} FAILS (scripted) - its arc goes unpatrolled from here on")
    print()
    print("=== Summary ===")


def summarize(world, extra, spec, frames):
    covered = frames[-1]["visited"] & set(extra["loop"])
    total = len(extra["loop"])
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
