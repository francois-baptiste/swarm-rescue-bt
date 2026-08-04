"""Communication relay: N robots hold position along the path between a
fixed source and a fixed sink so that every consecutive hop (source ->
robot -> robot -> ... -> sink) stays within comm_range, relaying a
message end to end. Unlike the decentralized branches, slot assignment
isn't something each robot computes for itself: a single Coordinator owns
the backbone path and the slot split, and rebalances it across whichever
robots are still heartbeating whenever that set changes - closing the gap
left by a dead robot instead of leaving it broken, PROVIDED enough
robots are still alive to physically span the distance within
comm_range; reflowing 3 robots' worth of coverage onto 2 doesn't always
fit.
"""

import math

import py_trees

from swarm.world import World, reachable_cells, bfs_path
from missions.common import tree_snapshot, PALETTE

TITLE = "Centralized Communication Relay"
Status = py_trees.common.Status

FAILURE_TIMEOUT = 15  # ticks of silence before the Coordinator drops a robot from the split

SCENARIOS = {
    "default": dict(
        description="3 robots deploy from the source corner to hold evenly-spaced slots along "
                     "the source->sink path - no failures, chain stays intact.",
        width=15, height=15,
        source=(0, 0), sink=(14, 14),
        starts=[(0, 0), (0, 0), (0, 0)],
        comm_range=6.0,
        failures=[],
        max_ticks=50,
        expect_intact=True,
    ),
    "robot_down": dict(
        description="Same as default but the middle relay robot fails at tick 25 (after "
                     "everyone's settled into their slot) - the Coordinator notices and "
                     "reflows the 2 survivors across the whole path, though 2 robots spanning "
                     "what 3 used to cover may still exceed comm_range.",
        width=15, height=15,
        source=(0, 0), sink=(14, 14),
        starts=[(0, 0), (0, 0), (0, 0)],
        comm_range=6.0,
        failures=[(1, 25)],
        max_ticks=50,
        expect_intact=False,
    ),
    "double_failure": dict(
        description="Same map, but 2 of the 3 relay robots fail (ticks 25 and 35) - the "
                     "Coordinator reflows down to the 1 survivor, who alone can't come close to "
                     "spanning the whole path within comm_range.",
        width=15, height=15,
        source=(0, 0), sink=(14, 14),
        starts=[(0, 0), (0, 0), (0, 0)],
        comm_range=6.0,
        failures=[(1, 25), (2, 35)],
        max_ticks=60,
        expect_intact=False,
    ),
    "long_chain": dict(
        description="25x25 map, 5 robots relay corner to corner - tests scaling to a longer path "
                     "and a bigger swarm.",
        width=25, height=25,
        source=(0, 0), sink=(24, 24),
        starts=[(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)],
        comm_range=9.0,
        failures=[],
        max_ticks=60,
        expect_intact=True,
    ),
}


class Coordinator:
    def __init__(self, backbone):
        self.backbone = backbone
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
        slots = _pick_slots(self.backbone, len(alive_ids))
        for rid, slot in zip(alive_ids, slots):
            by_id[rid].slot = slot
        self.events.append((t, len(alive_ids), alive_ids))


class NavigateToPose(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("NavigateToPose")
        self.robot = robot

    def update(self):
        r = self.robot
        goal = r.slot
        if not r.path or r.path[0] != r.pos or r.path[-1] != goal:
            r.path = bfs_path(r.world, r.pos, goal)
        if r.path and len(r.path) >= 2:
            r.pos = r.path[1]
            r.path = r.path[1:]
            r.state = f"-> {goal}"
        else:
            r.state = "HOLD"
        return Status.RUNNING


class Robot:
    def __init__(self, rid, start, world, coordinator, color="C0"):
        self.id = rid
        self.pos = start
        self.world = world
        self.coordinator = coordinator
        self.color = color

        self.slot = start
        self.path = []
        self.state = "DEPLOY"
        self.failed = False
        self.t = 0
        self.trail = [start]

        self.tree = self._build_tree()

    def _build_tree(self):
        return py_trees.composites.Selector("MissionRoot", memory=False, children=[
            py_trees.composites.Sequence("HoldRelaySlot", memory=False, children=[
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


def _pick_slots(path, n):
    """n evenly-spaced interior points along path (excluding its source/
    sink endpoints), one per robot, in path order."""
    last = len(path) - 1
    idxs = [round((i + 1) * last / (n + 1)) for i in range(n)]
    return [path[i] for i in idxs]


def build_scenario(name):
    spec = SCENARIOS[name]
    world = World(spec["width"], spec["height"])
    backbone = bfs_path(world, spec["source"], spec["sink"])
    assert backbone is not None, f"scenario {name!r}: source/sink not connected"

    free = set(world.all_free_cells())
    reach = reachable_cells(world, spec["source"])
    for pos in list(spec["starts"]) + [spec["source"], spec["sink"]]:
        assert pos in free, f"scenario {name!r}: {pos} is on a wall/out of bounds"
        assert pos in reach, f"scenario {name!r}: {pos} is unreachable from {spec['source']}"

    coordinator = Coordinator(backbone)
    robots = [
        Robot(i, start, world, coordinator, color=PALETTE[i % len(PALETTE)])
        for i, start in enumerate(spec["starts"])
    ]
    for r in robots:
        coordinator.heartbeat(r.id, 0)
    coordinator.rebalance(robots, 0)

    return world, coordinator, robots, spec


def run(world, coordinator, robots, spec):
    """Unlike the rescue mission, relay has no natural end - it always
    runs the full max_ticks."""
    frames = []
    for t in range(spec["max_ticks"]):
        for rid, fail_tick in spec["failures"]:
            if t == fail_tick:
                robots[rid].failed = True

        for r in robots:
            r.tick(t)
        coordinator.rebalance(robots, t)

        frames.append({
            "t": t,
            "positions": [r.pos for r in robots],
            "states": [r.state for r in robots],
            "trails": [list(r.trail) for r in robots],
            "bt_snapshot": [tree_snapshot(r) for r in robots],
            "source": spec["source"],
            "sink": spec["sink"],
        })
    return frames


def print_log(coordinator, world):
    print("=== Event log (centralized: the Coordinator owns the slot split) ===")
    print(f"backbone={len(coordinator.backbone)} cells")
    for t, n, alive_ids in coordinator.events:
        print(f"t={t:3d}  coordinator splits the path into {n} slot(s) among robots {list(alive_ids)}")
    print()
    print("=== Summary ===")


def _chain_gaps(frame):
    chain = [frame["source"]]
    for pos, state in zip(frame["positions"], frame["states"]):
        if state != "FAILED":
            chain.append(pos)
    chain.append(frame["sink"])
    return [math.dist(chain[i], chain[i + 1]) for i in range(len(chain) - 1)]


def summarize(world, coordinator, spec, frames):
    max_gap = max(_chain_gaps(frames[-1]))
    intact = max_gap <= spec["comm_range"]
    ok = intact if spec["expect_intact"] else True
    label = f"max link {max_gap:5.2f} (comm_range={spec['comm_range']:.1f})"
    return ok, label


def draw_extra(ax, frame, world):
    ax.plot(*frame["source"], marker="s", color="royalblue", markersize=13,
            markeredgecolor="black", zorder=3)
    ax.plot(*frame["sink"], marker="s", color="darkorchid", markersize=13,
            markeredgecolor="black", zorder=3)

    chain = [frame["source"]]
    for pos, state in zip(frame["positions"], frame["states"]):
        if state != "FAILED":
            chain.append(pos)
    chain.append(frame["sink"])
    xs = [c[0] for c in chain]
    ys = [c[1] for c in chain]
    ax.plot(xs, ys, color="black", linestyle=":", linewidth=1.5, zorder=2)


def legend_handles():
    from matplotlib.lines import Line2D
    return [
        Line2D([0], [0], marker="s", color="royalblue", linestyle="",
               markeredgecolor="black", markersize=10, label="Source"),
        Line2D([0], [0], marker="s", color="darkorchid", linestyle="",
               markeredgecolor="black", markersize=10, label="Sink"),
        Line2D([0], [0], color="black", linestyle=":", label="Relay chain (alive links only)"),
        Line2D([0], [0], marker="o", color="gray", linestyle="",
               markeredgecolor="black", markersize=11, label="Robot (active)"),
        Line2D([0], [0], marker="X", color="gray", linestyle="",
               markeredgecolor="black", markersize=11, label="Robot (failed)"),
    ]
