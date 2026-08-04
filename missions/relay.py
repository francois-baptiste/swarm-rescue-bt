"""Communication relay: N robots hold position along the path between a
fixed source and a fixed sink so that every consecutive hop (source ->
robot -> robot -> ... -> sink) stays within comm_range, relaying a
message end to end. There is no central coordinator: each robot computes
its own target slot once, from nothing but its own id and the total
robot count, and never revisits that decision.

The cost of that simplicity shows up on failure: if a robot dies, nobody
reflows the remaining robots to close the gap - the chain just breaks
wherever that robot's slot was. See missions/relay.py on the
centralized-swarm branch, where the Coordinator notices and reassigns
slots among the survivors instead.
"""

import math

import py_trees

from swarm.world import World, reachable_cells, bfs_path
from missions.common import tree_snapshot, PALETTE

TITLE = "Decentralized Communication Relay"
Status = py_trees.common.Status

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
                     "everyone's settled into their slot) - since slots are statically assigned, "
                     "nobody closes the resulting gap and the chain breaks.",
        width=15, height=15,
        source=(0, 0), sink=(14, 14),
        starts=[(0, 0), (0, 0), (0, 0)],
        comm_range=6.0,
        failures=[(1, 25)],
        max_ticks=50,
        expect_intact=False,
    ),
}


class NavigateToPose(py_trees.behaviour.Behaviour):
    """Walks toward the robot's assigned slot and then holds there - the
    only leaf in the tree, since there's nothing else to decide: the slot
    never changes once assigned."""

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
    def __init__(self, rid, start, world, slot, color="C0"):
        self.id = rid
        self.pos = start
        self.world = world
        self.slot = slot
        self.color = color

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

    n = len(spec["starts"])
    slots = _pick_slots(backbone, n)

    free = set(world.all_free_cells())
    reach = reachable_cells(world, spec["source"])
    for pos in list(spec["starts"]) + slots + [spec["source"], spec["sink"]]:
        assert pos in free, f"scenario {name!r}: {pos} is on a wall/out of bounds"
        assert pos in reach, f"scenario {name!r}: {pos} is unreachable from {spec['source']}"

    robots = [
        Robot(i, start, world, slots[i], color=PALETTE[i % len(PALETTE)])
        for i, start in enumerate(spec["starts"])
    ]
    extra = {"source": spec["source"], "sink": spec["sink"], "slots": slots,
             "backbone_len": len(backbone), "events": []}
    return world, extra, robots, spec


def run(world, extra, robots, spec):
    """Unlike the rescue mission, relay has no natural end - it always
    runs the full max_ticks."""
    frames = []
    for t in range(spec["max_ticks"]):
        for rid, fail_tick in spec["failures"]:
            if t == fail_tick:
                robots[rid].failed = True
                extra["events"].append((t, rid))

        for r in robots:
            r.tick(t)

        frames.append({
            "t": t,
            "positions": [r.pos for r in robots],
            "states": [r.state for r in robots],
            "trails": [list(r.trail) for r in robots],
            "bt_snapshot": [tree_snapshot(r) for r in robots],
            "source": extra["source"],
            "sink": extra["sink"],
        })
    return frames


def print_log(extra, world):
    print("=== Event log (decentralized: each robot holds a statically-assigned slot) ===")
    print(f"source={extra['source']} sink={extra['sink']} "
          f"backbone={extra['backbone_len']} cells, slots={extra['slots']}")
    for t, rid in extra["events"]:
        print(f"t={t:3d}  robot {rid} FAILS (scripted) - its slot is abandoned, nobody reflows")
    print()
    print("=== Summary ===")


def _chain_gaps(frame):
    chain = [frame["source"]]
    for pos, state in zip(frame["positions"], frame["states"]):
        if state != "FAILED":
            chain.append(pos)
    chain.append(frame["sink"])
    return [math.dist(chain[i], chain[i + 1]) for i in range(len(chain) - 1)]


def summarize(world, extra, spec, frames):
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
