"""Wolf pack: N robots hunt a single evasive prey that flees whichever
robot is nearest once one gets close enough to notice it. There is no
central coordinator: a robot that spots the prey broadcasts the sighting
over the same simulated radio the rescue mission uses, and every robot
- whether it can currently see the prey or not - independently decides to
converge on the most recent sighting it has heard, or fall back to
frontier exploration if no sighting is fresh enough to trust. Unlike the
patrol/relay missions, this is one place decentralized peer-to-peer
sharing is a genuine strength: a robot dying doesn't erase what it
already told everyone else.
"""

import py_trees

from swarm.world import World, reachable_cells, bfs_path
from swarm.comms import Bus
from swarm.robot import PickExploreGoal, NavigateToExploreGoal
from missions.common import tree_snapshot, PALETTE

TITLE = "Decentralized Wolf Pack"
Status = py_trees.common.Status

SIGHTING_TIMEOUT = 10  # ticks: a sighting with no update this long is no longer trusted

SCENARIOS = {
    "default": dict(
        description="3 robots hunt one evasive prey on the 15x15 map - no failures.",
        width=15, height=15,
        starts=[(0, 0), (14, 0), (7, 14)],
        prey_start=(7, 7),
        sensor_range=3.0, awareness_range=5.0, radio_range=30.0,
        failures=[],
        fail_after_first_sighting=False,
        max_ticks=100,
    ),
    "robot_down": dict(
        description="Same as default, but the robot that first spots the prey fails right after "
                     "reporting it - since the sighting was already broadcast, another robot picks "
                     "up the chase from the last-known position instead of losing the trail.",
        width=15, height=15,
        starts=[(0, 0), (14, 0), (7, 14)],
        prey_start=(7, 7),
        sensor_range=3.0, awareness_range=5.0, radio_range=30.0,
        failures=[],
        fail_after_first_sighting=True,  # see run(): scheduled dynamically once known
        max_ticks=100,
    ),
}


class HasLead(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("HasLead")
        self.robot = robot

    def update(self):
        r = self.robot
        if r.known_prey_pos is None:
            return Status.FAILURE
        fresh = (r.t - r.known_prey_seen_tick) < SIGHTING_TIMEOUT
        return Status.SUCCESS if fresh else Status.FAILURE


class DetectPrey(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("DetectPrey")
        self.robot = robot

    def update(self):
        r = self.robot
        px, py = r.prey.pos
        d2 = (px - r.pos[0]) ** 2 + (py - r.pos[1]) ** 2
        return Status.SUCCESS if d2 <= r.sensor_range ** 2 else Status.FAILURE


class ReportSighting(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("ReportSighting")
        self.robot = robot

    def update(self):
        r = self.robot
        r.known_prey_pos = r.prey.pos
        r.known_prey_seen_tick = r.t
        r.bus.broadcast(r.id, r.pos, r.radio_range,
                         {"type": "sighting", "prey_pos": r.prey.pos, "robot_id": r.id}, r.t)
        r.state = f"SPOTTED {r.prey.pos}"
        return Status.SUCCESS


class NavigateToPose(py_trees.behaviour.Behaviour):
    """PursueLead's leaf - BFS-walks one cell toward the most recent known
    prey position. Recomputes every tick since, unlike a static victim,
    the target itself moves."""

    def __init__(self, robot):
        super().__init__("NavigateToPose")
        self.robot = robot

    def update(self):
        r = self.robot
        goal = r.known_prey_pos
        r.path = bfs_path(r.world, r.pos, goal)
        if not r.path or len(r.path) < 2:
            return Status.RUNNING
        r.pos = r.path[1]
        r.visited.add(r.pos)
        r.state = f"-> {goal}"
        return Status.RUNNING


class Robot:
    def __init__(self, rid, start, world, bus, prey, sensor_range, radio_range, color="C0"):
        self.id = rid
        self.pos = start
        self.world = world
        self.bus = bus
        self.prey = prey
        self.sensor_range = sensor_range
        self.radio_range = radio_range
        self.color = color

        self.visited = {start}
        self.path = []
        self.explore_path = []
        self._exploring_frontier = False
        self.known_prey_pos = None
        self.known_prey_seen_tick = -SIGHTING_TIMEOUT
        self.state = "EXPLORE"
        self.failed = False
        self.t = 0
        self.trail = [start]

        self.tree = self._build_tree()

    def _build_tree(self):
        return py_trees.composites.Selector("MissionRoot", memory=False, children=[
            py_trees.composites.Sequence("DetectAndConverge", memory=False, children=[
                DetectPrey(self),
                ReportSighting(self),
                NavigateToPose(self),
            ]),
            py_trees.composites.Sequence("PursueLead", memory=False, children=[
                HasLead(self),
                NavigateToPose(self),
            ]),
            py_trees.composites.Sequence("Explore", memory=False, children=[
                PickExploreGoal(self),
                NavigateToExploreGoal(self),
            ]),
        ])

    def tick(self, t):
        self.t = t
        if self.failed:
            self.state = "FAILED"
            return
        for msg in self.bus.receive(self.pos):
            self._handle_message(msg, t)
        self.tree.tick_once()
        self.visited.add(self.pos)
        self.trail.append(self.pos)

    def _handle_message(self, msg, t):
        if msg["type"] == "sighting" and t > self.known_prey_seen_tick:
            self.known_prey_pos = msg["prey_pos"]
            self.known_prey_seen_tick = t


class Prey:
    """Flees whichever robot is nearest, but only once one is within
    awareness_range - otherwise holds position. Ties broken by fixed
    neighbor iteration order (world.neighbors()), not randomly, so a run
    is fully reproducible."""

    def __init__(self, world, start, awareness_range, move_every=2):
        self.world = world
        self.pos = start
        self.awareness_range = awareness_range
        # Moves once every `move_every` ticks - slower than the robots,
        # who move every tick. Without this handicap, N robots that all
        # independently head for the same "last known position" (no
        # flanking, no lookahead) can lock into a stable swap-cycle with
        # an equal-speed, optimally-evasive prey: the pursuers arrive
        # exactly where the prey *was*, the prey - reacting to their new
        # position - has an equally-good escape square, forever. A slower
        # prey can't sustain that indefinitely: the pursuers keep net
        # closing on the ticks it doesn't move.
        self.move_every = move_every
        self._tick = 0
        self.trail = [start]

    def step(self, robot_positions):
        self._tick += 1
        if self._tick % self.move_every != 0:
            self.trail.append(self.pos)
            return

        def nearest_d2(c):
            return min((rp[0] - c[0]) ** 2 + (rp[1] - c[1]) ** 2 for rp in robot_positions)

        if nearest_d2(self.pos) > self.awareness_range ** 2:
            self.trail.append(self.pos)
            return
        # self.pos (stay put) goes first so a tie between staying and
        # moving favors staying.
        candidates = [self.pos] + list(self.world.neighbors(self.pos))
        self.pos = max(candidates, key=nearest_d2)
        self.trail.append(self.pos)
        self.trail.append(self.pos)


def build_scenario(name):
    spec = SCENARIOS[name]
    world = World(spec["width"], spec["height"])
    prey = Prey(world, spec["prey_start"], spec["awareness_range"])
    bus = Bus()

    free = set(world.all_free_cells())
    reach = reachable_cells(world, spec["starts"][0])
    for pos in list(spec["starts"]) + [spec["prey_start"]]:
        assert pos in free, f"scenario {name!r}: {pos} is on a wall/out of bounds"
        assert pos in reach, f"scenario {name!r}: {pos} is unreachable from {spec['starts'][0]}"

    robots = [
        Robot(i, start, world, bus, prey, spec["sensor_range"], spec["radio_range"],
              color=PALETTE[i % len(PALETTE)])
        for i, start in enumerate(spec["starts"])
    ]

    return world, (bus, prey), robots, spec


def run(world, extra, robots, spec):
    """Stops early once any robot captures the prey (occupies its cell)."""
    bus, prey = extra
    failures = list(spec["failures"])
    triggered = set()
    frames = []
    for t in range(spec["max_ticks"]):
        for rid, fail_tick in failures:
            if t == fail_tick:
                robots[rid].failed = True
                bus.event_log.append((t, rid, {"type": "robot_failure"}))

        for r in robots:
            r.tick(t)
        prey.step([r.pos for r in robots if not r.failed])
        bus.advance_tick()

        # "robot_down" doesn't know in advance which robot will spot the
        # prey first, since that depends on the chase - so it schedules
        # its failure dynamically, one tick after the first sighting, the
        # first time run() actually sees one.
        if spec.get("fail_after_first_sighting") and not triggered:
            for t2, sender, msg in bus.event_log:
                if msg["type"] == "sighting":
                    fail_tick = t2 + 1
                    if fail_tick < spec["max_ticks"]:
                        failures.append((sender, fail_tick))
                    triggered.add(sender)
                    break

        frames.append({
            "t": t,
            "positions": [r.pos for r in robots],
            "states": [r.state for r in robots],
            "trails": [list(r.trail) for r in robots],
            "bt_snapshot": [tree_snapshot(r) for r in robots],
            "prey_pos": prey.pos,
            "prey_trail": list(prey.trail),
            "captured": any(r.pos == prey.pos and not r.failed for r in robots),
        })

        if frames[-1]["captured"]:
            break
    return frames


def print_log(extra, world):
    bus, prey = extra
    print("=== Event log (decentralized: sightings are broadcast, pursuit decisions are local) ===")
    for t, sender, msg in bus.event_log:
        if msg["type"] == "sighting":
            print(f"t={t:3d}  robot {sender} broadcasts: prey spotted at {msg['prey_pos']}")
        elif msg["type"] == "robot_failure":
            print(f"t={t:3d}  robot {sender} FAILS (scripted) - stops moving and broadcasting")
    print()
    print("=== Summary ===")
    print(f"prey final position: {prey.pos}")


def summarize(world, extra, spec, frames):
    ok = frames[-1]["captured"]
    label = "captured" if ok else "prey escaped"
    return ok, label


def draw_extra(ax, frame, world):
    tx = [p[0] for p in frame["prey_trail"]]
    ty = [p[1] for p in frame["prey_trail"]]
    ax.plot(tx, ty, color="black", alpha=0.2, linewidth=1.2, zorder=1)
    marker = "*" if frame["captured"] else "^"
    ax.plot(*frame["prey_pos"], marker=marker, color="black", markersize=14,
            markeredgecolor="yellow", markeredgewidth=1.5, zorder=4)


def legend_handles():
    from matplotlib.lines import Line2D
    return [
        Line2D([0], [0], marker="^", color="black", linestyle="",
               markeredgecolor="yellow", markersize=10, label="Prey"),
        Line2D([0], [0], marker="*", color="black", linestyle="",
               markeredgecolor="yellow", markersize=12, label="Prey (captured)"),
        Line2D([0], [0], marker="o", color="gray", linestyle="",
               markeredgecolor="black", markersize=11, label="Robot (active)"),
        Line2D([0], [0], marker="X", color="gray", linestyle="",
               markeredgecolor="black", markersize=11, label="Robot (failed)"),
    ]
