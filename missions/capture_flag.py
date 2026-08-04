"""Capture the flag: two teams race to reach the enemy's flag. Flag
positions are common knowledge (no detection phase - just BFS straight
there), so there's no task to allocate and nothing for a Coordinator to
decide differently: every robot on both architectures just always heads
for the enemy flag - this mission is identical on the decentralized
branches for exactly that reason, no swarm.coordinator.Coordinator
involved. The only adversarial rule is automatic and needs no BT
decision either - a robot caught within tag_range of an enemy robot while
on that enemy's side of the map is tagged and respawns at its own start,
losing all progress. First robot to touch the enemy flag wins.
"""

import py_trees

from swarm.world import World, reachable_cells, bfs_path
from missions.common import tree_snapshot

TITLE = "Capture the Flag"
Status = py_trees.common.Status

RED_COLORS = ["#d62728", "#ff4500", "#b22222", "#8b0000", "#cd5c5c"]
BLUE_COLORS = ["#1f77b4", "#17becf", "#4682b4", "#000080", "#4169e1"]

SCENARIOS = {
    "default": dict(
        description="3v3 on a 15x15 map, symmetric starts - a fair fight.",
        width=15, height=15,
        red_home=(0, 0), blue_home=(14, 14),
        red_starts=[(0, 0), (0, 1), (1, 0)],
        blue_starts=[(14, 14), (14, 13), (13, 14)],
        tag_range=1.5,
        max_ticks=100,
    ),
    "outnumbered": dict(
        description="Red (2 robots) vs Blue (5 robots) on the same map - Blue has both more "
                     "attackers and more incidental defenders.",
        width=15, height=15,
        red_home=(0, 0), blue_home=(14, 14),
        red_starts=[(0, 0), (0, 1)],
        blue_starts=[(14, 14), (14, 13), (13, 14), (14, 12), (12, 14)],
        tag_range=1.5,
        max_ticks=100,
    ),
}


class NavigateToPose(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("NavigateToPose")
        self.robot = robot

    def update(self):
        r = self.robot
        goal = r.enemy_flag
        if not r.path or r.path[0] != r.pos or r.path[-1] != goal:
            r.path = bfs_path(r.world, r.pos, goal)
        if not r.path or len(r.path) < 2:
            return Status.RUNNING
        r.pos = r.path[1]
        r.path = r.path[1:]
        r.state = f"-> {goal}"
        return Status.RUNNING


class Robot:
    def __init__(self, rid, start, world, team, enemy_flag, color):
        self.id = rid
        self.pos = start
        self.start = start
        self.world = world
        self.team = team
        self.enemy_flag = enemy_flag
        self.color = color

        self.path = []
        self.state = "ATTACK"
        self.failed = False
        self.t = 0
        self.trail = [start]

        self.tree = self._build_tree()

    def _build_tree(self):
        return py_trees.composites.Selector("MissionRoot", memory=False, children=[
            py_trees.composites.Sequence("SeekEnemyFlag", memory=False, children=[
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

    def tag(self):
        self.pos = self.start
        self.path = []
        self.state = "TAGGED"


def _resolve_tags(robots, midline_x, tag_range, events, t):
    """A robot on enemy soil caught within tag_range of any enemy robot
    respawns at its own start - pure world rule, no BT node involved."""
    for r in robots:
        on_enemy_side = (r.team == "red" and r.pos[0] >= midline_x) or \
                         (r.team == "blue" and r.pos[0] < midline_x)
        if not on_enemy_side:
            continue
        for o in robots:
            if o.team == r.team:
                continue
            d2 = (o.pos[0] - r.pos[0]) ** 2 + (o.pos[1] - r.pos[1]) ** 2
            if d2 <= tag_range ** 2:
                r.tag()
                events.append((t, r.id, r.team))
                break


def _check_winner(robots, red_flag, blue_flag):
    for r in robots:
        if r.team == "red" and r.pos == blue_flag:
            return "red", r.id
        if r.team == "blue" and r.pos == red_flag:
            return "blue", r.id
    return None, None


def build_scenario(name):
    spec = SCENARIOS[name]
    world = World(spec["width"], spec["height"])
    midline_x = spec["width"] / 2.0

    free = set(world.all_free_cells())
    reach = reachable_cells(world, spec["red_home"])
    all_positions = (list(spec["red_starts"]) + list(spec["blue_starts"]) +
                      [spec["red_home"], spec["blue_home"]])
    for pos in all_positions:
        assert pos in free, f"scenario {name!r}: {pos} is on a wall/out of bounds"
        assert pos in reach, f"scenario {name!r}: {pos} is unreachable from {spec['red_home']}"

    robots = []
    for i, start in enumerate(spec["red_starts"]):
        robots.append(Robot(f"R{i}", start, world, "red", spec["blue_home"],
                             RED_COLORS[i % len(RED_COLORS)]))
    for i, start in enumerate(spec["blue_starts"]):
        robots.append(Robot(f"B{i}", start, world, "blue", spec["red_home"],
                             BLUE_COLORS[i % len(BLUE_COLORS)]))

    extra = {"red_flag": spec["red_home"], "blue_flag": spec["blue_home"],
              "midline_x": midline_x, "events": []}
    return world, extra, robots, spec


def run(world, extra, robots, spec):
    """Stops early once either team's flag is captured."""
    frames = []
    winner, winner_id = None, None
    for t in range(spec["max_ticks"]):
        for r in robots:
            r.tick(t)
        _resolve_tags(robots, extra["midline_x"], spec["tag_range"], extra["events"], t)
        winner, winner_id = _check_winner(robots, extra["red_flag"], extra["blue_flag"])

        frames.append({
            "t": t,
            "positions": [r.pos for r in robots],
            "states": [r.state for r in robots],
            "trails": [list(r.trail) for r in robots],
            "bt_snapshot": [tree_snapshot(r) for r in robots],
            "red_flag": extra["red_flag"],
            "blue_flag": extra["blue_flag"],
            "midline_x": extra["midline_x"],
            "winner": winner,
        })
        if winner:
            extra["winner"] = (winner, winner_id, t)
            break
    return frames


def print_log(extra, world):
    print("=== Event log (no coordinator on either team: every robot always attacks) ===")
    for t, rid, team in extra["events"]:
        print(f"t={t:3d}  {rid} ({team}) is tagged in enemy territory - respawns at start")
    print()
    print("=== Summary ===")
    if "winner" in extra:
        team, rid, t = extra["winner"]
        print(f"{rid} ({team}) captured the enemy flag at t={t}")
    else:
        print("Neither flag was captured.")


def summarize(world, extra, spec, frames):
    ok = "winner" in extra
    if ok:
        team, rid, t = extra["winner"]
        label = f"{rid} ({team}) wins @ t={t}"
    else:
        label = "no capture"
    return ok, label


def draw_extra(ax, frame, world):
    ax.axvline(frame["midline_x"], color="gray", linestyle="--", linewidth=1, alpha=0.6, zorder=1)
    ax.plot(*frame["red_flag"], marker="P", color="red", markersize=15,
            markeredgecolor="black", zorder=3)
    ax.plot(*frame["blue_flag"], marker="P", color="blue", markersize=15,
            markeredgecolor="black", zorder=3)


def legend_handles():
    from matplotlib.lines import Line2D
    return [
        Line2D([0], [0], marker="P", color="red", linestyle="",
               markeredgecolor="black", markersize=10, label="Red flag"),
        Line2D([0], [0], marker="P", color="blue", linestyle="",
               markeredgecolor="black", markersize=10, label="Blue flag"),
        Line2D([0], [0], marker="o", color="gray", linestyle="",
               markeredgecolor="black", markersize=11, label="Robot"),
    ]
