"""Centralized search & rescue: N robots search a grid for victims and
rescue them. Perception stays local (a robot only reports victims within
its own sensor_range) but task allocation does not: every robot reports
sightings and a heartbeat to a single swarm.coordinator.Coordinator,
which is the only thing that ever decides who pursues which victim.
"""

from matplotlib.lines import Line2D

from swarm.world import World, reachable_cells
from swarm.coordinator import Coordinator
from swarm.robot import Robot
from missions.common import tree_snapshot, PALETTE

TITLE = "Centralized SAR swarm"

# Every scenario here has a parameter-identical twin in the decentralized
# branches' missions/rescue.py (same map, victims, starts, sensor_range,
# failures, max_ticks - minus the radio_range field those branches have
# and this architecture doesn't need) - that's what makes `--compare`'s
# ticks-to-complete numbers meaningful to compare *across* branches, not
# just within one.
SCENARIOS = {
    "default": dict(
        description="3 robots, 4 victims, 15x15 map, one scripted mid-mission failure.",
        width=15, height=15,
        victims=[(12, 12), (2, 12), (10, 3), (3, 3)],
        starts=[(0, 0), (14, 0), (7, 14)],
        sensor_range=2.5,
        failures=[(1, 11)],
        max_ticks=220,
    ),
    "many_victims": dict(
        description="Same 3 robots and map, 9 victims instead of 4 - stresses the assignment matcher.",
        width=15, height=15,
        victims=[(12, 12), (2, 12), (10, 3), (3, 3), (7, 7), (1, 1), (13, 1), (1, 13), (13, 13)],
        starts=[(0, 0), (14, 0), (7, 14)],
        sensor_range=2.5,
        failures=[],
        max_ticks=500,
    ),
    "robot_heavy": dict(
        description="6 robots for only 2 victims on the same map - tests over-provisioning/idle behavior.",
        width=15, height=15,
        victims=[(12, 12), (3, 3)],
        starts=[(0, 0), (14, 0), (7, 14), (14, 14), (0, 14), (7, 0)],
        sensor_range=2.5,
        failures=[],
        max_ticks=220,
    ),
    "large_map": dict(
        description="25x25 map, 5 robots, 7 victims, one scripted failure - tests scaling.",
        width=25, height=25,
        victims=[(20, 20), (4, 20), (16, 5), (5, 5), (12, 12), (20, 4), (4, 12)],
        starts=[(0, 0), (24, 0), (12, 24), (0, 24), (24, 24)],
        sensor_range=3.0,
        failures=[(2, 25)],
        max_ticks=700,
    ),
    "double_failure": dict(
        description="Same as default but two robots fail at different times - tests resilience "
                     "when the swarm loses most of its capacity and the Coordinator has to "
                     "reassign twice.",
        width=15, height=15,
        victims=[(12, 12), (2, 12), (10, 3), (3, 3)],
        starts=[(0, 0), (14, 0), (7, 14)],
        sensor_range=2.5,
        failures=[(1, 11), (0, 30)],
        max_ticks=250,
    ),
    "short_sensors": dict(
        description="Same as default but sensor_range roughly halved (1.2) - perception-limited: "
                     "the Coordinator can only assign victims robots have actually reported.",
        width=15, height=15,
        victims=[(12, 12), (2, 12), (10, 3), (3, 3)],
        starts=[(0, 0), (14, 0), (7, 14)],
        sensor_range=1.2,
        failures=[(1, 11)],
        max_ticks=500,
    ),
}


def build_scenario(name):
    spec = SCENARIOS[name]
    world = World(spec["width"], spec["height"])
    for vid, pos in enumerate(spec["victims"]):
        world.add_victim(vid, pos)

    coordinator = Coordinator(world)
    robots = [
        Robot(i, start, world, coordinator, sensor_range=spec["sensor_range"],
              color=PALETTE[i % len(PALETTE)])
        for i, start in enumerate(spec["starts"])
    ]

    free = set(world.all_free_cells())
    reach = reachable_cells(world, spec["starts"][0])
    for pos in list(spec["starts"]) + list(spec["victims"]):
        assert pos in free, f"scenario {name!r}: {pos} is on a wall/out of bounds"
        assert pos in reach, f"scenario {name!r}: {pos} is unreachable from {spec['starts'][0]}"

    return world, coordinator, robots, spec


def run(world, coordinator, robots, spec):
    """Returns per-tick snapshots for rendering, and stops early once every
    victim is rescued."""
    frames = []
    for t in range(spec["max_ticks"]):
        for rid, fail_tick in spec["failures"]:
            if t == fail_tick:
                robots[rid].failed = True
                coordinator.event_log.append((t, rid, {"type": "robot_failure"}))

        for r in robots:
            r.tick(t)
        coordinator.assign(robots, t)

        frames.append({
            "t": t,
            "positions": [r.pos for r in robots],
            "states": [r.state for r in robots],
            "trails": [list(r.trail) for r in robots],
            "bt_snapshot": [tree_snapshot(r) for r in robots],
            "victims": {vid: dict(v) for vid, v in world.victims.items()},
        })

        if all(v["rescued"] for v in world.victims.values()):
            break
    return frames


def print_log(coordinator, world):
    print("=== Event log (centralized: every allocation decision is the Coordinator's) ===")
    for t, sender, msg in coordinator.event_log:
        if msg["type"] == "assign":
            print(f"t={t:3d}  coordinator assigns robot {msg['robot_id']} to victim {msg['victim_id']}")
        elif msg["type"] == "reassign":
            print(f"t={t:3d}  coordinator revokes robot {msg['robot_id']}'s assignment on victim "
                  f"{msg['victim_id']} (robot went silent) - up for reassignment")
        elif msg["type"] == "rescued":
            print(f"t={t:3d}  robot {msg['robot_id']} reports: victim {msg['victim_id']} rescued")
        elif msg["type"] == "robot_failure":
            print(f"t={t:3d}  robot {sender} FAILS (scripted) - stops moving and reporting")
    print()
    print("=== Summary ===")
    for vid, v in world.victims.items():
        status = f"rescued by robot {v['rescued_by']}" if v["rescued"] else "NOT rescued"
        print(f"victim {vid} @ {v['pose']}: {status}")


def summarize(world, coordinator, spec, frames):
    total = len(world.victims)
    rescued = sum(1 for v in world.victims.values() if v["rescued"])
    ok = rescued == total
    label = f"{rescued:3d}/{total:<3d} rescued"
    return ok, label


def draw_extra(ax, frame, world):
    for vid, v in frame["victims"].items():
        x, y = v["pose"]
        if v["rescued"]:
            ax.plot(x, y, marker="D", color="limegreen", markersize=10,
                    markeredgecolor="black", zorder=3)
        else:
            ax.plot(x, y, marker="x", color="black", markersize=12,
                    markeredgewidth=3, zorder=3)


def legend_handles():
    return [
        Line2D([0], [0], marker="x", color="black", linestyle="",
               markeredgewidth=3, markersize=10, label="Victim (not found)"),
        Line2D([0], [0], marker="D", color="limegreen", linestyle="",
               markeredgecolor="black", markersize=9, label="Victim (rescued)"),
        Line2D([0], [0], marker="o", color="gray", linestyle="",
               markeredgecolor="black", markersize=11, label="Robot (active)"),
        Line2D([0], [0], marker="X", color="gray", linestyle="",
               markeredgecolor="black", markersize=11, label="Robot (failed)"),
    ]
