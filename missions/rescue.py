"""Decentralized search & rescue: N robots search a grid for victims and
rescue them. There is no central coordinator: task allocation emerges
from robots broadcasting claims over a simulated radio and each robot
independently applying the same rule to what it hears.
"""

from matplotlib.lines import Line2D

from swarm.world import World, reachable_cells
from swarm.comms import Bus
from swarm.robot import Robot
from missions.common import tree_snapshot, PALETTE

TITLE = "Decentralized SAR swarm"

# Every scenario here has a parameter-identical twin in the centralized
# branch's missions/rescue.py (same map, victims, starts, sensor_range,
# failures, max_ticks) - that's what makes `--compare`'s ticks-to-complete
# numbers meaningful to compare *across* branches, not just within one.
SCENARIOS = {
    "default": dict(
        description="3 robots, 4 victims, 15x15 map, one scripted mid-mission failure.",
        width=15, height=15,
        victims=[(12, 12), (2, 12), (10, 3), (3, 3)],
        starts=[(0, 0), (14, 0), (7, 14)],
        sensor_range=2.5, radio_range=30.0,
        failures=[(1, 11)],
        max_ticks=220,
    ),
    "many_victims": dict(
        description="Same 3 robots and map, 9 victims instead of 4 - stresses claim contention.",
        width=15, height=15,
        victims=[(12, 12), (2, 12), (10, 3), (3, 3), (7, 7), (1, 1), (13, 1), (1, 13), (13, 13)],
        starts=[(0, 0), (14, 0), (7, 14)],
        sensor_range=2.5, radio_range=30.0,
        failures=[],
        max_ticks=500,
    ),
    "robot_heavy": dict(
        description="6 robots for only 2 victims on the same map - tests over-provisioning/idle behavior.",
        width=15, height=15,
        victims=[(12, 12), (3, 3)],
        starts=[(0, 0), (14, 0), (7, 14), (14, 14), (0, 14), (7, 0)],
        sensor_range=2.5, radio_range=30.0,
        failures=[],
        max_ticks=220,
    ),
    "large_map": dict(
        description="25x25 map, 5 robots, 7 victims, one scripted failure - tests scaling.",
        width=25, height=25,
        victims=[(20, 20), (4, 20), (16, 5), (5, 5), (12, 12), (20, 4), (4, 12)],
        starts=[(0, 0), (24, 0), (12, 24), (0, 24), (24, 24)],
        sensor_range=3.0, radio_range=40.0,
        failures=[(2, 25)],
        max_ticks=700,
    ),
    "double_failure": dict(
        description="Same as default but robots 1 and 0 both fail (at 11 and 64) - only robot 2 "
                     "is left to finish alone, testing resilience when the swarm loses most of "
                     "its capacity.",
        width=15, height=15,
        victims=[(12, 12), (2, 12), (10, 3), (3, 3)],
        starts=[(0, 0), (14, 0), (7, 14)],
        sensor_range=2.5, radio_range=30.0,
        failures=[(1, 11), (0, 64)],
        max_ticks=250,
    ),
    "short_sensors": dict(
        description="Same as default but sensor_range roughly halved (1.2) - perception-limited.",
        width=15, height=15,
        victims=[(12, 12), (2, 12), (10, 3), (3, 3)],
        starts=[(0, 0), (14, 0), (7, 14)],
        sensor_range=1.2, radio_range=30.0,
        failures=[(1, 11)],
        max_ticks=500,
    ),
}


def build_scenario(name):
    spec = SCENARIOS[name]
    world = World(spec["width"], spec["height"])
    for vid, pos in enumerate(spec["victims"]):
        world.add_victim(vid, pos)

    bus = Bus()
    robots = [
        Robot(i, start, world, bus, sensor_range=spec["sensor_range"],
              radio_range=spec["radio_range"], color=PALETTE[i % len(PALETTE)])
        for i, start in enumerate(spec["starts"])
    ]

    free = set(world.all_free_cells())
    reach = reachable_cells(world, spec["starts"][0])
    for pos in list(spec["starts"]) + list(spec["victims"]):
        assert pos in free, f"scenario {name!r}: {pos} is on a wall/out of bounds"
        assert pos in reach, f"scenario {name!r}: {pos} is unreachable from {spec['starts'][0]}"

    return world, bus, robots, spec


def run(world, bus, robots, spec):
    """Returns per-tick snapshots for rendering, and stops early once every
    victim is rescued."""
    frames = []
    for t in range(spec["max_ticks"]):
        for rid, fail_tick in spec["failures"]:
            if t == fail_tick:
                robots[rid].failed = True
                bus.event_log.append((t, rid, {"type": "robot_failure"}))

        for r in robots:
            r.tick(t)
        bus.advance_tick()

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


def print_log(bus, world):
    print("=== Event log (decentralized: each line is one robot's own local decision) ===")
    for t, sender, msg in bus.event_log:
        if msg["type"] == "claim":
            print(f"t={t:3d}  robot {sender} broadcasts: claiming victim {msg['victim_id']}")
        elif msg["type"] == "rescued":
            print(f"t={t:3d}  robot {sender} broadcasts: victim {msg['victim_id']} rescued")
        elif msg["type"] == "robot_failure":
            print(f"t={t:3d}  robot {sender} FAILS (scripted) - stops moving and broadcasting")
    print()
    print("=== Summary ===")
    for vid, v in world.victims.items():
        status = f"rescued by robot {v['rescued_by']}" if v["rescued"] else "NOT rescued"
        print(f"victim {vid} @ {v['pose']}: {status}")


def summarize(world, bus, spec, frames):
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
