"""Decentralized search-and-rescue swarm demo.

3 ground robots ("drones"), each running its own behavior tree, search a grid
for victims and rescue them. There is no central coordinator: task
allocation emerges from robots broadcasting claims over a simulated radio and
each robot independently applying the same rule to what it hears. A robot is
scripted to fail mid-mission to show the swarm re-allocates its work without
any external intervention.

Run:  python3 sim.py
Produces: output/sar_swarm_demo.gif and a text event log on stdout.
"""

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D
import numpy as np

from swarm.world import World
from swarm.comms import Bus
from swarm.robot import Robot

MAX_TICKS = 220
FAIL_ROBOT_ID = 1
FAIL_TICK = 11  # fails right after claiming a victim, before rescuing it -
                # forces another robot to detect the stale claim and take over
SENSOR_RANGE = 2.5
RADIO_RANGE = 30.0  # effectively whole-map mesh; decentralization is about
                     # *who decides*, not physical radio reach - see README.


def build_scenario():
    world = World(15, 15)
    for vid, pos in enumerate([(12, 12), (2, 12), (10, 3), (3, 3)]):
        world.add_victim(vid, pos)

    bus = Bus()
    starts = [(0, 0), (14, 0), (7, 14)]
    colors = ["#d62728", "#1f77b4", "#2ca02c"]
    robots = [
        Robot(i, starts[i], world, bus, sensor_range=SENSOR_RANGE,
              radio_range=RADIO_RANGE, color=colors[i])
        for i in range(3)
    ]
    return world, bus, robots


def run(world, bus, robots):
    """Returns per-tick snapshots for rendering, and stops early once every
    victim is rescued."""
    frames = []
    for t in range(MAX_TICKS):
        if t == FAIL_TICK:
            robots[FAIL_ROBOT_ID].failed = True
            bus.event_log.append((t, FAIL_ROBOT_ID, {"type": "robot_failure"}))

        for r in robots:
            r.tick(t)
        bus.advance_tick()

        frames.append({
            "t": t,
            "positions": [r.pos for r in robots],
            "states": [r.state for r in robots],
            "trails": [list(r.trail) for r in robots],
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


def render(world, frames, gif_path):
    fig, ax = plt.subplots(figsize=(7, 7))
    grid = np.array(world.grid)

    def draw(i):
        ax.clear()
        ax.imshow(1 - grid, cmap="gray", vmin=0, vmax=1, origin="lower",
                   extent=(-0.5, world.width - 0.5, -0.5, world.height - 0.5))
        f = frames[i]

        for vid, v in f["victims"].items():
            x, y = v["pose"]
            if v["rescued"]:
                ax.plot(x, y, marker="D", color="limegreen", markersize=10,
                        markeredgecolor="black", zorder=3)
            else:
                ax.plot(x, y, marker="x", color="black", markersize=12,
                        markeredgewidth=3, zorder=3)

        for rid, (pos, state, trail) in enumerate(
                zip(f["positions"], f["states"], f["trails"])):
            color = robots[rid].color
            tx = [p[0] for p in trail]
            ty = [p[1] for p in trail]
            ax.plot(tx, ty, color=color, alpha=0.25, linewidth=1.5, zorder=1)
            marker = "X" if state == "FAILED" else "o"
            ax.plot(*pos, marker=marker, color=color, markersize=16,
                     markeredgecolor="black", zorder=4)
            ax.annotate(f"R{rid}: {state}", pos, textcoords="offset points",
                        xytext=(10, 8), fontsize=8, color=color, weight="bold")

        ax.set_title(f"Decentralized SAR swarm - tick {f['t']}")
        ax.set_xlim(-0.5, world.width - 0.5)
        ax.set_ylim(-0.5, world.height - 0.5)
        ax.set_xticks([])
        ax.set_yticks([])

        legend_handles = [
            Line2D([0], [0], marker="x", color="black", linestyle="",
                   markeredgewidth=3, markersize=10, label="Victim (not found)"),
            Line2D([0], [0], marker="D", color="limegreen", linestyle="",
                   markeredgecolor="black", markersize=9, label="Victim (rescued)"),
            Line2D([0], [0], marker="o", color="gray", linestyle="",
                   markeredgecolor="black", markersize=11, label="Robot (active)"),
            Line2D([0], [0], marker="X", color="gray", linestyle="",
                   markeredgecolor="black", markersize=11, label="Robot (failed)"),
        ]
        ax.legend(handles=legend_handles, loc="upper center",
                  bbox_to_anchor=(0.5, -0.02), ncol=2, fontsize=8, frameon=False)

    anim = animation.FuncAnimation(fig, draw, frames=len(frames), interval=120)
    anim.save(gif_path, writer=animation.PillowWriter(fps=8))
    plt.close(fig)


if __name__ == "__main__":
    world, bus, robots = build_scenario()

    free = set(world.all_free_cells())
    assert all(r.pos in free for r in robots), "a robot starts on a wall"

    frames = run(world, bus, robots)
    print_log(bus, world)
    print(f"\nSimulation ran {len(frames)} ticks.")

    render(world, frames, "output/sar_swarm_demo.gif")
    print("Saved animation to output/sar_swarm_demo.gif")
