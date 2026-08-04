"""Decentralized search-and-rescue swarm demo.

3 ground robots ("drones"), each running its own behavior tree, search a grid
for victims and rescue them. There is no central coordinator: task
allocation emerges from robots broadcasting claims over a simulated radio and
each robot independently applying the same rule to what it hears. A robot is
scripted to fail mid-mission to show the swarm re-allocates its work without
any external intervention.

Run:  python3 sim.py         -> saves output/sar_swarm_demo.gif
Run:  python3 sim.py --live  -> also opens an interactive matplotlib window
                                 (map + per-robot BT state + position-vector
                                 time series), requires a display
Produces: output/sar_swarm_demo.gif and a text event log on stdout.
"""

import sys

LIVE = "--live" in sys.argv

import matplotlib
if not LIVE:
    # Agg is headless-safe (no display needed) for GIF-only runs; --live
    # instead leaves the backend unset so matplotlib picks whatever GUI
    # toolkit is installed, which is required for plt.show() to open a window.
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


def _active_bt_path(robot):
    """The branch of the tree that just fired, e.g. "PursueClaim >
    NavigateToPose [RUNNING]", read off py_trees' own current_child chain
    (root.tip()) rather than tracked separately - the tree is the source
    of truth for its own active path."""
    if robot.failed:
        return "FAILED"
    leaf = robot.tree.tip()
    if leaf is None:
        return "-"
    names = []
    node = leaf
    while node is not None:
        names.append(node.name)
        node = node.parent
    names.reverse()
    return " > ".join(names[1:]) + f"  [{leaf.status.name}]"


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
            "bt_active": [_active_bt_path(r) for r in robots],
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


def render(world, robots, frames, gif_path=None, live=False):
    fig = plt.figure(figsize=(13, 7))
    gs = fig.add_gridspec(2, 2, width_ratios=(2, 1), wspace=0.35, hspace=0.45)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_bt = fig.add_subplot(gs[0, 1])
    ax_vec = fig.add_subplot(gs[1, 1])
    grid = np.array(world.grid)

    def draw(i):
        f = frames[i]

        # ---- map: grid, victims, trails, robots, motion vectors ---------
        ax_map.clear()
        ax_map.imshow(1 - grid, cmap="gray", vmin=0, vmax=1, origin="lower",
                      extent=(-0.5, world.width - 0.5, -0.5, world.height - 0.5))

        for vid, v in f["victims"].items():
            x, y = v["pose"]
            if v["rescued"]:
                ax_map.plot(x, y, marker="D", color="limegreen", markersize=10,
                            markeredgecolor="black", zorder=3)
            else:
                ax_map.plot(x, y, marker="x", color="black", markersize=12,
                            markeredgewidth=3, zorder=3)

        for rid, (pos, state, trail) in enumerate(
                zip(f["positions"], f["states"], f["trails"])):
            color = robots[rid].color
            tx = [p[0] for p in trail]
            ty = [p[1] for p in trail]
            ax_map.plot(tx, ty, color=color, alpha=0.25, linewidth=1.5, zorder=1)

            if len(trail) >= 2 and state != "FAILED":
                (px, py), (cx, cy) = trail[-2], trail[-1]
                if (px, py) != (cx, cy):
                    ax_map.annotate("", xy=(cx, cy), xytext=(px, py),
                                    arrowprops=dict(arrowstyle="-|>", color=color, lw=2),
                                    zorder=5)

            marker = "X" if state == "FAILED" else "o"
            ax_map.plot(*pos, marker=marker, color=color, markersize=16,
                        markeredgecolor="black", zorder=4)
            ax_map.annotate(f"R{rid}: {state}", pos, textcoords="offset points",
                            xytext=(10, 8), fontsize=8, color=color, weight="bold")

        ax_map.set_title(f"Decentralized SAR swarm - tick {f['t']}")
        ax_map.set_xlim(-0.5, world.width - 0.5)
        ax_map.set_ylim(-0.5, world.height - 0.5)
        ax_map.set_xticks([])
        ax_map.set_yticks([])

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
        ax_map.legend(handles=legend_handles, loc="upper center",
                      bbox_to_anchor=(0.5, -0.02), ncol=2, fontsize=8, frameon=False)

        # ---- behavior tree state panel -----------------------------------
        ax_bt.clear()
        ax_bt.axis("off")
        ax_bt.set_title("Behavior tree state", fontsize=10, weight="bold", loc="left")
        for rid, path in enumerate(f["bt_active"]):
            ax_bt.text(0.0, 0.9 - 0.12 * rid, f"R{rid}: {path}",
                       color=robots[rid].color, fontsize=9, family="monospace",
                       transform=ax_bt.transAxes, va="top")

        # ---- position vector evolution (x/y over time per robot) --------
        ax_vec.clear()
        ax_vec.set_title("Position vector evolution", fontsize=10, weight="bold", loc="left")
        for rid, trail in enumerate(f["trails"]):
            xs = [p[0] for p in trail]
            ys = [p[1] for p in trail]
            ts = range(len(trail))
            color = robots[rid].color
            ax_vec.plot(ts, xs, color=color, linestyle="-", linewidth=1.2)
            ax_vec.plot(ts, ys, color=color, linestyle="--", linewidth=1.2)
        ax_vec.set_xlim(0, len(frames))
        ax_vec.set_ylim(-0.5, max(world.width, world.height) - 0.5)
        ax_vec.set_xlabel("tick", fontsize=8)
        ax_vec.set_ylabel("grid coordinate", fontsize=8)
        ax_vec.tick_params(labelsize=7)
        style_handles = [
            Line2D([0], [0], color="gray", linestyle="-", label="x"),
            Line2D([0], [0], color="gray", linestyle="--", label="y"),
        ]
        ax_vec.legend(handles=style_handles, loc="upper right", fontsize=7, frameon=False)

    anim = animation.FuncAnimation(fig, draw, frames=len(frames), interval=120)
    if gif_path:
        anim.save(gif_path, writer=animation.PillowWriter(fps=8))
    if live:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    world, bus, robots = build_scenario()

    free = set(world.all_free_cells())
    assert all(r.pos in free for r in robots), "a robot starts on a wall"

    frames = run(world, bus, robots)
    print_log(bus, world)
    print(f"\nSimulation ran {len(frames)} ticks.")

    render(world, robots, frames, gif_path="output/sar_swarm_demo.gif", live=LIVE)
    print("Saved animation to output/sar_swarm_demo.gif")
