"""Centralized search-and-rescue swarm demo.

3 ground robots ("drones"), each running its own behavior tree, search a grid
for victims and rescue them. Perception stays local (a robot only reports
victims within its own sensor_range) but task allocation does not: every
robot reports sightings and a heartbeat to a single swarm.coordinator.
Coordinator, which is the only thing that ever decides who pursues which
victim. A robot is scripted to fail mid-mission to show the Coordinator
notices the silence and reassigns its victim - compare this to the
decentralized branch, where the *robots* notice a stale claim; here losing
the Coordinator itself would stop task allocation entirely.

Run:  python3 sim.py           -> saves output/sar_swarm_demo.gif
Run:  python3 sim.py --live    -> also opens an interactive matplotlib window
                                   that autoplays (map + one unfolded
                                   behavior tree per robot, every node
                                   color-coded by its py_trees status),
                                   requires a display
Run:  python3 sim.py --slider  -> opens the same window but paused, with a
                                   tick slider (and Play/Pause button) to
                                   scrub back and forth through the run by
                                   hand, requires a display
Produces: output/sar_swarm_demo.gif (skipped in --slider mode) and a text
event log on stdout.
"""

import sys

LIVE = "--live" in sys.argv
SLIDER = "--slider" in sys.argv

import matplotlib
if not LIVE and not SLIDER:
    # Agg is headless-safe (no display needed) for GIF-only runs; --live and
    # --slider instead leave the backend unset so matplotlib picks whatever
    # GUI toolkit is installed, which is required to open a window.
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D
from matplotlib.widgets import Slider, Button
import numpy as np
import py_trees

from swarm.world import World
from swarm.coordinator import Coordinator
from swarm.robot import Robot

Status = py_trees.common.Status
STATUS_COLORS = {
    Status.SUCCESS: "#2ca02c",
    Status.RUNNING: "#ff7f0e",
    Status.FAILURE: "#d62728",
    Status.INVALID: "#7f7f7f",
}

MAX_TICKS = 220
FAIL_ROBOT_ID = 1
FAIL_TICK = 11  # fails right after being assigned a victim, before rescuing it -
                # forces the Coordinator to notice the silence and reassign it
SENSOR_RANGE = 2.5


def build_scenario():
    world = World(15, 15)
    for vid, pos in enumerate([(12, 12), (2, 12), (10, 3), (3, 3)]):
        world.add_victim(vid, pos)

    coordinator = Coordinator(world)
    starts = [(0, 0), (14, 0), (7, 14)]
    colors = ["#d62728", "#1f77b4", "#2ca02c"]
    robots = [
        Robot(i, starts[i], world, coordinator, sensor_range=SENSOR_RANGE, color=colors[i])
        for i in range(3)
    ]
    return world, coordinator, robots


def _tree_snapshot(robot):
    """(depth, name, status) for every node in robot's tree, read off
    py_trees' own node.status right after this tick - snapshotted per frame
    because by render time the live tree only holds the final tick's state."""
    if robot.failed:
        return None

    def walk(node, depth=0):
        yield (depth, node.name, node.status)
        for child in getattr(node, "children", []):
            yield from walk(child, depth + 1)

    return list(walk(robot.tree))


def run(world, coordinator, robots):
    """Returns per-tick snapshots for rendering, and stops early once every
    victim is rescued."""
    frames = []
    for t in range(MAX_TICKS):
        if t == FAIL_TICK:
            robots[FAIL_ROBOT_ID].failed = True
            coordinator.event_log.append((t, FAIL_ROBOT_ID, {"type": "robot_failure"}))

        for r in robots:
            r.tick(t)
        coordinator.assign(robots, t)

        frames.append({
            "t": t,
            "positions": [r.pos for r in robots],
            "states": [r.state for r in robots],
            "trails": [list(r.trail) for r in robots],
            "bt_snapshot": [_tree_snapshot(r) for r in robots],
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


def render(world, robots, frames, gif_path=None, live=False, slider=False):
    fig = plt.figure(figsize=(13, 9))
    gs = fig.add_gridspec(3, 2, width_ratios=(2, 1), wspace=0.35, hspace=0.6,
                           bottom=0.12 if slider else 0.05)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_trees = [fig.add_subplot(gs[rid, 1]) for rid in range(len(robots))]
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

        ax_map.set_title(f"Centralized SAR swarm - tick {f['t']}")
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

        # ---- behavior tree panel, one per robot, fully unfolded ---------
        for rid, ax in enumerate(ax_trees):
            ax.clear()
            ax.axis("off")
            color = robots[rid].color
            ax.set_title(f"R{rid} behavior tree", fontsize=9, weight="bold",
                         loc="left", color=color)

            snapshot = f["bt_snapshot"][rid]
            if snapshot is None:
                ax.text(0.0, 0.85, "FAILED", color=color, fontsize=9,
                        family="monospace", weight="bold",
                        transform=ax.transAxes, va="top")
                continue

            y = 0.95
            dy = 0.9 / max(len(snapshot), 1)
            for depth, name, status in snapshot:
                ax.text(0.0, y, f"{'  ' * depth}{name}  [{status.name}]",
                        color=STATUS_COLORS.get(status, "black"), fontsize=6.5,
                        family="monospace", transform=ax.transAxes, va="top")
                y -= dy

    if slider:
        _run_interactive(fig, draw, len(frames))
        return

    anim = animation.FuncAnimation(fig, draw, frames=len(frames), interval=120)
    if gif_path:
        anim.save(gif_path, writer=animation.PillowWriter(fps=8))
    if live:
        plt.show()
    plt.close(fig)


def _run_interactive(fig, draw, n_frames):
    """Tick slider + Play/Pause button, wired to the same draw(i) used by the
    GIF/--live animation - scrubbing calls draw() exactly like a frame tick
    would, so both paths render identically."""
    slider_ax = fig.add_axes((0.15, 0.045, 0.55, 0.03))
    tick_slider = Slider(slider_ax, "tick", 0, n_frames - 1, valinit=0, valstep=1)

    play_ax = fig.add_axes((0.75, 0.035, 0.08, 0.05))
    play_button = Button(play_ax, "Play")
    playing = {"on": False}

    def on_slider(val):
        draw(int(tick_slider.val))
        fig.canvas.draw_idle()

    def toggle_play(event):
        playing["on"] = not playing["on"]
        play_button.label.set_text("Pause" if playing["on"] else "Play")

    def advance():
        if playing["on"]:
            nxt = int(tick_slider.val) + 1
            if nxt >= n_frames:
                playing["on"] = False
                play_button.label.set_text("Play")
                return
            tick_slider.set_val(nxt)  # triggers on_slider -> redraw

    tick_slider.on_changed(on_slider)
    play_button.on_clicked(toggle_play)

    timer = fig.canvas.new_timer(interval=120)
    timer.add_callback(advance)
    timer.start()

    draw(0)
    plt.show()


if __name__ == "__main__":
    world, coordinator, robots = build_scenario()

    free = set(world.all_free_cells())
    assert all(r.pos in free for r in robots), "a robot starts on a wall"

    frames = run(world, coordinator, robots)
    print_log(coordinator, world)
    print(f"\nSimulation ran {len(frames)} ticks.")

    gif_path = None if SLIDER else "output/sar_swarm_demo.gif"
    render(world, robots, frames, gif_path=gif_path, live=LIVE, slider=SLIDER)
    if gif_path:
        print(f"Saved animation to {gif_path}")
