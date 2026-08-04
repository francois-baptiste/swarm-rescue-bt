"""Centralized search-and-rescue swarm demo.

N ground robots ("drones"), each running its own behavior tree, search a
grid for victims and rescue them. Perception stays local (a robot only
reports victims within its own sensor_range) but task allocation does
not: every robot reports sightings and a heartbeat to a single
swarm.coordinator.Coordinator, which is the only thing that ever decides
who pursues which victim. SCENARIOS below covers several maps/robot-
victim ratios/failure patterns, all going through the same Coordinator,
so their outcomes can be compared against the decentralized branches' own
SCENARIOS (kept parameter-for-parameter identical apart from the radio-
specific fields those branches have and this one doesn't) to see which
architecture suits which situation.

Run:  python3 sim.py                        -> runs "default", saves a GIF
Run:  python3 sim.py --scenario many_victims -> runs a named scenario
Run:  python3 sim.py --live                 -> also opens an interactive
                                                matplotlib window that
                                                autoplays (map + one
                                                unfolded behavior tree per
                                                robot, color-coded by its
                                                py_trees status), requires
                                                a display
Run:  python3 sim.py --slider               -> opens the same window but
                                                paused, with a tick slider
                                                (and Play/Pause button) to
                                                scrub by hand, requires a
                                                display
Run:  python3 sim.py --compare              -> runs every scenario
                                                headlessly (no GIF/window)
                                                and prints a ticks-to-
                                                complete comparison table;
                                                this is also the project's
                                                end-to-end test
Produces: output/sar_swarm_<scenario>.gif (skipped in --slider/--compare
mode) and a text event log on stdout.
"""

import argparse
import sys

PALETTE = ["#d62728", "#1f77b4", "#2ca02c", "#ff7f0e", "#9467bd",
           "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"]

# Every scenario here has a parameter-identical twin in the decentralized
# branches' sim.py (same map, victims, starts, sensor_range, failures,
# max_ticks - minus the radio_range field those branches have and this
# architecture doesn't need) - that's what makes `--compare`'s ticks-to-
# complete numbers meaningful to compare *across* branches, not just
# within one.
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


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", default="default", choices=list(SCENARIOS),
                         help="which scenario to run (ignored with --compare)")
    parser.add_argument("--live", action="store_true",
                         help="open an interactive matplotlib window instead of only saving a GIF")
    parser.add_argument("--slider", action="store_true",
                         help="open the same window paused, with a tick slider to scrub by hand")
    parser.add_argument("--compare", action="store_true",
                         help="run every scenario headlessly and print a comparison table (no GIF/window)")
    return parser.parse_args()


ARGS = _parse_args()
LIVE, SLIDER, COMPARE, SCENARIO = ARGS.live, ARGS.slider, ARGS.compare, ARGS.scenario

import matplotlib
if not LIVE and not SLIDER:
    # Agg is headless-safe (no display needed) for GIF-only/--compare runs;
    # --live and --slider instead leave the backend unset so matplotlib
    # picks whatever GUI toolkit is installed, required to open a window.
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D
from matplotlib.widgets import Slider, Button
import numpy as np
import py_trees

from swarm.world import World, reachable_cells
from swarm.coordinator import Coordinator
from swarm.robot import Robot

Status = py_trees.common.Status
STATUS_COLORS = {
    Status.SUCCESS: "#2ca02c",
    Status.RUNNING: "#ff7f0e",
    Status.FAILURE: "#d62728",
    Status.INVALID: "#7f7f7f",
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
    n = len(robots)
    fig = plt.figure(figsize=(13, max(7, 2.2 * n)))
    gs = fig.add_gridspec(n, 2, width_ratios=(2, 1), wspace=0.35, hspace=0.6,
                           bottom=0.12 if slider else 0.05)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_trees = [fig.add_subplot(gs[rid, 1]) for rid in range(n)]
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

        ax_map.set_title(f"Centralized SAR swarm ({SCENARIO}) - tick {f['t']}")
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


def compare(names):
    """Runs every named scenario headlessly (no rendering) and prints a
    ticks-to-complete table - this doubles as the project's end-to-end
    test: a scenario that never finishes (DNF) or raises is a real bug,
    not just a slow demo."""
    print(f"{'scenario':16s} {'ticks':>6s} {'rescued':>9s}  outcome")
    print("-" * 50)
    all_ok = True
    for name in names:
        world, coordinator, robots, spec = build_scenario(name)
        frames = run(world, coordinator, robots, spec)
        total = len(world.victims)
        rescued = sum(1 for v in world.victims.values() if v["rescued"])
        ok = rescued == total
        all_ok &= ok
        outcome = "OK" if ok else f"DNF (max_ticks={spec['max_ticks']})"
        print(f"{name:16s} {len(frames):6d} {rescued:4d}/{total:<4d}  {outcome}")
    print()
    print("All scenarios completed." if all_ok else "SOME SCENARIOS DID NOT FINISH - see DNF rows above.")
    return all_ok


if __name__ == "__main__":
    if COMPARE:
        ok = compare(list(SCENARIOS))
        sys.exit(0 if ok else 1)

    world, coordinator, robots, spec = build_scenario(SCENARIO)
    print(f"Scenario {SCENARIO!r}: {spec['description']}")

    frames = run(world, coordinator, robots, spec)
    print_log(coordinator, world)
    print(f"\nSimulation ran {len(frames)} ticks.")

    gif_name = "demo" if SCENARIO == "default" else SCENARIO
    gif_path = None if SLIDER else f"output/sar_swarm_{gif_name}.gif"
    render(world, robots, frames, gif_path=gif_path, live=LIVE, slider=SLIDER)
    if gif_path:
        print(f"Saved animation to {gif_path}")
