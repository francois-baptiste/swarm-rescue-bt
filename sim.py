"""Centralized multi-robot swarm demos, driven by behavior trees.

Every mission below (where it's meaningful - see missions/capture_flag.py
for the exception) is solved through a single central Coordinator: robots
report what they sense and a heartbeat, and the Coordinator is the only
thing that ever decides who does what - compare this to the
`py-trees-port`/`master` branches, where the same missions are solved
with no central arbiter at all.

Run:  python3 sim.py                                  -> "rescue"/"default", saves a GIF
Run:  python3 sim.py --mission patrol                  -> a different mission's default scenario
Run:  python3 sim.py --mission rescue --scenario many_victims  -> a named scenario
Run:  python3 sim.py --live                            -> also opens an interactive
                                                           matplotlib window that
                                                           autoplays (map + one
                                                           unfolded behavior tree per
                                                           robot, color-coded by its
                                                           py_trees status), requires
                                                           a display
Run:  python3 sim.py --slider                          -> opens the same window but
                                                           paused, with a tick slider
                                                           (and Play/Pause button) to
                                                           scrub by hand, requires a
                                                           display
Run:  python3 sim.py --compare                         -> runs every scenario of the
                                                           current mission headlessly
                                                           (no GIF/window) and prints a
                                                           comparison table; this is
                                                           also the project's
                                                           end-to-end test
Produces: output/sar_<mission>_<scenario>.gif (skipped in --slider/--compare
mode) and a text event log on stdout.

Missions (see missions/<name>.py for each one's own docstring):
  rescue        - search a grid for victims and rescue them (the original demo)
  patrol        - walk a shared border loop, split into per-robot arcs, reflowed on failure
  relay         - hold a chain of positions to relay a message source -> sink, reflowed on failure
  wolfpack      - converge on and capture a moving, evasive prey
  capture_flag  - two teams race to the enemy flag, with auto-tag-back in enemy territory
                  (no Coordinator role here - see missions/capture_flag.py)
"""

import argparse
import importlib
import sys

MISSIONS = ["rescue", "patrol", "relay", "wolfpack", "capture_flag"]


def _parse_args():
    # --mission has to be known before we can validate --scenario against
    # that mission's own SCENARIOS dict, so parse it in a first pass.
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--mission", default="rescue", choices=MISSIONS)
    pre_args, _ = pre.parse_known_args()
    mission = importlib.import_module(f"missions.{pre_args.mission}")

    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mission", default="rescue", choices=MISSIONS,
                         help="which mission to run")
    parser.add_argument("--scenario", default="default", choices=list(mission.SCENARIOS),
                         help=f"which {pre_args.mission!r} scenario to run (ignored with --compare)")
    parser.add_argument("--live", action="store_true",
                         help="open an interactive matplotlib window instead of only saving a GIF")
    parser.add_argument("--slider", action="store_true",
                         help="open the same window paused, with a tick slider to scrub by hand")
    parser.add_argument("--compare", action="store_true",
                         help="run every scenario headlessly and print a comparison table (no GIF/window)")
    args = parser.parse_args()
    return args, mission


ARGS, MISSION = _parse_args()
LIVE, SLIDER, COMPARE, SCENARIO = ARGS.live, ARGS.slider, ARGS.compare, ARGS.scenario

import matplotlib
if not LIVE and not SLIDER:
    # Agg is headless-safe (no display needed) for GIF-only/--compare runs;
    # --live and --slider instead leave the backend unset so matplotlib
    # picks whatever GUI toolkit is installed, required to open a window.
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button
import numpy as np
import py_trees

Status = py_trees.common.Status
STATUS_COLORS = {
    Status.SUCCESS: "#2ca02c",
    Status.RUNNING: "#ff7f0e",
    Status.FAILURE: "#d62728",
    Status.INVALID: "#7f7f7f",
}


def _label(robot):
    """Most missions give robots a plain int id (0, 1, 2, ...), display
    as "R0"/"R1"/...; capture_flag's ids are already prefixed strings
    ("R0", "B0", ...) since two teams share one robot list."""
    return robot.id if isinstance(robot.id, str) else f"R{robot.id}"


def render(world, robots, frames, gif_path=None, live=False, slider=False):
    n = len(robots)
    fig = plt.figure(figsize=(13, max(7, 2.2 * n)))
    gs = fig.add_gridspec(n, 2, width_ratios=(2, 1), wspace=0.35, hspace=0.6,
                           bottom=0.12 if slider else 0.05)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_trees = [fig.add_subplot(gs[rid, 1]) for rid in range(n)]
    grid = np.array(world.grid)
    legend_handles = MISSION.legend_handles()

    def draw(i):
        f = frames[i]

        # ---- map: grid, mission-specific markers, trails, robots --------
        ax_map.clear()
        ax_map.imshow(1 - grid, cmap="gray", vmin=0, vmax=1, origin="lower",
                      extent=(-0.5, world.width - 0.5, -0.5, world.height - 0.5))

        MISSION.draw_extra(ax_map, f, world)

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
            ax_map.annotate(f"{_label(robots[rid])}: {state}", pos, textcoords="offset points",
                            xytext=(10, 8), fontsize=8, color=color, weight="bold")

        ax_map.set_title(f"{MISSION.TITLE} ({SCENARIO}) - tick {f['t']}")
        ax_map.set_xlim(-0.5, world.width - 0.5)
        ax_map.set_ylim(-0.5, world.height - 0.5)
        ax_map.set_xticks([])
        ax_map.set_yticks([])
        ax_map.legend(handles=legend_handles, loc="upper center",
                      bbox_to_anchor=(0.5, -0.02), ncol=2, fontsize=8, frameon=False)

        # ---- behavior tree panel, one per robot, fully unfolded ---------
        for rid, ax in enumerate(ax_trees):
            ax.clear()
            ax.axis("off")
            color = robots[rid].color
            ax.set_title(f"{_label(robots[rid])} behavior tree", fontsize=9, weight="bold",
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
    comparison table - this doubles as the project's end-to-end test: a
    scenario that never finishes (DNF) or raises is a real bug, not just
    a slow demo."""
    print(f"{'scenario':16s} {'ticks':>6s}  outcome")
    print("-" * 60)
    all_ok = True
    for name in names:
        world, extra, robots, spec = MISSION.build_scenario(name)
        frames = MISSION.run(world, extra, robots, spec)
        ok, label = MISSION.summarize(world, extra, spec, frames)
        all_ok &= ok
        outcome = label if ok else f"{label}  DNF (max_ticks={spec['max_ticks']})"
        print(f"{name:16s} {len(frames):6d}  {outcome}")
    print()
    print("All scenarios completed." if all_ok else "SOME SCENARIOS DID NOT FINISH - see DNF rows above.")
    return all_ok


if __name__ == "__main__":
    if COMPARE:
        ok = compare(list(MISSION.SCENARIOS))
        sys.exit(0 if ok else 1)

    world, extra, robots, spec = MISSION.build_scenario(SCENARIO)
    print(f"Mission {ARGS.mission!r}, scenario {SCENARIO!r}: {spec['description']}")

    frames = MISSION.run(world, extra, robots, spec)
    MISSION.print_log(extra, world)
    print(f"\nSimulation ran {len(frames)} ticks.")

    gif_name = "demo" if (ARGS.mission == "rescue" and SCENARIO == "default") else f"{ARGS.mission}_{SCENARIO}"
    gif_path = None if SLIDER else f"output/sar_swarm_{gif_name}.gif"
    render(world, robots, frames, gif_path=gif_path, live=LIVE, slider=SLIDER)
    if gif_path:
        print(f"Saved animation to {gif_path}")
