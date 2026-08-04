# swarm-rescue-bt

A decentralized multi-robot search & rescue simulation, written in pure
Python with **no ROS2 / Nav2 / Gazebo dependency**. It exists to validate
a coordination architecture — behavior trees driving fully local decisions,
with no central arbiter — before porting it onto a real robotics stack.

![demo](output/sar_swarm_demo.gif)

## What it demonstrates

The default scenario: three ground robots start from opposite corners of
a 15×15 grid map hiding 4 victims. Each robot runs its own behavior tree
and knows only its own position, what its sensors currently detect, and
what its radio has received — no robot ever sees the whole board. At tick
11, robot 1 is scripted to fail right after claiming a victim. The other
two robots notice the claim has gone stale and pick up the mission with
no outside intervention, which is the whole point of the demo:
coordination that *emerges* from local rules, not from a scheduler.

`sim.py`'s `SCENARIOS` dict has several more maps/robot-victim
ratios/failure patterns (see [Scenarios](#scenarios) below) - each one has
a parameter-identical twin on the `centralized-swarm` branch, so the same
scenario can be run on both to see which architecture actually suits it
better, rather than reasoning about it in the abstract.

## How it's structured

- **`swarm/robot.py`**'s leaf classes and `Robot._build_tree()` — the tree is
  built with [py_trees](https://py-trees.readthedocs.io/) (`Selector`,
  `Sequence`, one `py_trees.behaviour.Behaviour` subclass per leaf), using
  the same node names as the taxonomy BehaviorTree.CPP / Nav2 uses
  (`nav2_behavior_tree`). Each robot owns and ticks its own tree once per
  timestep.
- **`swarm/world.py`** — a 2D grid with obstacles and BFS pathfinding,
  standing in for Nav2's global planner + local controller (give a path
  to a goal, advance one cell).
- **`swarm/comms.py`** — a simulated radio bus: broadcast with limited
  range and one tick of latency. It never makes decisions — it only
  propagates messages.
- **`swarm/robot.py`** — a robot is a behavior tree plus purely local
  state (position, what it has seen, what it has heard). Coordination
  emerges from two rules: "claim what you can see that isn't already
  actively claimed" and "a claim with no update for 15 ticks is
  considered stale" — a simplified, fault-tolerant contract-net protocol.
- **`sim.py`** — the scenario described above, wiring the three robots,
  the world, and the radio bus together and rendering the run.

## Running it

```bash
pip install -r requirements.txt
python3 sim.py
```

This prints a text log of the robots' decisions (who claims what, who
rescues whom, the scripted failure) and writes an animation to
`output/sar_swarm_demo.gif`. The animation is a dashboard: the map (with
a motion arrow per robot) plus each robot's full behavior tree, every
node color-coded by its live py_trees status.

Add `--live` to open an interactive matplotlib window that autoplays
instead of only saving the GIF, or `--slider` for the same window paused
with a tick slider and Play/Pause button to scrub by hand (both require a
display):

```bash
python3 sim.py --live
python3 sim.py --slider
```

## Scenarios

`python3 sim.py --scenario NAME` runs any of the following instead of the
default (`--scenario` accepts an invalid name too, and lists the valid
ones in its error message):

| name | what it changes | what it tests |
| --- | --- | --- |
| `default` | 3 robots, 4 victims, 15×15, one scripted failure | the baseline demo above |
| `many_victims` | same robots/map, 9 victims | claim contention when victims outnumber robots |
| `robot_heavy` | 6 robots, 2 victims | over-provisioning / idle-robot behavior |
| `large_map` | 25×25 map, 5 robots, 7 victims, one failure | scaling to a bigger map and swarm |
| `double_failure` | robots 1 and 0 both fail (ticks 11 and 64) | resilience when the swarm loses most of its capacity |
| `short_sensors` | `sensor_range` roughly halved (1.2) | how much perception range matters when claiming is peer-arbitrated |

`python3 sim.py --compare` runs every scenario headlessly (no GIF/window)
and prints a ticks-to-complete table - this is also the project's
end-to-end test: a scenario that raises or never finishes (DNF) inside its
`max_ticks` is a real bug.

## Why it's decentralized

No process holds a global view of claims or arbitrates conflicts. Each
robot only knows its own position, what its `sensor_range` detects, and
what the radio has delivered to it. Removing a robot from the list in
`sim.py` doesn't break the other two — the scripted failure of robot 1
demonstrates exactly that within the scenario itself.

## Toward a real multi-robot Nav2 stack

This prototype simplifies two things to stay readable:

1. **Navigation** — BFS on a known grid stands in for Nav2
   (`bt_navigator` + `planner_server` + `controller_server`). On a real
   stack, each robot would run its own Nav2 instance in its own ROS2
   namespace (`/robot_0/...`, `/robot_1/...`), and the
   `PickExploreGoal` / `ClaimVictim` / `NavigateToPose` nodes here would
   become BT.CPP nodes calling Nav2's `NavigateToPose` action instead of
   moving a point on a grid — see the `nav2-port` branch, which does
   exactly this with the same BT leaf names.
2. **Radio** — `radio_range` is deliberately generous (close to the
   whole map) to keep things simple. On ROS2 this would be a DDS topic
   (`/swarm/claims`) with best-effort QoS — DDS natively handles
   peer-to-peer discovery with no central master, which matches this
   prototype's "no arbiter" assumption exactly.

The claim / timeout / recovery logic itself carries over unchanged — it's
the part that actually validates decentralized coordination, independent
of whatever navigation stack sits underneath.

---

*[Version française](README.fr.md)*
