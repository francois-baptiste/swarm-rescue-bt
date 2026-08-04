# swarm-rescue-bt (centralized branch)

A centralized multi-robot search & rescue simulation, written in pure
Python with **no ROS2 / Nav2 / Gazebo dependency**. It exists to validate a
coordination architecture — behavior trees driving local perception and
navigation, with a single Coordinator making every task-allocation
decision — before porting it onto a real robotics stack. Compare this to
the `py-trees-port` / `master` branches, where the *same* scenario is
solved with no central arbiter at all.

![demo](output/sar_swarm_demo.gif)

## What it demonstrates

Three ground robots start from opposite corners of a 15×15 grid map
hiding 4 victims. Each robot still runs its own behavior tree and still
only perceives victims within its own `sensor_range` — but instead of
claiming a sensed victim for itself, it reports it (and a heartbeat) to a
single `swarm.coordinator.Coordinator`, which is the only thing that ever
decides which robot pursues which victim. At tick 11, robot 1 is scripted
to fail right after being assigned a victim. The Coordinator notices the
silence and reassigns the victim to another robot — the point of *this*
demo is that allocation is simple and globally optimal as long as the
Coordinator is alive, at the cost of it being a single point of failure
for the whole swarm's task allocation.

`sim.py`'s `SCENARIOS` dict has several more maps/robot-victim
ratios/failure patterns (see [Scenarios](#scenarios) below) - each one has
a parameter-identical twin on the decentralized branches, so the same
scenario can be run on both to see which architecture actually suits it
better, rather than reasoning about it in the abstract.

## How it's structured

- **`swarm/coordinator.py`** — the Coordinator: an authoritative registry
  of which victims are known and who is assigned to which, a heartbeat
  timeout that reassigns a silent robot's victim, and a greedy
  nearest-(robot, victim)-pair matcher run once per tick after every robot
  has sensed and moved.
- **`swarm/robot.py`**'s leaf classes and `Robot._build_tree()` — the tree
  is built with [py_trees](https://py-trees.readthedocs.io/) (`Selector`,
  `Sequence`, one `py_trees.behaviour.Behaviour` subclass per leaf).
  `DetectVictim`/`ReportVictim` replace the decentralized branches'
  `DetectVictim`/`ClaimVictim` — a robot reports what it senses instead of
  deciding to claim it — and `HasAssignment`/`PursueAssignment` replace
  `HasClaim`/`PursueClaim`. `Explore` is unchanged: movement stays a
  purely local decision even here.
- **`swarm/world.py`** — a 2D grid with obstacles and BFS pathfinding,
  standing in for Nav2's global planner + local controller (give a path
  to a goal, advance one cell).
- **`sim.py`** — the scenario described above, wiring the three robots,
  the world, and the Coordinator together and rendering the run.

## Running it

```bash
pip install -r requirements.txt
python3 sim.py
```

This prints a text log of the Coordinator's decisions (who it assigns to
what, who it reassigns and why, who reports a rescue) and writes an
animation to `output/sar_swarm_demo.gif`. The animation is a dashboard:
the map (with a motion arrow per robot) plus each robot's full behavior
tree, every node color-coded by its live py_trees status.

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
| `many_victims` | same robots/map, 9 victims | how the greedy matcher holds up when victims outnumber robots |
| `robot_heavy` | 6 robots, 2 victims | over-provisioning / idle-robot behavior |
| `large_map` | 25×25 map, 5 robots, 7 victims, one failure | scaling to a bigger map and swarm |
| `double_failure` | robots 1 and 0 both fail (ticks 11 and 30) | resilience when the swarm loses most of its capacity and the Coordinator has to reassign twice |
| `short_sensors` | `sensor_range` roughly halved (1.2) | how much perception range matters when the Coordinator can only assign what's been reported |

`python3 sim.py --compare` runs every scenario headlessly (no GIF/window)
and prints a ticks-to-complete table - this is also the project's
end-to-end test: a scenario that raises or never finishes (DNF) inside its
`max_ticks` is a real bug. Every scenario's parameters are identical to
its decentralized-branch twin, so `--compare`'s output from both branches
is meant to be read side by side, not just within one branch. On the
scenarios tried so far, centralized allocation finishes noticeably faster
when the swarm is healthy (`default`: 87 ticks here vs. 139 decentralized)
because the Coordinator's greedy matcher is globally optimal instead of
first-detected-first-claimed, but loses that edge fast under
`double_failure` (172 ticks here vs. 144 decentralized) since losing two
of three robots leaves the Coordinator's allocation advantage with only
one robot left to execute on.

## Why it's centralized

`swarm/coordinator.py`'s `Coordinator` is the one process that holds a
global view of every known victim and every assignment, and the one place
that arbitrates conflicts and detects a silent robot. Robots never talk to
each other and never decide who pursues what — remove the Coordinator and
every robot still senses and explores, but nothing ever gets assigned or
rescued. That's the deliberate inverse of the decentralized branches'
"remove any robot and the others keep working unchanged" — see those
branches' READMEs for the trade-off in the other direction.

## Toward a real multi-robot Nav2 stack

This prototype simplifies the same two things the decentralized branches
do, for the same reason (stay readable):

1. **Navigation** — BFS on a known grid stands in for Nav2
   (`bt_navigator` + `planner_server` + `controller_server`). On a real
   stack, each robot would run its own Nav2 instance in its own ROS2
   namespace, and `PickExploreGoal` / `NavigateToPose` here would become
   BT.CPP nodes calling Nav2's `NavigateToPose` action instead of moving a
   point on a grid.
2. **Uplink** — a robot's report/heartbeat to the Coordinator is modeled
   as an instantaneous method call, with no propagation delay (unlike the
   decentralized branches' one-tick radio latency). On a real stack this
   would be a ROS2 topic or service call to a fleet-manager node, which
   does have latency — the Coordinator's `FAILURE_TIMEOUT` would need to
   account for that the same way `CLAIM_TIMEOUT` does on the decentralized
   branches.

---

*[Version française](README.fr.md)*
