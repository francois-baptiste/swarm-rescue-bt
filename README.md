# swarm-rescue-bt (centralized branch)

Five centralized multi-robot swarm simulations, written in pure Python
with **no ROS2 / Nav2 / Gazebo dependency**. Each one exists to validate a
coordination architecture — behavior trees driving local perception and
navigation, with a single Coordinator making every allocation decision —
before porting it onto a real robotics stack. Compare this to the
`py-trees-port` / `master` branches, where the *same* missions are solved
with no central arbiter at all.

![demo](output/sar_swarm_demo.gif)

## What it demonstrates

The flagship mission (`rescue`, the original demo): three ground robots
start from opposite corners of a 15×15 grid map hiding 4 victims. Each
robot still runs its own behavior tree and still only perceives victims
within its own `sensor_range` — but instead of claiming a sensed victim
for itself, it reports it (and a heartbeat) to a single
`swarm.coordinator.Coordinator`, which is the only thing that ever
decides which robot pursues which victim. At tick 11, robot 1 is scripted
to fail right after being assigned a victim. The Coordinator notices the
silence and reassigns the victim to another robot — the point of *this*
demo is that allocation is simple and globally optimal as long as the
Coordinator is alive, at the cost of it being a single point of failure
for the whole swarm's task allocation.

Four more missions (see [Missions](#missions) below) explore the same
idea in different shapes — patrolling a shared perimeter, holding a
communication relay chain, hunting a moving target, and a two-team
capture-the-flag contest (the one mission where a Coordinator has
nothing to do differently - see its own section below). Every mission
has a parameter-identical twin on the decentralized branches, so the same
scenario can be run on both to see which architecture actually suits it
better, rather than reasoning about it in the abstract.

## How it's structured

- **`missions/<name>.py`** — one file per mission (`rescue`, `patrol`,
  `relay`, `wolfpack`, `capture_flag`), each defining its own Coordinator
  (where it has one), robot class(es), `SCENARIOS` dict,
  `build_scenario`/`run`/`print_log`/`summarize`, and the map decorations
  `sim.py` needs to render it (`draw_extra`/`legend_handles`).
  `missions/common.py` holds the handful of things every mission shares
  (the color palette, the py_trees tree-snapshot helper). `rescue`'s
  Coordinator lives in `swarm/coordinator.py` instead, since it predates
  the mission system and other code still imports it directly.
- Every mission's tree is built with
  [py_trees](https://py-trees.readthedocs.io/) (`Selector`, `Sequence`,
  one `py_trees.behaviour.Behaviour` subclass per leaf), using the same
  node-naming taxonomy BehaviorTree.CPP / Nav2 uses (`nav2_behavior_tree`)
  - `NavigateToPose` means the same thing (BFS-walk one cell toward a
  goal) in every mission, whatever leaf currently owns picking that goal.
- **`swarm/world.py`** — a 2D grid with obstacles and BFS pathfinding,
  standing in for Nav2's global planner + local controller (give a path
  to a goal, advance one cell).
- **`sim.py`** — mission-agnostic driver: CLI parsing, the shared
  render/GIF/`--live`/`--slider`/`--compare` machinery, dispatching to
  whichever `missions/<name>.py` module `--mission` selects.

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

## Missions

`python3 sim.py --mission NAME` picks a mission (default `rescue`);
`--scenario NAME` then picks one of that mission's own scenarios
(`--mission`/`--scenario` accept an invalid name too, and list the valid
ones in their error message). `python3 sim.py --mission NAME --compare`
runs every scenario of that mission headlessly (no GIF/window) and prints
a comparison table - this is also the project's end-to-end test: a
scenario that raises or never finishes (DNF) inside its `max_ticks` is a
real bug. Every scenario's parameters are identical to its decentralized-
branch twin, so `--compare`'s output from both branches is meant to be
read side by side.

### `rescue` (default)

Search a grid for victims and rescue them; a Coordinator greedily matches
each known, unassigned victim to its nearest idle robot. See
[What it demonstrates](#what-it-demonstrates) above.

| scenario | what it changes | what it tests |
| --- | --- | --- |
| `default` | 3 robots, 4 victims, 15×15, one scripted failure | the baseline demo |
| `many_victims` | same robots/map, 9 victims | how the greedy matcher holds up when victims outnumber robots |
| `robot_heavy` | 6 robots, 2 victims | over-provisioning / idle-robot behavior |
| `large_map` | 25×25 map, 5 robots, 7 victims, one failure | scaling to a bigger map and swarm |
| `double_failure` | robots 1 and 0 both fail (ticks 11 and 30) | resilience when the swarm loses most of its capacity and the Coordinator has to reassign twice |
| `short_sensors` | `sensor_range` roughly halved (1.2) | how much perception range matters when the Coordinator can only assign what's been reported |

On the scenarios tried so far, centralized allocation finishes noticeably
faster when the swarm is healthy (`default`: 87 ticks here vs. 139
decentralized) because the Coordinator's greedy matcher is globally
optimal instead of first-detected-first-claimed, but loses that edge fast
under `double_failure` (172 ticks here vs. 144 decentralized) since
losing two of three robots leaves the Coordinator's allocation advantage
with only one robot left to execute on.

### `patrol`

N robots split a shared border loop into contiguous arcs. Unlike the
decentralized branches, the split isn't something each robot computes for
itself: a Coordinator owns the loop and rebalances the arcs across
whichever robots are still heartbeating whenever that set changes.

| scenario | what it changes | what it tests |
| --- | --- | --- |
| `default` | 4 robots patrol a 15×15 border, no failures | full coverage |
| `robot_down` | robot 1 fails almost immediately (tick 3) | the Coordinator notices (after `FAILURE_TIMEOUT`) and rebalances the loop across the 3 survivors - **full coverage recovers** (52/52), unlike the decentralized branch's permanent 43/52 gap |

### `relay`

N robots hold evenly-spaced positions between a fixed source and sink so
every consecutive hop stays within `comm_range`. The Coordinator owns the
slot split and rebalances it across survivors on failure - but reflowing
what N robots covered onto fewer doesn't always fit within `comm_range`.

| scenario | what it changes | what it tests |
| --- | --- | --- |
| `default` | 3 robots hold a chain from corner to corner | the chain stays intact |
| `robot_down` | the middle relay robot fails after settling in | the Coordinator reflows the 2 survivors across the whole path, **shrinking the worst gap from the decentralized branch's 10.0 to 7.07** - better, but still over `comm_range=6.0`, since 2 robots physically can't cover what 3 did |

### `wolfpack`

N robots hunt one evasive prey. Unlike the decentralized branches, a
robot that spots the prey doesn't broadcast to its peers - it reports
straight to a Coordinator, and every robot queries that same Coordinator
for the most recent sighting, with no propagation delay and no "was I in
radio range" question.

| scenario | what it changes | what it tests |
| --- | --- | --- |
| `default` | 3 robots hunt 1 prey, no failures | a straightforward hunt |
| `robot_down` | the robot that first spots the prey fails right after reporting it | the sighting already reached the Coordinator, so the other robots act on it next tick - in practice this mission's numbers end up close to the decentralized branch's, since its generous radio range rarely made propagation delay the bottleneck |

### `capture_flag`

Two teams (red/blue) race to touch the enemy's flag; a robot caught
within `tag_range` of an enemy robot while on that enemy's side of the
map respawns at its own start. Flag positions are common knowledge (no
detection phase), so there's no task to allocate - **this mission has no
Coordinator at all**, and is identical to its decentralized-branch twin
for exactly that reason (same code, verified same outcomes).

| scenario | what it changes | what it tests |
| --- | --- | --- |
| `default` | 3v3, symmetric starts | a fair fight |
| `outnumbered` | red (2) vs blue (5) | more attackers *and* more incidental defenders |

## Why it's centralized

Each mission's Coordinator (`swarm/coordinator.py` for `rescue`, a small
class local to the other `missions/<name>.py` files) is the one process
that holds a global view of the swarm's state and arbitrates who does
what. Robots never talk to each other and never decide allocation for
themselves - remove the Coordinator and every robot still senses,
explores, and moves, but nothing ever gets assigned, reflowed, or
(in `rescue`) rescued. That's the deliberate inverse of the decentralized
branches' "remove any robot and the others keep working unchanged" - see
those branches' READMEs for the trade-off in the other direction, and
`patrol`/`relay` above for what centralization buys back in return: a
dead robot's work doesn't just vanish, it gets reflowed onto whoever's
left, within whatever the survivors can physically cover.

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
