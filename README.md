# swarm-rescue-bt

A decentralized multi-robot search & rescue simulation, written in pure
Python with **no ROS2 / Nav2 / Gazebo dependency**. It exists to validate
a coordination architecture — behavior trees driving fully local decisions,
with no central arbiter — before porting it onto a real robotics stack.

![demo](output/sar_swarm_demo.gif)

## What it demonstrates

Three ground robots start from opposite corners of a 15×15 grid map
hiding 4 victims. Each robot runs its own behavior tree and knows only
its own position, what its sensors currently detect, and what its radio
has received — no robot ever sees the whole board. At tick 11, robot 1
is scripted to fail right after claiming a victim. The other two robots
notice the claim has gone stale and pick up the mission with no outside
intervention, which is the whole point of the demo: coordination that
*emerges* from local rules, not from a scheduler.

## How it's structured

- **`bt/core.py`** — a small hand-rolled behavior tree engine (`Sequence`,
  `Selector`, `Condition`, `Action`), modeled on the taxonomy used by
  BehaviorTree.CPP / Nav2 (`nav2_behavior_tree`). Each robot owns and
  ticks its own tree once per timestep.
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
python3 sim.py
```

This prints a text log of the robots' decisions (who claims what, who
rescues whom, the scripted failure) and writes an animation to
`output/sar_swarm_demo.gif`.

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
   `Explore` / `ClaimAndBroadcast` / `NavigateToClaim` nodes here would
   become BT.CPP nodes calling Nav2's `NavigateToPose` action instead of
   moving a point on a grid.
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
