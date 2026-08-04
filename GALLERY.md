# Decentralized vs. centralized, side by side

One GIF per mission per architecture. Left column is the
[`py-trees-port`](https://github.com/francois-baptiste/swarm-rescue-bt/tree/py-trees-port)
branch (no coordinator, robots self-organize over a simulated radio);
right column is the
[`centralized-swarm`](https://github.com/francois-baptiste/swarm-rescue-bt/tree/centralized-swarm)
branch (a single Coordinator makes every allocation decision). Each pair
is running the *same* scenario with the *same* parameters, so the
difference you see is purely architectural.

See each branch's own `README.md` for the full writeup — this page is
just the pictures.

## Rescue

Search a grid for victims and rescue them. Robot 1 fails right after
claiming/being assigned a victim; watch who picks up the slack, and how
fast.

| Decentralized | Centralized |
| --- | --- |
| ![rescue decentralized](https://raw.githubusercontent.com/francois-baptiste/swarm-rescue-bt/py-trees-port/output/sar_swarm_demo.gif) | ![rescue centralized](https://raw.githubusercontent.com/francois-baptiste/swarm-rescue-bt/centralized-swarm/output/sar_swarm_demo.gif) |

## Patrol (`robot_down`)

N robots split a border loop into arcs. Robot 1 fails almost immediately.

| Decentralized — static split, permanent gap | Centralized — Coordinator reflows, coverage recovers |
| --- | --- |
| ![patrol decentralized](https://raw.githubusercontent.com/francois-baptiste/swarm-rescue-bt/py-trees-port/output/sar_swarm_patrol_robot_down.gif) | ![patrol centralized](https://raw.githubusercontent.com/francois-baptiste/swarm-rescue-bt/centralized-swarm/output/sar_swarm_patrol_robot_down.gif) |

## Relay (`robot_down`)

N robots hold a chain of positions between a source and a sink. The
middle relay robot fails after settling into its slot.

| Decentralized — chain breaks where the dead robot was | Centralized — Coordinator reflows, gap shrinks but doesn't fully close |
| --- | --- |
| ![relay decentralized](https://raw.githubusercontent.com/francois-baptiste/swarm-rescue-bt/py-trees-port/output/sar_swarm_relay_robot_down.gif) | ![relay centralized](https://raw.githubusercontent.com/francois-baptiste/swarm-rescue-bt/centralized-swarm/output/sar_swarm_relay_robot_down.gif) |

## Wolf pack (`robot_down`)

N robots hunt one evasive prey. The robot that first spots the prey
fails right after reporting it.

| Decentralized — sighting broadcast over radio | Centralized — sighting reported straight to the Coordinator |
| --- | --- |
| ![wolfpack decentralized](https://raw.githubusercontent.com/francois-baptiste/swarm-rescue-bt/py-trees-port/output/sar_swarm_wolfpack_robot_down.gif) | ![wolfpack centralized](https://raw.githubusercontent.com/francois-baptiste/swarm-rescue-bt/centralized-swarm/output/sar_swarm_wolfpack_robot_down.gif) |

## Capture the flag (`default`)

Two teams race to the enemy flag, with auto-tag-back in enemy territory.
No coordinator role either way — flag positions are common knowledge, so
there's nothing to allocate. Included for completeness; expect these two
to look basically the same.

| Decentralized | Centralized |
| --- | --- |
| ![capture the flag decentralized](https://raw.githubusercontent.com/francois-baptiste/swarm-rescue-bt/py-trees-port/output/sar_swarm_capture_flag_default.gif) | ![capture the flag centralized](https://raw.githubusercontent.com/francois-baptiste/swarm-rescue-bt/centralized-swarm/output/sar_swarm_capture_flag_default.gif) |
