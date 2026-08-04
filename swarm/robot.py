"""One autonomous ground drone in the *centralized* variant of this demo
(compare swarm/robot.py on the py-trees-port/master branches, where every
decision is made locally). Perception is still fully local - DetectVictim
below only reports victims within this robot's own sensor_range - but task
allocation is not: a robot never claims a victim for itself or listens to
its peers, it just reports sightings and a heartbeat to a single
swarm/coordinator.py Coordinator every tick. That Coordinator is the only
thing that ever decides which robot pursues which victim, and the only
thing that notices a robot has gone silent and hands its work to someone
else - see swarm/coordinator.py for why that is the opposite trade-off
from the decentralized branch.

The tree is still built with py_trees (composites.Selector/Sequence, one
behaviour.Behaviour subclass per leaf below). Node names mostly match the
decentralized branch's (RescueIfArrived, NavigateToPose, Explore, ...)
except where the leaf's job genuinely changed with the architecture:
HasClaim -> HasAssignment, and DetectAndClaim/ClaimVictim ->
ReportVictims/ReportVictim (a robot now reports what it sees instead of
deciding to claim it).
"""

import random

import py_trees

from swarm.world import bfs_nearest, bfs_path

Status = py_trees.common.Status


class HasAssignment(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("HasAssignment")
        self.robot = robot

    def update(self):
        r = self.robot
        return Status.SUCCESS if r.coordinator.assignment_for(r.id) is not None else Status.FAILURE


class IsAtVictim(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("IsAtVictim")
        self.robot = robot

    def update(self):
        r = self.robot
        vid = r.coordinator.assignment_for(r.id)
        at_victim = r.pos == r.world.victims[vid]["pose"]
        return Status.SUCCESS if at_victim else Status.FAILURE


class RescueVictim(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("RescueVictim")
        self.robot = robot

    def update(self):
        r = self.robot
        vid = r.coordinator.assignment_for(r.id)
        victim = r.world.victims[vid]
        victim["rescued"] = True
        victim["rescued_by"] = r.id
        r.rescued_by_me.append(vid)
        r.coordinator.complete(r.id, vid, r.t)
        r.state = f"RESCUED v{vid}"
        r.path = []
        return Status.SUCCESS


class NavigateToAssignment(py_trees.behaviour.Behaviour):
    """PursueAssignment's NavigateToPose leaf: BFS-walks one cell toward
    the victim the Coordinator assigned this robot. Shares its node name
    with Explore's NavigateToPose leaf below - same node type as
    nav2-port's single NavigateToPose action, just wired to a different
    goal in each Sequence."""

    def __init__(self, robot):
        super().__init__("NavigateToPose")
        self.robot = robot

    def update(self):
        r = self.robot
        vid = r.coordinator.assignment_for(r.id)
        victim = r.world.victims.get(vid) if vid is not None else None
        if victim is None or victim["rescued"]:
            r.path = []
            return Status.FAILURE
        goal = victim["pose"]
        if not r.path or r.path[0] != r.pos or r.path[-1] != goal:
            r.path = bfs_path(r.world, r.pos, goal)
        if not r.path or len(r.path) < 2:
            return Status.RUNNING
        r.pos = r.path[1]
        r.path = r.path[1:]
        r.visited.add(r.pos)
        r.state = f"-> v{vid}"
        return Status.RUNNING


class DetectVictim(py_trees.behaviour.Behaviour):
    """SUCCESS if any unrescued victim is within sensor_range - unlike the
    decentralized branch's DetectVictim, this reports every victim in
    range (not just the nearest) and never checks who else might be
    pursuing one: allocation is the Coordinator's job now, not this
    robot's."""

    def __init__(self, robot):
        super().__init__("DetectVictim")
        self.robot = robot

    def update(self):
        r = self.robot
        sensed = [
            vid for vid, v in r.world.victims.items()
            if not v["rescued"]
            and (v["pose"][0] - r.pos[0]) ** 2 + (v["pose"][1] - r.pos[1]) ** 2 <= r.sensor_range ** 2
        ]
        if not sensed:
            return Status.FAILURE
        r._sensed_victim_ids = sensed
        return Status.SUCCESS


class ReportVictim(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("ReportVictim")
        self.robot = robot

    def update(self):
        r = self.robot
        for vid in r._sensed_victim_ids:
            r.coordinator.report(r.id, vid, r.t)
        r.state = "REPORT " + ",".join(f"v{vid}" for vid in r._sensed_victim_ids)
        return Status.SUCCESS


class PickExploreGoal(py_trees.behaviour.Behaviour):
    """Targets the nearest unvisited BFS frontier cell - exploration stays
    a purely local decision even in the centralized variant, since the
    Coordinator only arbitrates victim assignments, not movement."""

    def __init__(self, robot):
        super().__init__("PickExploreGoal")
        self.robot = robot

    def update(self):
        r = self.robot
        frontier = {c for c in r.world.all_free_cells() if c not in r.visited}
        if frontier:
            path = bfs_nearest(r.world, r.pos, frontier)
            if path and len(path) >= 2:
                r.explore_path = path
                r._exploring_frontier = True
                return Status.SUCCESS
        nbrs = list(r.world.neighbors(r.pos))
        if nbrs:
            r.explore_path = [r.pos, random.choice(nbrs)]
            r._exploring_frontier = False
            return Status.SUCCESS
        return Status.FAILURE


class NavigateToExploreGoal(py_trees.behaviour.Behaviour):
    """Explore's NavigateToPose leaf - see NavigateToAssignment above."""

    def __init__(self, robot):
        super().__init__("NavigateToPose")
        self.robot = robot

    def update(self):
        r = self.robot
        path = r.explore_path
        if not path or len(path) < 2:
            return Status.FAILURE
        r.pos = path[1]
        r.visited.add(r.pos)
        r.state = "EXPLORE" if r._exploring_frontier else "PATROL"
        return Status.RUNNING


class Robot:
    def __init__(self, rid, start, world, coordinator, sensor_range=2.5, color="C0"):
        self.id = rid
        self.pos = start
        self.world = world
        self.coordinator = coordinator
        self.sensor_range = sensor_range
        self.color = color

        self.visited = {start}
        self.path = []
        self.rescued_by_me = []
        self.state = "EXPLORING"
        self.failed = False
        self.t = 0
        self._sensed_victim_ids = []
        self.explore_path = []
        self._exploring_frontier = False
        self.trail = [start]

        self.tree = self._build_tree()

    # ---- behavior tree -------------------------------------------------
    def _build_tree(self):
        return py_trees.composites.Selector("MissionRoot", memory=False, children=[
            py_trees.composites.Sequence("RescueIfArrived", memory=False, children=[
                HasAssignment(self),
                IsAtVictim(self),
                RescueVictim(self),
            ]),
            py_trees.composites.Sequence("PursueAssignment", memory=False, children=[
                HasAssignment(self),
                NavigateToAssignment(self),
            ]),
            py_trees.composites.Sequence("ReportVictims", memory=False, children=[
                DetectVictim(self),
                ReportVictim(self),
            ]),
            py_trees.composites.Sequence("Explore", memory=False, children=[
                PickExploreGoal(self),
                NavigateToExploreGoal(self),
            ]),
        ])

    # ---- per-tick entry point -------------------------------------------
    def tick(self, t):
        self.t = t
        if self.failed:
            self.state = "FAILED"
            return
        self.coordinator.heartbeat(self.id, t)
        self.tree.tick_once()
        self.trail.append(self.pos)
