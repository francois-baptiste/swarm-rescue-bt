"""One autonomous ground drone. Every robot runs the *same* behavior tree and
makes every decision (explore where? claim this victim? give up a stale
claim?) from purely local information: its own position, its own sensor
readings and whatever it has heard on the radio. There is no supervisor
object anywhere that tells a robot what to do - remove any robot from the
list in sim.py and the other two keep working unchanged.

The tree is built with py_trees (composites.Selector/Sequence, one
behaviour.Behaviour subclass per leaf below). Leaf names mirror the
nav2-port branch's sar_mission_tree.xml (BehaviorTree.CPP MissionRoot) so
both implementations read as the same design even though one drives a
grid BFS and the other drives real Nav2. Each robot builds its own tree
instance and each leaf closes over that one robot, so - unlike a ROS
blackboard - there is no shared state to key by robot id.
"""

import random

import py_trees

from swarm.world import bfs_nearest, bfs_path

CLAIM_TIMEOUT = 15  # ticks: a claim with no progress this long is considered stale

Status = py_trees.common.Status


class HasClaim(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("HasClaim")
        self.robot = robot

    def update(self):
        return Status.SUCCESS if self.robot.claim is not None else Status.FAILURE


class IsAtVictim(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("IsAtVictim")
        self.robot = robot

    def update(self):
        r = self.robot
        at_victim = r.pos == r.world.victims[r.claim]["pose"]
        return Status.SUCCESS if at_victim else Status.FAILURE


class RescueVictim(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("RescueVictim")
        self.robot = robot

    def update(self):
        r = self.robot
        vid = r.claim
        victim = r.world.victims[vid]
        victim["rescued"] = True
        victim["rescued_by"] = r.id
        r.rescued_by_me.append(vid)
        r.bus.broadcast(r.id, r.pos, r.radio_range,
                         {"type": "rescued", "victim_id": vid, "robot_id": r.id}, r.t)
        r.state = f"RESCUED v{vid}"
        r.claim = None
        r.path = []
        return Status.SUCCESS


class NavigateToClaim(py_trees.behaviour.Behaviour):
    """PursueClaim's NavigateToPose leaf: BFS-walks one cell toward the
    claimed victim. Shares its node name with Explore's NavigateToPose
    leaf below - same node type as nav2-port's single NavigateToPose
    action, just wired to a different goal in each Sequence."""

    def __init__(self, robot):
        super().__init__("NavigateToPose")
        self.robot = robot

    def update(self):
        r = self.robot
        victim = r.world.victims.get(r.claim)
        if victim is None or victim["rescued"]:
            r.claim = None
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
        r.state = f"-> v{r.claim}"
        return Status.RUNNING


class DetectVictim(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("DetectVictim")
        self.robot = robot

    def update(self):
        r = self.robot
        best = None
        for vid, v in r.world.victims.items():
            if v["rescued"]:
                continue
            d2 = (v["pose"][0] - r.pos[0]) ** 2 + (v["pose"][1] - r.pos[1]) ** 2
            if d2 > r.sensor_range ** 2:
                continue
            claim = r.known_claims.get(vid)
            if claim is not None:
                claimant, last_seen = claim
                if claimant != r.id and (r.t - last_seen) < CLAIM_TIMEOUT:
                    continue  # someone else actively owns it -> respect the claim
            if best is None or d2 < best[1]:
                best = (vid, d2)
        if best is None:
            return Status.FAILURE
        r._seen_victim_id = best[0]
        return Status.SUCCESS


class ClaimVictim(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__("ClaimVictim")
        self.robot = robot

    def update(self):
        r = self.robot
        vid = r._seen_victim_id
        r.claim = vid
        r.known_claims[vid] = (r.id, r.t)
        r.path = []
        r.bus.broadcast(r.id, r.pos, r.radio_range,
                         {"type": "claim", "victim_id": vid, "robot_id": r.id}, r.t)
        r.state = f"CLAIM v{vid}"
        return Status.SUCCESS


class PickExploreGoal(py_trees.behaviour.Behaviour):
    """Targets the nearest unvisited BFS frontier cell, unlike nav2-port's
    random radius sample - falls back to a random neighbor once the whole
    map has been visited."""

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
    """Explore's NavigateToPose leaf - see NavigateToClaim above."""

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
    def __init__(self, rid, start, world, bus, sensor_range=2.5, radio_range=30.0, color="C0"):
        self.id = rid
        self.pos = start
        self.world = world
        self.bus = bus
        self.sensor_range = sensor_range
        self.radio_range = radio_range
        self.color = color

        self.visited = {start}
        self.path = []
        self.claim = None            # victim id this robot is currently pursuing
        self.known_claims = {}       # vid -> (claimant_id, last_seen_tick)
        self.rescued_by_me = []
        self.state = "EXPLORING"
        self.failed = False
        self.t = 0
        self._seen_victim_id = None
        self.explore_path = []
        self._exploring_frontier = False
        self.trail = [start]

        self.tree = self._build_tree()

    # ---- behavior tree -------------------------------------------------
    def _build_tree(self):
        return py_trees.composites.Selector("MissionRoot", memory=False, children=[
            py_trees.composites.Sequence("RescueIfArrived", memory=False, children=[
                HasClaim(self),
                IsAtVictim(self),
                RescueVictim(self),
            ]),
            py_trees.composites.Sequence("PursueClaim", memory=False, children=[
                HasClaim(self),
                NavigateToClaim(self),
            ]),
            py_trees.composites.Sequence("DetectAndClaim", memory=False, children=[
                DetectVictim(self),
                ClaimVictim(self),
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
        for msg in self.bus.receive(self.pos):
            self._handle_message(msg, t)
        self.tree.tick_once()
        self.trail.append(self.pos)

    def _handle_message(self, msg, t):
        if msg["type"] == "claim":
            vid = msg["victim_id"]
            self.known_claims[vid] = (msg["robot_id"], t)
        elif msg["type"] == "rescued":
            vid = msg["victim_id"]
            if self.claim == vid:
                self.claim = None
                self.path = []
            self.known_claims.pop(vid, None)
