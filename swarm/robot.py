"""One autonomous ground drone. Every robot runs the *same* behavior tree and
makes every decision (explore where? claim this victim? give up a stale
claim?) from purely local information: its own position, its own sensor
readings and whatever it has heard on the radio. There is no supervisor
object anywhere that tells a robot what to do - remove any robot from the
list in sim.py and the other two keep working unchanged.
"""

import random

from bt import Action, Condition, Selector, Sequence, Status
from swarm.world import bfs_nearest, bfs_path

CLAIM_TIMEOUT = 15  # ticks: a claim with no progress this long is considered stale


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
        # Leaf names mirror the nav2-port branch's sar_mission_tree.xml
        # (BehaviorTree.CPP MissionRoot) so both implementations read as
        # the same design even though one drives a grid BFS and the other
        # drives real Nav2.
        return Selector("MissionRoot", [
            Sequence("RescueIfArrived", [
                Condition("HasClaim", lambda r: r.claim is not None),
                Condition("IsAtVictim", lambda r: r.pos == r.world.victims[r.claim]["pose"]),
                Action("RescueVictim", self._act_rescue_victim),
            ]),
            Sequence("PursueClaim", [
                Condition("HasClaim", lambda r: r.claim is not None),
                Action("NavigateToPose", self._act_navigate_to_claim),
            ]),
            Sequence("DetectAndClaim", [
                Condition("DetectVictim", self._cond_detect_victim),
                Action("ClaimVictim", self._act_claim_victim),
            ]),
            Sequence("Explore", [
                Action("PickExploreGoal", self._act_pick_explore_goal),
                Action("NavigateToPose", self._act_navigate_to_explore_goal),
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
        self.tree.tick(self)
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

    # ---- BT leaves --------------------------------------------------------
    def _cond_detect_victim(self, r):
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
            return False
        r._seen_victim_id = best[0]
        return True

    def _act_claim_victim(self, r):
        vid = r._seen_victim_id
        r.claim = vid
        r.known_claims[vid] = (r.id, r.t)
        r.path = []
        r.bus.broadcast(r.id, r.pos, r.radio_range,
                         {"type": "claim", "victim_id": vid, "robot_id": r.id}, r.t)
        r.state = f"CLAIM v{vid}"
        return Status.SUCCESS

    def _act_navigate_to_claim(self, r):
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

    def _act_rescue_victim(self, r):
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

    # PickExploreGoal + NavigateToPose mirror the nav2-port Explore Sequence
    # (PickExploreGoalAction samples a random point, NavigateToPose drives
    # to it); here PickExploreGoal instead targets the nearest unvisited
    # frontier cell via BFS, falling back to a random neighbor once the
    # whole map has been visited.
    def _act_pick_explore_goal(self, r):
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

    def _act_navigate_to_explore_goal(self, r):
        path = r.explore_path
        if not path or len(path) < 2:
            return Status.FAILURE
        r.pos = path[1]
        r.visited.add(r.pos)
        r.state = "EXPLORE" if r._exploring_frontier else "PATROL"
        return Status.RUNNING
