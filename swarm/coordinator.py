"""Centralized task allocator - the mirror image of the decentralized
branch's swarm/comms.py Bus, which deliberately arbitrates nothing.

Robots still sense locally (DetectVictim in swarm/robot.py only reports
victims within this robot's own sensor_range - perception stays
distributed), but they no longer decide anything for themselves: every
robot reports what it sees and sends a heartbeat to this single
Coordinator each tick, and the Coordinator is the only place that decides
who pursues which victim, and the only place that notices a robot has
gone silent and hands its work to someone else. Remove the Coordinator
and every robot just explores forever with no way to know a victim
exists or that another robot already has one - the opposite of the
decentralized branch's "remove any robot and the others keep working
unchanged".
"""

FAILURE_TIMEOUT = 15  # ticks of silence before a robot's assignment is revoked


class Coordinator:
    def __init__(self, world):
        self.world = world
        self.known_victim_ids = set()   # victims reported by at least one robot
        self.assignments = {}           # robot_id -> victim_id
        self.event_log = []             # (t, "coordinator"/robot_id, msg) - demo narration
        self._last_seen = {}            # robot_id -> tick of its last report/heartbeat

    def heartbeat(self, robot_id, t):
        self._last_seen[robot_id] = t

    def report(self, robot_id, victim_id, t):
        self.known_victim_ids.add(victim_id)
        self._last_seen[robot_id] = t

    def assignment_for(self, robot_id):
        return self.assignments.get(robot_id)

    def complete(self, robot_id, victim_id, t):
        self.assignments.pop(robot_id, None)
        self.event_log.append(
            (t, robot_id, {"type": "rescued", "victim_id": victim_id, "robot_id": robot_id}))

    def assign(self, robots, t):
        """Run once per tick, after every robot has sensed and moved:
        revoke assignments held by robots that stopped reporting/
        heartbeating (failed), then greedily match each still-unassigned,
        known, unrescued victim to its nearest idle robot - closest
        (robot, victim) pair first, so several simultaneous detections
        don't all pile onto the same nearest robot."""
        for rid, vid in list(self.assignments.items()):
            if t - self._last_seen.get(rid, t) > FAILURE_TIMEOUT:
                del self.assignments[rid]
                self.event_log.append(
                    (t, "coordinator",
                     {"type": "reassign", "victim_id": vid, "robot_id": rid}))

        by_id = {r.id: r for r in robots}
        idle_ids = {r.id for r in robots if not r.failed} - set(self.assignments)
        candidates = {
            vid for vid in self.known_victim_ids
            if not self.world.victims[vid]["rescued"] and vid not in self.assignments.values()
        }

        while idle_ids and candidates:
            best = None
            for rid in idle_ids:
                px, py = by_id[rid].pos
                for vid in candidates:
                    vx, vy = self.world.victims[vid]["pose"]
                    d2 = (vx - px) ** 2 + (vy - py) ** 2
                    if best is None or d2 < best[0]:
                        best = (d2, rid, vid)
            _, rid, vid = best
            self.assignments[rid] = vid
            idle_ids.discard(rid)
            candidates.discard(vid)
            self.event_log.append(
                (t, "coordinator", {"type": "assign", "victim_id": vid, "robot_id": rid}))
