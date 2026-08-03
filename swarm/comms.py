"""Simulated ad-hoc radio: there is no server and no arbiter here.

`Bus` only models physical broadcast propagation for the current simulation
tick (like a real mesh radio, a message only reaches robots within range of
the sender). All decision-making - who claims a victim, who backs off, who
takes over after a failure - happens independently inside each Robot by
looking at what it received. Nothing in this file decides anything.
"""


class Bus:
    """Double-buffered on purpose: anything broadcast during tick t becomes
    readable starting tick t+1. This gives every robot exactly one tick of
    propagation delay regardless of the order robots are processed in, so
    the outcome never depends on iteration order (no simulation artifacts
    disguised as "decentralization")."""

    def __init__(self):
        self.event_log = []   # human-readable log, for the demo narration
        self._current = []    # messages readable during the current tick
        self._next = []       # messages broadcast this tick, readable next tick

    def broadcast(self, sender_id, sender_pos, radio_range, msg, t):
        self._next.append({"pos": sender_pos, "range": radio_range, "msg": msg})
        self.event_log.append((t, sender_id, msg))

    def receive(self, pos):
        """All messages broadcast last tick that reach `pos`."""
        out = []
        x, y = pos
        for m in self._current:
            sx, sy = m["pos"]
            if (sx - x) ** 2 + (sy - y) ** 2 <= m["range"] ** 2:
                out.append(m["msg"])
        return out

    def advance_tick(self):
        self._current = self._next
        self._next = []
