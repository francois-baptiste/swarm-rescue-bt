"""Minimal Behavior Tree engine, modelled after the node taxonomy used by
BehaviorTree.CPP / Nav2 (nav2_behavior_tree): ControlNode (Sequence/Selector),
ConditionNode, ActionNode. Ticking is synchronous: tick() is called once per
control-loop step and returns SUCCESS / FAILURE / RUNNING.
"""

from enum import Enum


class Status(Enum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"


class Node:
    def __init__(self, name):
        self.name = name

    def tick(self, bb):
        raise NotImplementedError


class Sequence(Node):
    """Ticks children in order. Stops at the first child that is not SUCCESS."""

    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self, bb):
        for child in self.children:
            status = child.tick(bb)
            if status != Status.SUCCESS:
                return status
        return Status.SUCCESS


class Selector(Node):
    """Ticks children in order. Stops at the first child that is not FAILURE."""

    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self, bb):
        for child in self.children:
            status = child.tick(bb)
            if status != Status.FAILURE:
                return status
        return Status.FAILURE


class Condition(Node):
    def __init__(self, name, predicate):
        super().__init__(name)
        self.predicate = predicate

    def tick(self, bb):
        return Status.SUCCESS if self.predicate(bb) else Status.FAILURE


class Action(Node):
    def __init__(self, name, fn):
        super().__init__(name)
        self.fn = fn

    def tick(self, bb):
        return self.fn(bb)
