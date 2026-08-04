"""Small helpers shared by every mission module and by sim.py's generic
renderer, so no mission module needs to import sim.py (which would create
a circular import, since sim.py imports missions.<name>)."""

PALETTE = ["#d62728", "#1f77b4", "#2ca02c", "#ff7f0e", "#9467bd",
           "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"]


def tree_snapshot(robot):
    """(depth, name, status) for every node in robot's tree, read off
    py_trees' own node.status right after this tick - snapshotted per frame
    because by render time the live tree only holds the final tick's state."""
    if robot.failed:
        return None

    def walk(node, depth=0):
        yield (depth, node.name, node.status)
        for child in getattr(node, "children", []):
            yield from walk(child, depth + 1)

    return list(walk(robot.tree))
