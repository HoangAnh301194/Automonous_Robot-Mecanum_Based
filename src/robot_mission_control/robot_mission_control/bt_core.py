from enum import Enum

class NodeStatus(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

class Blackboard:
    """
    A simple dictionary-based Blackboard for sharing data between BT nodes.
    """
    def __init__(self):
        self.memory = {}

    def set(self, key, value):
        self.memory[key] = value

    def get(self, key, default=None):
        return self.memory.get(key, default)

class TreeNode:
    def __init__(self, name="Node"):
        self.name = name
        self.status = NodeStatus.FAILURE

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        raise NotImplementedError("Tick must be implemented by subclasses")

    def halt(self, blackboard: Blackboard = None) -> None:
        """Reset node state when preempted by a higher-priority branch or invalidated."""
        self.status = NodeStatus.FAILURE


class Sequence(TreeNode):
    """
    Sequence ticks its children sequentially.
    - If a child returns RUNNING, Sequence returns RUNNING.
    - If a child returns FAILURE, Sequence halts remaining children and returns FAILURE.
    - If all children return SUCCESS, Sequence returns SUCCESS.
    """
    def __init__(self, name="Sequence", children=None):
        super().__init__(name)
        self.children = children if children else []

    def halt(self, blackboard: Blackboard = None) -> None:
        for child in self.children:
            if child.status == NodeStatus.RUNNING:
                child.halt(blackboard)
        self.status = NodeStatus.FAILURE

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        for idx, child in enumerate(self.children):
            status = child.tick(blackboard)
            if status != NodeStatus.SUCCESS:
                for tail in self.children[idx + 1:]:
                    if tail.status == NodeStatus.RUNNING:
                        tail.halt(blackboard)
                self.status = status
                return status
        self.status = NodeStatus.SUCCESS
        return NodeStatus.SUCCESS


class Fallback(TreeNode):
    """
    Fallback (Selector) ticks its children sequentially.
    - If a child returns RUNNING, Fallback halts lower-priority running siblings and returns RUNNING.
    - If a child returns SUCCESS, Fallback halts lower-priority running siblings and returns SUCCESS.
    - If all children return FAILURE, Fallback returns FAILURE.
    This acts as a reactive priority manager with preemption.
    """
    def __init__(self, name="Fallback", children=None):
        super().__init__(name)
        self.children = children if children else []

    def halt(self, blackboard: Blackboard = None) -> None:
        for child in self.children:
            if child.status == NodeStatus.RUNNING:
                child.halt(blackboard)
        self.status = NodeStatus.FAILURE

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        for idx, child in enumerate(self.children):
            status = child.tick(blackboard)
            if status != NodeStatus.FAILURE:
                for j, other in enumerate(self.children):
                    if j != idx and other.status == NodeStatus.RUNNING:
                        other.halt(blackboard)
                self.status = status
                return status
        self.status = NodeStatus.FAILURE
        return NodeStatus.FAILURE


class Condition(TreeNode):
    """
    Condition checks a state on the blackboard.
    Returns SUCCESS if true, FAILURE if false. Never RUNNING.
    """
    def __init__(self, name="Condition"):
        super().__init__(name)


class Action(TreeNode):
    """
    Action performs a task.
    Can return SUCCESS, FAILURE, or RUNNING.
    """
    def __init__(self, name="Action"):
        super().__init__(name)
