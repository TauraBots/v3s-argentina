from __future__ import annotations
from typing import Dict
from .behavior import Behavior
from core.types import Robot, Ball

class BehaviorManager:
    """
    Mantém um dicionário {robot_id: behavior}.
    """
    def __init__(self):
        self._behaviors: Dict[int, Behavior] = {}

    def set_behavior(self, robot_id: int, behavior: Behavior) -> None:
        self._behaviors[robot_id] = behavior

    def step_all(self, robots: list[Robot], ball: Ball | None) -> dict[int, tuple[float, float]]:
        cmds: dict[int, tuple[float, float]] = {}
        for r in robots:
            beh = self._behaviors.get(r.id)
            if beh:
                cmds[r.id] = beh.step(r, ball)
        return cmds
