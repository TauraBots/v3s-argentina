from .behavior import Behavior
from core.motion_controller import DifferentialController
from core.types import Robot, Ball

class GotoBall(Behavior):
    """
    Aproxime o robô até parar a `stop_distance` da bola.
    """

    def __init__(self, controller: DifferentialController | None = None):
        self.controller = controller or DifferentialController()

    def step(self, robot: Robot, ball: Ball | None) -> tuple[float, float]:
        if ball is None:
            return 0.0, 0.0
        return self.controller.goto_point(robot.pose, ball.position)
