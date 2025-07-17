from .types import Pose, Vector2
from .math_utils import normalize_angle
import math

class DifferentialController:
    """
    Calcula velocidades de roda para um robô com tração diferencial.
    """
    def __init__(
        self,
        k_forward: float = 50.0,
        k_turn: float = 10.0,
        stop_distance: float = 0.10,
        angle_threshold: float = 0.10,
    ):
        self.k_forward = k_forward
        self.k_turn = k_turn
        self.stop_distance = stop_distance
        self.angle_threshold = angle_threshold

    def goto_point(self, robot: Pose, target: Vector2) -> tuple[float, float]:
        dx, dy = target.x - robot.position.x, target.y - robot.position.y
        distance = math.hypot(dx, dy)

        angle_to_target = math.atan2(dy, dx)
        angle_err = normalize_angle(angle_to_target - robot.theta)

        if distance < self.stop_distance:
            fwd = turn = 0.0
        else:
            fwd = 0.0 if abs(angle_err) > self.angle_threshold else self.k_forward * distance
            turn = self.k_turn * angle_err

        return fwd - turn, fwd + turn  # (left_speed, right_speed)
