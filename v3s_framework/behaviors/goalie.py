from .behavior import Behavior
from core.types import Vector2, Robot, Ball
from core.math_utils import normalize_angle
import math

class Goalie(Behavior):
    """
    Goleiro limitado à caixa de defesa:
      x ∈ [zone_min_x, zone_max_x]
      y ∈ [zone_min_y, zone_max_y]

    1) Gira até a orientação fixa (goalie_orientation).
    2) Se sair da zona, direciona-se à borda mais próxima.
    3) Senão, move em direção à bola, mas clampada à zona.
    """

    def __init__(
        self,
        goalie_orientation: float = math.pi/2,
        k_turn: float = 10.0,
        k_move: float = 90.0,
        angle_threshold: float = 0.1,
        stop_tolerance: float = 0.02,
        zone_min_x: float =  0.5,
        zone_max_x: float =  0.7,
        zone_min_y: float = -0.3,
        zone_max_y: float =  0.3,
    ):
        self.goalie_orientation = goalie_orientation
        self.k_turn            = k_turn
        self.k_move            = k_move
        self.angle_threshold   = angle_threshold
        self.stop_tolerance    = stop_tolerance
        self.zone_min_x        = zone_min_x
        self.zone_max_x        = zone_max_x
        self.zone_min_y        = zone_min_y
        self.zone_max_y        = zone_max_y

    def step(self, robot: Robot, ball: Ball | None) -> tuple[float, float]:
        # 1) Gira até a orientação de goleiro
        err_ang = normalize_angle(self.goalie_orientation - robot.pose.theta)
        if abs(err_ang) > self.angle_threshold:
            return 0.0, self.k_turn * err_ang

        # 2) Definir target: se fora da zona, volta; senão, segue bola clampada
        pos = robot.pose.position
        # verifica se saiu dos limites
        outside = (
            pos.x < self.zone_min_x or pos.x > self.zone_max_x or
            pos.y < self.zone_min_y or pos.y > self.zone_max_y
        )

        if outside:
            # target = ponto mais próximo DA ZONA (clamp da posição atual)
            target = Vector2(
                max(self.zone_min_x, min(self.zone_max_x, pos.x)),
                max(self.zone_min_y, min(self.zone_max_y, pos.y)),
            )
        else:
            if ball is None:
                return 0.0, 0.0
            raw = Vector2(ball.position.x, ball.position.y)
            # clamp da bola à zona
            target = Vector2(
                max(self.zone_min_x, min(self.zone_max_x, raw.x)),
                max(self.zone_min_y, min(self.zone_max_y, raw.y)),
            )

        # 3) Projeção ao longo do eixo frontal (heading) do robô
        dx = target.x - pos.x
        dy = target.y - pos.y
        hx = math.cos(robot.pose.theta)
        hy = math.sin(robot.pose.theta)
        dist = dx * hx + dy * hy

        if abs(dist) < self.stop_tolerance:
            forward = 0.0
        else:
            forward = self.k_move * dist

        return forward, forward
