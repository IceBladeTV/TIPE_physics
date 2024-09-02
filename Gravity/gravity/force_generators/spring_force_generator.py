from ..system_state import SystemState
from .force_generator import ForceGenerator
from ..rigid_body_systems.rigid_body import RigidBody

from ..math.vector import Vector2


class SpringForceGenerator(ForceGenerator):
    def __init__(self, rest_length: float, spring_constant: float, damping: float, body1: RigidBody, body2: RigidBody, local_pos1: Vector2 = Vector2(), local_pos2: Vector2 = Vector2()) -> None:
        self.rest_length: float = rest_length
        self.spring_constant: float = spring_constant
        self.damping: float = damping

        self.body_1: RigidBody = body1
        self.body_2: RigidBody = body2

        self.local_position_1: Vector2 = local_pos1
        self.local_position_2: Vector2 = local_pos2

    def apply(self, system_state: SystemState) -> None:
        delta = self.body_2.position - self.body_1.position
        delta_velocity = self.body_2.velocity - self.body_1.velocity

        length = delta.length()

        magnitude = (length - self.rest_length) * self.spring_constant

        if length == 0:
            delta = Vector2(0.1, 0)

        damping_force = delta_velocity.dot(delta.normalize()) * self.damping

        force = (magnitude + damping_force) * delta.normalize()

        system_state.apply_force(self.body_1, force, self.local_position_1)
        system_state.apply_force(self.body_2, -force, self.local_position_2)