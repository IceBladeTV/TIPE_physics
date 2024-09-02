from ..system_state import SystemState
from .force_generator import ForceGenerator
from ..rigid_body_systems.rigid_body import RigidBody

from ..math.vector import Vector2


class StaticForceGenerator(ForceGenerator):
    def __init__(self, position: Vector2 = Vector2(), force: Vector2 = Vector2()) -> None:
        self.position = position
        self.force = force

        self.body: RigidBody

    def apply(self, system_state: SystemState) -> None:
        system_state.apply_force(self.body, self.force, self.position)