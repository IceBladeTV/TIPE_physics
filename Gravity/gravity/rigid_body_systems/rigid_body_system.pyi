from ..system_state import SystemState
from ..rigid_body_systems.rigid_body import RigidBody
from ..force_generators.force_generator import ForceGenerator


class RigidBodySystem:
    def __init__(self, bodies: list[RigidBody], force_generators: list[ForceGenerator]) -> None:
        self.bodies: list[RigidBody]

        self.system_state: SystemState

    def process(self, dt: float, steps: int) -> None:
        ...

    def populate_system_state(self) -> None:
        ...
        
    def process_forces(self) -> None:
        ...