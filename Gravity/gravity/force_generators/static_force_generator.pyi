from ..system_state import SystemState
from .force_generator import ForceGenerator

from ..math.vector import Vector2


class StaticForceGenerator(ForceGenerator):
    def __init__(self, position: Vector2, force: Vector2) -> None:
        ...

    def apply(self, system_state: SystemState) -> None:
        ...