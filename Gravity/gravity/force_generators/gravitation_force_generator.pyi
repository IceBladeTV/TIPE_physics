from ..system_state import SystemState
from .force_generator import ForceGenerator


class GravitationForceGenerator(ForceGenerator):
    def __init__(self) -> None:
        ...

    def apply(self, system_state: SystemState) -> None:
        ...