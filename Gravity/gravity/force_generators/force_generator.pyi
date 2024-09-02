from ..system_state import SystemState


class ForceGenerator:
    def __init__(self) -> None:
        ...

    def apply(self, system_state: SystemState) -> None:
        ...