from ..system_state import SystemState


class ForceGenerator:
    def __init__(self) -> None:
        raise NotImplementedError

    def apply(self, system_state: SystemState) -> None:
        raise NotImplementedError