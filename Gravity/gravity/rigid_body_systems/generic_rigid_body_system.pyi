from .rigid_body_system import RigidBodySystem
from .rigid_body import RigidBody
from ..force_generators.force_generator import ForceGenerator
from ..solvers.ode_solver import OdeSolver


class GenericRigidBodySystem(RigidBodySystem):
    def __init__(self, bodies: list[RigidBody], force_generators: list[ForceGenerator], ode_solver: OdeSolver) -> None:
        ...

    def process(self, dt: float, steps: int) -> None:
        ...