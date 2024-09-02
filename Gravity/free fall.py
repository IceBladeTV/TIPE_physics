from gravity.force_generators.gravity_force_generator import GravityForceGenerator
from gravity.force_generators.spring_force_generator import SpringForceGenerator
from gravity.math.vector import Vector2
from gravity.rigid_body_systems.rigid_body import RigidBody
from gravity.rigid_body_systems.rigid_body_system import RigidBodySystem
from gravity.rigid_body_systems.generic_rigid_body_system import GenericRigidBodySystem
from gravity.solvers.euler_ode_solver import EulerOdeSolver
from gravity.solvers.rk4_ode_solver import Rk4OdeSolver
import matplotlib.pyplot as plt

# Initialisation des variables 

g = 9.81  # accélération due à la gravité en m/s^2
simulation_time = 10  # durée de la simulation en secondes
dt = 0.01  # pas de temps en secondes

# Créer un objet RigidBody pour représenter la masse
mass = 1.0  # masse en kg
initial_position = Vector2(0, 0)  # position initiale en mètres (100 m au-dessus du sol)
initial_velocity = Vector2(0, 0)  # vitesse initiale (0 m/s)
initial_angle = 0.0  # angle initial (inutile pour une chute libre, donc 0)
initial_angular_velocity = 0.0  # vitesse angulaire initiale (inutile pour une chute libre, donc 0)
inital_inertia = 1.0  # inertie (inutile pour une chute libre simple)

body = RigidBody(
    mass=mass,
    position=initial_position,
    velocity=initial_velocity,
    angle=initial_angle,
    angular_velocity=initial_angular_velocity,
    inertia=inital_inertia
)

# Création de la force de gravité
gravity = GravityForceGenerator()

# Initialisation du solver sous méthode d'Euler
solver = Rk4OdeSolver()
solver.dt = dt

# Création du système 
system = GenericRigidBodySystem(
    bodies=[body], 
    force_generators=[gravity],
    ode_solver = solver
)

time_data = []
y_positions = []

# Exécuter la simulation pour 10 secondes
for t in range(int(simulation_time / dt)):
    system.process(dt, 1)
    current_time = t * dt

    # Enregistrer le temps et la position de body2
    time_data.append(current_time)
    y_positions.append(body.position.y)
    
    print(f"Time {current_time:.2f}s: Position {body.position.y:.2f} m, Velocity {body.velocity.y:.2f} m/s")

# Comparer la position finale avec la solution analytique : 1/2 * g * t^2
final_position = initial_position.y - (0.5 * g * simulation_time**2)
print(f"\nPosition finale théorique: {final_position:.2f} m")
print(f"Position finale simulée: {body.position.y:.2f} m")

plt.figure(figsize=(10, 6))
plt.plot(time_data, y_positions, label='RK4, 1 Steps')
plt.title('Chute libre')
plt.xlabel('Temps (s)')
plt.ylabel('Position y (m)')
plt.legend()
plt.grid(True)

plt.show()