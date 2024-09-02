import time
from gravity.force_generators.gravity_force_generator import GravityForceGenerator
from gravity.math.vector import Vector2
from gravity.rigid_body_systems.rigid_body import RigidBody
from gravity.rigid_body_systems.generic_rigid_body_system import GenericRigidBodySystem
from gravity.solvers.euler_ode_solver import EulerOdeSolver
from gravity.solvers.rk4_ode_solver import Rk4OdeSolver
import matplotlib.pyplot as plt

# Paramètres de la simulation
g = 9.81  # accélération due à la gravité en m/s^2
simulation_time = 10  # durée de la simulation en secondes
dt = 0.01  # pas de temps en secondes

# Création du corps rigide (la masse)
mass = 1.0  # masse en kg
initial_position = Vector2(0, 0)  # position initiale en mètres
initial_velocity = Vector2(0, 0)  # vitesse initiale (0 m/s)
initial_angle = 0.0  # angle initial
initial_angular_velocity = 0.0  # vitesse angulaire initiale
inital_inertia = 1.0  # inertie

# Initialisation des positions pour les deux méthodes
euler_y_positions = []
rk4_y_positions = []
time_data = []

# Création de la force de gravité
gravity = GravityForceGenerator()

# ---- Simulation avec Euler (4 steps) ----
body_euler = RigidBody(
    mass=mass,
    position=initial_position,
    velocity=initial_velocity,
    angle=initial_angle,
    angular_velocity=initial_angular_velocity,
    inertia=inital_inertia
)

solver_euler = EulerOdeSolver()
solver_euler.dt = dt / 4

system_euler = GenericRigidBodySystem(
    bodies=[body_euler],
    force_generators=[gravity],
    ode_solver=solver_euler
)

start_time_euler = time.time()

for t in range(int(simulation_time / dt)):
    system_euler.process(dt, 4)
    euler_y_positions.append(body_euler.position.y)
    time_data.append(t * dt)

end_time_euler = time.time()
euler_duration = end_time_euler - start_time_euler

# ---- Simulation avec RK4 (1 step) ----
body_rk4 = RigidBody(
    mass=mass,
    position=initial_position,
    velocity=initial_velocity,
    angle=initial_angle,
    angular_velocity=initial_angular_velocity,
    inertia=inital_inertia
)

solver_rk4 = Rk4OdeSolver()
solver_rk4.dt = dt

system_rk4 = GenericRigidBodySystem(
    bodies=[body_rk4],
    force_generators=[gravity],
    ode_solver=solver_rk4
)

start_time_rk4 = time.time()

for t in range(int(simulation_time / dt)):
    system_rk4.process(dt, 1)
    rk4_y_positions.append(body_rk4.position.y)

end_time_rk4 = time.time()
rk4_duration = end_time_rk4 - start_time_rk4

# ---- Affichage des résultats ----
print(f"Temps d'exécution avec Euler (4 steps): {euler_duration:.4f} secondes")
print(f"Temps d'exécution avec RK4 (1 step): {rk4_duration:.4f} secondes")

plt.figure(figsize=(10, 6))
plt.plot(time_data, euler_y_positions, label='Euler, 4 Steps')
plt.plot(time_data, rk4_y_positions, label='RK4, 1 Step')
plt.title('Comparaison de la chute libre')
plt.xlabel('Temps (s)')
plt.ylabel('Position y (m)')
plt.legend()
plt.grid(True)

plt.show()
