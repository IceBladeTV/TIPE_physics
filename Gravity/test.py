from gravity.force_generators.gravity_force_generator import GravityForceGenerator
from gravity.math.vector import Vector2
from gravity.rigid_body_systems.rigid_body import RigidBody
from gravity.rigid_body_systems.generic_rigid_body_system import GenericRigidBodySystem
from gravity.solvers.euler_ode_solver import EulerOdeSolver
from gravity.solvers.rk4_ode_solver import Rk4OdeSolver
import matplotlib.pyplot as plt

# Initialisation des variables
g = 9.81  # accélération due à la gravité en m/s^2
simulation_time = 10  # durée de la simulation en secondes
dt = 0.01  # pas de temps en secondes

# Calcul de la position théorique finale
final_position_theoretical = -0.5 * g * simulation_time ** 2

# Fonction pour créer un corps rigide
def create_rigidbody():
    mass = 1.0  # masse en kg
    initial_position = Vector2(0, 0)  # position initiale en mètres
    initial_velocity = Vector2(0, 0)  # vitesse initiale (0 m/s)
    initial_angle = 0.0  # angle initial
    initial_angular_velocity = 0.0  # vitesse angulaire initiale
    inital_inertia = 1.0  # inertie
    return RigidBody(
        mass=mass,
        position=initial_position,
        velocity=initial_velocity,
        angle=initial_angle,
        angular_velocity=initial_angular_velocity,
        inertia=inital_inertia
    )

# Fonction pour exécuter une simulation avec un solver donné
def run_simulation(solver, steps):
    body = create_rigidbody()
    gravity = GravityForceGenerator()
    system = GenericRigidBodySystem(
        bodies=[body], 
        force_generators=[gravity],
        ode_solver=solver
    )
    
    time_data = []
    y_positions = []
    
    for t in range(int(simulation_time / dt)):
        system.process(dt, steps)
        current_time = t * dt
        time_data.append(current_time)
        y_positions.append(body.position.y)
    
    return time_data, y_positions

# Courbe théorique de la chute libre
time_data = [t * dt for t in range(int(simulation_time / dt))]
y_theoretical = [-(0.5 * g * t**2) for t in time_data]

# Courbe obtenue avec RK4 (1 step)
solver_rk4 = Rk4OdeSolver()
solver_rk4.dt = dt
time_data_rk4, y_rk4 = run_simulation(solver_rk4, steps=1)

# Courbe obtenue avec Euler (4 steps)
solver_euler = EulerOdeSolver()
solver_euler.dt = dt
time_data_euler, y_euler = run_simulation(solver_euler, steps=4)

# Tracer les courbes
plt.figure(figsize=(10, 6))
plt.plot(time_data, y_theoretical, 'r--', label='Théorique', linewidth=2)
plt.plot(time_data_rk4, y_rk4, 'b-', label='RK4, 1 Step', linewidth=2)
plt.plot(time_data_euler, y_euler, 'g-', label='Euler, 4 Steps', linewidth=2)

# Ajout d'une ligne horizontale pour la position finale théorique
plt.axhline(final_position_theoretical, color='red', linestyle=':', label=f'Position finale théorique: {final_position_theoretical:.2f} m')

# Annotations pour la position finale théorique
plt.text(simulation_time, final_position_theoretical, f'{final_position_theoretical:.2f} m', ha='left', va='bottom', color='red')

# Configuration du graphique
plt.title('Chute libre : Théorique vs RK4 (1 Step) vs Euler (4 Steps)')
plt.xlabel('Temps (s)')
plt.ylabel('Position y (m)')
plt.legend()
plt.grid(True)

# Afficher le graphique
plt.show()
