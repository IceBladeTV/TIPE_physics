from gravity.force_generators.gravity_force_generator import GravityForceGenerator
from gravity.math.vector import Vector2
from gravity.rigid_body_systems.rigid_body import RigidBody
from gravity.rigid_body_systems.generic_rigid_body_system import GenericRigidBodySystem
from gravity.solvers.rk4_ode_solver import Rk4OdeSolver
import matplotlib.pyplot as plt
import matplotlib.animation as animation

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

# Initialisation du solver (RK4)
solver = Rk4OdeSolver()
solver.dt = dt

# Création du système
system = GenericRigidBodySystem(
    bodies=[body],
    force_generators=[gravity],
    ode_solver=solver
)

# Paramètres pour l'animation
fig, ax = plt.subplots(figsize=(5, 10))
ax.set_xlim(-1, 1)
ax.set_ylim(-500, 10)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Chute Libre')

mass_plot, = ax.plot([], [], 'bo', markersize=20)  # Représentation de la masse

def init():
    mass_plot.set_data([], [])
    return mass_plot,

def update(frame):
    system.process(dt, 1)
    mass_plot.set_data(body.position.x, body.position.y)
    return mass_plot,

ani = animation.FuncAnimation(fig, update, frames=int(simulation_time / dt),
                              init_func=init, blit=True)

# Sauvegarder l'animation en tant que GIF
ani.save('chute_libre_position.gif', writer='pillow', fps=30)

plt.show()
