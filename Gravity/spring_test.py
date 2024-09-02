from gravity.force_generators.gravity_force_generator import GravityForceGenerator
from gravity.force_generators.spring_force_generator import SpringForceGenerator
from gravity.math.vector import Vector2
from gravity.rigid_body_systems.rigid_body import RigidBody
from gravity.rigid_body_systems.rigid_body_system import RigidBodySystem
from gravity.rigid_body_systems.generic_rigid_body_system import GenericRigidBodySystem
from gravity.solvers.euler_ode_solver import EulerOdeSolver
from gravity.solvers.rk4_ode_solver import Rk4OdeSolver
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Simulation sur 10 secondes
simulation_time = 10.0  # Durée totale de la simulation
dt = 0.01        # Pas de temps

# Initialisation des corps rigides
body1 = RigidBody(
    position=Vector2(0, 0),  # Position initiale du premier corps
    velocity=Vector2(0, 0),  # Vitesse initiale du premier corps
    mass=5.0,                # Masse du premier corps
    angle=0,
    angular_velocity=0,
    inertia=1.0
)

body2 = RigidBody(
    position=Vector2(2, 0),  # Position initiale du second corps, décalée de 2 mètres
    velocity=Vector2(0, 0),  # Vitesse initiale du second corps
    mass=5.0,                # Masse du second corps
    angle=0,
    angular_velocity=0,
    inertia=1.0
)

# Initialisation du générateur de force de ressort
spring = SpringForceGenerator(
    rest_length=1.0,         # Longueur au repos du ressort
    spring_constant=50.0,    # Constante de ressort (raideur)
    damping=0.1,             # Amortissement
    body1=body1,
    body2=body2
)

# Initialisation du solveur
solver = Rk4OdeSolver()
solver.dt = dt

# Configuration de l'état du système
system = GenericRigidBodySystem(
    bodies=[body1, body2],
    force_generators=[spring],
    ode_solver = solver
)

# Configuration de l'animation
fig, ax = plt.subplots()
ax.set_xlim(-3, 5)  # Ajustez selon la plage de votre simulation
ax.set_ylim(-1, 1)  # Ajustez si nécessaire
body1_plot, = ax.plot([], [], 'bo', label='Body 1')
body2_plot, = ax.plot([], [], 'ro', label='Body 2')

positions_body1 = []
positions_body2 = []

# Boucle de simulation
for t in range(int(simulation_time / dt)):
    system.process(dt, 1)
    
    # Stocke les positions à chaque étape de temps
    positions_body1.append((body1.position.x, 0))  # Utilise (x, 0) pour body1
    positions_body2.append((body2.position.x, 0))  # Utilise (x, 0) pour body2
    
    print(f"Time {t * dt:.2f}s: "
          f"Body1 Position {body1.position.x:.2f} m, Velocity {body1.velocity.x:.2f} m/s | "
          f"Body2 Position {body2.position.x:.2f} m, Velocity {body2.velocity.x:.2f} m/s")

def init():
    body1_plot.set_data([], [])
    body2_plot.set_data([], [])
    return body1_plot, body2_plot

def update(frame):
    try:
        x1, y1 = positions_body1[frame]  # Décompose le tuple (x, 0) pour body1
        x2, y2 = positions_body2[frame]  # Décompose le tuple (x, 0) pour body2
        body1_plot.set_data([x1], [y1])
        body2_plot.set_data([x2], [y2])
    except IndexError:
        pass
    return body1_plot, body2_plot

# Nombre total de frames dans l'animation
num_frames = len(positions_body1)

# Crée l'animation
ani = FuncAnimation(fig, update, frames=num_frames, init_func=init, blit=True)

# Sauvegarde l'animation en tant que GIF
ani.save('spring_simulation.gif', writer='pillow', fps=30)

plt.show()