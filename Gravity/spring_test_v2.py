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
dt = 0.001      # Pas de temps

# Corps fixe (body1) avec une grande masse mais finie
body1 = RigidBody(
    position=Vector2(0, 0),  # Point fixe
    velocity=Vector2(0, 0),  # Vitesse initiale nulle
    mass=1e10,               # Masse très grande pour simuler un point fixe
    angle=0,
    angular_velocity=0,
    inertia=1e10             # Inertie très grande pour éviter la rotation
)

# Corps mobile (body2)
body2 = RigidBody(
    position=Vector2(0, -2),  # Position initiale de la masse mobile
    velocity=Vector2(0, 0),  # Vitesse initiale nulle
    mass=5.0,                # Masse de la masse mobile
    angle=0,
    angular_velocity=0,
    inertia=1.0
)

# Initialisation du générateur de force de ressort
spring = SpringForceGenerator(
    rest_length=1.0,         # Longueur au repos du ressort
    spring_constant=100.0,    # Constante de ressort (raideur)
    damping=0,             # Amortissement ( se comporte comme les frottements donc si = 0 -> oscillateur harmonique)
    body1=body1,
    body2=body2
)

# Initialisation du générateur de force de gravité (uniquement sur body2)
gravity = GravityForceGenerator()  # Gravité vers le bas

# Initialisation du solveur
solver = Rk4OdeSolver()
solver.dt = dt

# Listes pour stocker le temps et les positions
time_data = []
y_positions = []

# Configuration de l'état du système
system = GenericRigidBodySystem(
    bodies=[body1, body2],
    force_generators=[spring,gravity],
    ode_solver = solver
)

# Boucle de simulation
positions_body2 = []

for t in range(int(simulation_time / dt)):
    system.process(dt, 1)

    body1.position = Vector2(0, 0)
    
    # Stocke les positions à chaque étape de temps
    positions_body2.append((body2.position.x, body2.position.y))

    # Enregistrer le temps et la position de body2
    time_data.append(t * dt)
    y_positions.append(body2.position.y)
    
    print(f"Time {t * dt:.2f}s: "
          f"Body2 Position {body2.position.x:.2f}, {body2.position.y:.2f} m, "
          f"Velocity {body2.velocity.x:.2f}, {body2.velocity.y:.2f} m/s")



# Visualisation de l'oscillation en utilisant Matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
body2_plot, = ax.plot([], [], 'ro', label='Body 2')

def init():
    body2_plot.set_data([], [])
    return body2_plot,

def update(frame):
    x, y = positions_body2[frame]
    body2_plot.set_data([x], [y])
    return body2_plot,

ani = FuncAnimation(fig, update, frames=len(positions_body2), init_func=init, blit=True)

# Sauvegarde l'animation en tant que GIF
ani.save('spring_simulation_v2.gif', writer='pillow', fps=30)


'''
# Tracer la position y de body2 en fonction du temps
plt.figure(figsize=(10, 6))
plt.plot(time_data, y_positions, label='Position y de Body2')
plt.title('Position y de Body2 en fonction du temps')
plt.xlabel('Temps (s)')
plt.ylabel('Position y (m)')
plt.legend()
plt.grid(True)
'''

plt.show()