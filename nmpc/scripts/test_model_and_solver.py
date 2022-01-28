import time
import numpy as np
import random
from acados_settings import acados_settings

Tf = 2.0  # Prediction horizon
N = 40    # Discretization steps

# Load model and solver
model, acados_solver = acados_settings(Tf, N)

# Dimensions
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu

# Set initial condition
x0 = np.array([0.0, 0.0, 1.0, 0.00, 0.0, 0.0, 0.0, 0.0, 0.0])
var = np.array([random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2),
                random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2),
                random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2)])
x0 += var
parameter_values = np.array(
    [0.15,      # Roll time constant
     1.02,      # Roll gain
     0.15,      # Pitch time constant
     1.02,      # Pitch gain
     -0.38,     # Damping x
     -0.38,     # Damping y
     -0.10,     # Damping z
     random.uniform(-2, 2),         # Disturbance force x
     random.uniform(-2, 2),         # Disturbance force y
     random.uniform(-2, 2),         # Disturbance force z
     13.74,     # Thrust coefficients
     -9.8066])  # Gravity


acados_solver.set(0, "lbx", x0)
acados_solver.set(0, "ubx", x0)

print("X initial: {}".format(x0))
print("Parameters: {}".format(parameter_values))

# Set reference
yref = np.array([0.0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
yref_e = np.array([0.0, 0, 1.0, 0, 0, 0, 0, 0, 0])

for i in range(N):
    acados_solver.set(i, "yref", yref)
    acados_solver.set(i, 'p', parameter_values)

acados_solver.set(N, "yref", yref_e)
acados_solver.set(N, 'p', parameter_values)

# solve ocp
t = time.time()

status = acados_solver.solve()
if status != 0:
    print("acados returned status {}.".format(status))

elapsed = 1000 * (time.time() - t)

print("Elapsed time: {}ms".format(elapsed))

# get solution
x = acados_solver.get(0, "x")
u = acados_solver.get(0, "u")

print("x:")
for i in range(N + 1):
    x = acados_solver.get(i, "x")
    print("{}:\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}".format(
        i, x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8]))

print("\nu:")
for i in range(N):
    u = acados_solver.get(i, "u")
    print("{}:\t{}\t{}\t{}\t{}".format(
        i, u[0], u[1], u[2], u[3]))
