import time
import numpy as np
from acados_settings import acados_settings

Tf = 2.0  # Prediction horizon
N = 20    # Discretization steps

# Load model and solver
model, acados_solver = acados_settings(Tf, N)

# Dimensions
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu

# Set initial condition
x0 = np.array([0, 0, 1.0, 0, 0, 0, 0, 0, 0])

acados_solver.set(0, "lbx", x0)
acados_solver.set(0, "ubx", x0)

# Set reference
yref =   np.array([0.1, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
yref_e = np.array([0.1, 0, 1.0, 0, 0, 0, 0, 0, 0])

for i in range(N):
    acados_solver.set(i, "yref", yref)

acados_solver.set(N, "yref", yref_e)

# solve ocp
t = time.time()

status = acados_solver.solve()
if status != 0:
    print("acados returned status {}.".format(status))

elapsed = 1000* (time.time() - t)

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
