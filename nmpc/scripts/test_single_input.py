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

x0 = np.array([0.01997384,  0.00121468,  1.0, -0.00693238,  0.0053205,   0.04041892, -0.00303419, -0.04739311,  0.01045966])
yref = np.array([0.5, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
yref_e = np.array([0.5, 0, 1.0, 0, 0, 0, 0, 0, 0])

parameter_values = np.array(
    [0.15,      # Roll time constant
    1.02,      # Roll gain
    0.15,      # Pitch time constant
    1.02,      # Pitch gain
    -0.38,     # Damping x
    -0.38,     # Damping y
    -0.10,     # Damping z
    -0.14634596,         # Disturbance force x
    -0.13851274,         # Disturbance force y
    -0.14672742,         # Disturbance force z
    13.74,     # Thrust coefficients
    -9.8066])  # Gravity

# Set initial conditions
acados_solver.set(0, "lbx", x0)
acados_solver.set(0, "ubx", x0)

# Set reference
for i in range(N):
    acados_solver.set(i, "yref", yref)
    acados_solver.set(i, 'p', parameter_values)

acados_solver.set(N, "yref", yref_e)
acados_solver.set(N, 'p', parameter_values)

# Solve without RTI
print("Solving without warm start")
# acados_solver.options_set("rti_phase", 1)
acados_solver.options_set('qp_warm_start', 0)

# solve ocp
t = time.time()

status = acados_solver.solve()
if status != 0:
    print("acados returned status {}.".format(status))

elapsed = 1000 * (time.time() - t)

print("Elapsed time: {}ms".format(elapsed))

# get solution
x_curr = acados_solver.get(1, "x")
u = acados_solver.get(0, "u")

# print("u_0: {}, {}, {}, {}\n".format(u[0], u[1], u[2], u[3]))

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


# Solve with RTI
print("Solving with warm start")

# acados_solver.options_set("rti_phase", 1)
acados_solver.options_set('qp_warm_start', 1)

# solve ocp
t = time.time()

status = acados_solver.solve()
if status != 0:
    print("acados returned status {}.".format(status))

elapsed = 1000 * (time.time() - t)

print("Elapsed time: {}ms".format(elapsed))

# get solution
x_curr = acados_solver.get(1, "x")
u = acados_solver.get(0, "u")

# print("u_0: {}, {}, {}, {}\n".format(u[0], u[1], u[2], u[3]))

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