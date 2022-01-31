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

x0 = np.array([0.0, 0.0, 0.5, 0.00, 0.0, 0.0, 0.0, 0.0, 0.0])
yref = np.array([0.0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
yref_e = np.array([0.0, 0, 1.0, 0, 0, 0, 0, 0, 0])


parameter_values = np.array(
    [0.15,      # Roll time constant
    1.02,      # Roll gain
    0.15,      # Pitch time constant
    1.02,      # Pitch gain
    -0.38,     # Damping x
    -0.38,     # Damping y
    -0.10,     # Damping z
    0.0,         # Disturbance force x
    0.0,         # Disturbance force y
    0.0,         # Disturbance force z
    13.74,     # Thrust coefficients
    -9.8066])  # Gravity

x_curr = x0

for i in range(10):
    # Set initial conditions
    var = np.array([random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05),
                    random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05),
                    random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05)])
    x_curr += var

    parameter_values[7] += random.uniform(-0.5, 0.5)
    parameter_values[8] += random.uniform(-0.5, 0.5)
    parameter_values[9] += random.uniform(-0.5, 0.5)

    acados_solver.set(0, "lbx", x_curr)
    acados_solver.set(0, "ubx", x_curr)

    print("Current state: {}".format(x_curr))
    print("Disturbances: {}".format(parameter_values[7:10]))

    # Set reference
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
    x_curr = acados_solver.get(1, "x")
    u = acados_solver.get(0, "u")

    print("u_0: {}\n".format(u))
    print("x_1: {}\n".format(x_curr))

# print("x:")
# for i in range(N + 1):
#     x = acados_solver.get(i, "x")
#     print("{}:\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}".format(
#         i, x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8]))

# print("\nu:")
# for i in range(N):
#     u = acados_solver.get(i, "u")
#     print("{}:\t{}\t{}\t{}\t{}".format(
#         i, u[0], u[1], u[2], u[3]))


# enum return_values
# {
#     ACADOS_SUCCESS,
#     ACADOS_FAILURE,
#     ACADOS_MAXITER,
#     ACADOS_MINSTEP,
#     ACADOS_QP_FAILURE,
#     ACADOS_READY,
# };