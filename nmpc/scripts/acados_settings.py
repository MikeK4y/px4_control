from acados_template import AcadosOcp, AcadosOcpSolver, acados_ocp
from drone_model import drone_model
import numpy as np
import scipy.linalg


def acados_settings(Tf, N):
    """
      Set's up the acados Optimal Control Problem and Solver

      ...

      Parameters
      ----------
      Tf : double
        Prediction horizon
      N : int
        Prediction steps
    """

    # Get drone model
    model = drone_model()

    # acados OCP handle
    ocp = AcadosOcp()

    # OCP dimensions
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ocp.dims.N = N

    # OCP costs
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    # State map
    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    # Input map
    Vu = np.zeros((ny, nu))
    Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    # Terminal cost map
    Vx_e = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    # State and input cost
    W = np.diag([1e2, 1e2, 1e2,     # Position
                 1e-3, 1e-3, 1e-3,  # Velocity
                 1e-6, 1e-6, 1e-6,  # Attitude
                 1e-9,              # Yaw rate
                 1e-9,              # Pitch
                 1e-9,              # Roll
                 1e-9])             # Thrust

    # Stage cost
    ocp.cost.W = W

    # Terminal cost
    ocp.cost.W_e = N * W[:nx, :nx]

    # References
    ocp.cost.yref = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    ocp.cost.yref_e = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])

    # OCP constraints
    # Attitude constraints
    pitch_min = -0.2*np.pi
    pitch_max = 0.2*np.pi
    roll_min = -0.2*np.pi
    roll_max = 0.2*np.pi
    yaw_min = -np.pi
    yaw_max = np.pi

    ocp.constraints.idxbx = np.array([6, 7, 8])
    ocp.constraints.lbx = np.array([roll_min, pitch_min, yaw_min])
    ocp.constraints.ubx = np.array([roll_max, pitch_max, yaw_max])

    # Input constraints
    yaw_rate_min = -0.5*np.pi
    yaw_rate_max = 0.5*np.pi
    pitch_cmd_min = -0.2*np.pi
    pitch_cmd_max = 0.2*np.pi
    roll_cmd_min = -0.2*np.pi
    roll_cmd_max = 0.2*np.pi
    thrust_min = -1.0
    thrust_max = 1.0

    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    ocp.constraints.lbu = np.array(
        [yaw_rate_min, pitch_cmd_min, roll_cmd_min, thrust_min])
    ocp.constraints.ubu = np.array(
        [yaw_rate_max, pitch_cmd_max, roll_cmd_max, thrust_max])

    # Initial state
    ocp.constraints.x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])

    # Model
    ocp.model = model
    ocp.parameter_values = np.array(
        [0.15,     # Roll time constant
         1.02,     # Roll gain
         0.15,     # Pitch time constant
         1.02,     # Pitch gain
         -0.38,    # Damping x
         -0.38,    # Damping y
         -0.10,    # Damping z
         0,        # Disturbance force x
         0,        # Disturbance force y
         0,        # Disturbance force z
         13.74,    # Thrust coefficients
         -9.8066])  # Gravity

    # OCP options
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'

    # Path to acados include and lib directories
    # Uncomment if you want to build generated c code
    # ocp.acados_include_path = '../../acados/include'
    # ocp.acados_lib_path = '../../acados/lib'

    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

    return model, acados_solver


acados_settings(3, 20)
print("Acados NMPC generated")
