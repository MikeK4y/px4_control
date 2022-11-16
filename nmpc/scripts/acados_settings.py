from acados_template import AcadosOcp, AcadosOcpSolver
from drone_model import drone_model
import numpy as np


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
    n_par = model.p.size()[0]
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

    # Weights
    W = np.eye(ny)

    # Stage cost
    ocp.cost.W = W

    # Terminal cost map
    Vx_e = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    # Terminal cost
    ocp.cost.W_e = N * W[:nx, :nx]

    # References
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(nx)

    # OCP constraints
    # Attitude constraints
    pitch_min = -0.2*np.pi
    pitch_max = 0.2*np.pi
    roll_min = -0.2*np.pi
    roll_max = 0.2*np.pi

    ocp.constraints.idxbx = np.array([6, 7])
    ocp.constraints.lbx = np.array([roll_min, pitch_min])
    ocp.constraints.ubx = np.array([roll_max, pitch_max])

    # Input constraints
    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.lbu = np.zeros(nu)
    ocp.constraints.ubu = np.zeros(nu)

    # Initial state
    ocp.constraints.x0 = np.zeros(nx)

    # Model
    ocp.model = model
    ocp.parameter_values = np.zeros(n_par)

    # OCP options
    ocp.solver_options.tf = Tf
    # PARTIAL_CONDENSING_HPIPM
    # FULL_CONDENSING_HPIPM
    # FULL_CONDENSING_QPOASES
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'

    acados_solver = AcadosOcpSolver(ocp)

    return model, acados_solver


if __name__ == "__main__":
    T = 3  # sec
    control_rate = 20  # Hz
    acados_settings(T, T*control_rate)
    print("Acados NMPC generated")
