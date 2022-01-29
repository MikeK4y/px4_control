from unittest import TestProgram
from cv2 import sepFilter2D
import numpy as np
import rospy as rp
import math
import time

from acados_settings import acados_settings

from mavros_msgs.msg import AttitudeTarget, PositionTarget, State
from std_srvs.srv import SetBool
from px4_control_msgs.msg import DroneState, Setpoint

import threading

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert rpy to quaternions
    """
    t0 = math.cos(yaw * 0.5)
    t1 = math.sin(yaw * 0.5)
    t2 = math.cos(roll * 0.5)
    t3 = math.sin(roll * 0.5)
    t4 = math.cos(pitch * 0.5)
    t5 = math.sin(pitch * 0.5)

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


class PX4NMPCNode():
    """
    A ROS node that implements an NMPC using acados
    """

    def __init__(self, Tf, N):
        # Acados NMPC
        # Load model and solver
        model, self.acados_solver = acados_settings(Tf, N)
        self.has_drone_state = False
        self.has_trajectory = False
        self.enable_controller = False
        self.mavros_status = State()

        # Dimensions
        self.N = N
        self.NX = model.x.size()[0]
        self.NU = model.u.size()[0]
        self.NY = self.NX + self.NU

        self.x_curr = np.zeros((self.NX, 1))
        self.y_ref = np.zeros((self.NY, 1))
        self.y_ref_e = np.zeros((self.NX, 1))
        self.parameter_values = np.array(
            [0.15,      # Roll time constant
             1.02,      # Roll gain
             0.15,      # Pitch time constant
             1.02,      # Pitch gain
             -0.38,     # Damping x
             -0.38,     # Damping y
             -0.10,     # Damping z
             0.0,       # Disturbance force x
             0.0,       # Disturbance force y
             0.0,       # Disturbance force z
             13.74,     # Thrust coefficients
             -9.8066])  # Gravity

        rp.init_node("px4_control_node_py")
        self.rate = N / Tf

        # ROS Publishers
        self.att_control_pub = rp.Publisher(
            "/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)
        self.vel_control_pub = rp.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

        # ROS Subscribers
        self.drone_state_sub = rp.Subscriber(
            "/drone_state", DroneState, self.droneStateCallback, queue_size=1)
        self.setpoint_sub = rp.Subscriber(
            "/drone_setpoint", Setpoint, self.setpointCallback, queue_size=1)
        self.mavros_status_sub = rp.Subscriber(
            "/mavros/state", State, self.mavrosStatusCallback, queue_size=1)
        
        # ROS Services
        self.enable_controller_serv = rp.Service("/enable_controller", SetBool, self.enableControllerServCallback)

        t = threading.Thread(target=self.controlPublisher)
        t.start()

        rp.spin()

    def enableControllerServCallback(self, req):
      if not req:
        self.enable_controller = False
      elif (req and not self.has_trajectory):
        self.enable_controller = False
      elif (req and self.has_trajectory):
        self.enable_controller = True

      return True

    def droneStateCallback(self, msg):
        self.x_curr[0] = msg.pose.position.x
        self.x_curr[1] = msg.pose.position.y
        self.x_curr[2] = msg.pose.position.z
        self.x_curr[3] = msg.velocity.x
        self.x_curr[4] = msg.velocity.y
        self.x_curr[5] = msg.velocity.z
        self.x_curr[6] = msg.orientation_euler.x
        self.x_curr[7] = msg.orientation_euler.y
        self.x_curr[8] = msg.orientation_euler.z

        self.parameter_values[7] = msg.disturbances.x
        self.parameter_values[8] = msg.disturbances.y

        if (abs(msg.pose.position.z) < 0.1 and abs(msg.disturbances.z - 9.8066) < 0.1):
            self.parameter_values[9] = 0.0
        else:
            self.parameter_values[9] = msg.disturbances.z

        self.has_drone_state = True

    def setpointCallback(self, msg):
        self.y_ref[0] = msg.position.x
        self.y_ref[1] = msg.position.y
        self.y_ref[2] = msg.position.z
        self.y_ref[3] = msg.velocity.x
        self.y_ref[4] = msg.velocity.y
        self.y_ref[5] = msg.velocity.z
        self.y_ref[6] = msg.orientation.x
        self.y_ref[7] = msg.orientation.y
        self.y_ref[8] = msg.orientation.z

        self.y_ref_e[0] = msg.position.x
        self.y_ref_e[1] = msg.position.y
        self.y_ref_e[2] = msg.position.z
        self.y_ref_e[3] = msg.velocity.x
        self.y_ref_e[4] = msg.velocity.y
        self.y_ref_e[5] = msg.velocity.z
        self.y_ref_e[6] = msg.orientation.x
        self.y_ref_e[7] = msg.orientation.y
        self.y_ref_e[8] = msg.orientation.z

        self.has_trajectory = True

    def mavrosStatusCallback(self, msg):
        self.mavros_status = msg

    def controlPublisher(self,):
        r = rp.Rate(self.rate)

        # Attitude command
        att_cmd = AttitudeTarget()
        att_cmd.type_mask = att_cmd.IGNORE_ROLL_RATE + att_cmd.IGNORE_PITCH_RATE

        # Velocity command
        vel_cmd = PositionTarget()
        vel_cmd.coordinate_frame = vel_cmd.FRAME_BODY_NED
        vel_cmd.type_mask = vel_cmd.IGNORE_PX + vel_cmd.IGNORE_PY + \
            vel_cmd.IGNORE_PZ + \
            vel_cmd.IGNORE_AFX + vel_cmd.IGNORE_AFY + vel_cmd.IGNORE_AFZ + \
            vel_cmd.IGNORE_YAW
        vel_cmd.velocity.x = 0.0
        vel_cmd.velocity.y = 0.0
        vel_cmd.velocity.z = 0.0
        vel_cmd.yaw_rate = 0.0

        while not rp.is_shutdown():
            if (self.mavros_status.connected):
                is_offboard = self.mavros_status.mode == "OFFBOARD"
                if (self.enable_controller and self.has_drone_state and self.has_trajectory and is_offboard):
                    # Set initial conditions
                    current_yaw = self.x_curr[8]
                    self.acados_solver.set(0, "lbx", self.x_curr)
                    self.acados_solver.set(0, "ubx", self.x_curr)

                    # Set reference and parameters
                    for i in range(self.N):
                        self.acados_solver.set(i, "yref", self.y_ref)
                        self.acados_solver.set(i, 'p', self.parameter_values)

                    self.acados_solver.set(self.N, "yref", self.y_ref_e)
                    self.acados_solver.set(self.N, 'p', self.parameter_values)

                    t = time.time()
                    ocp_status = self.acados_solver.solve()
                    elapsed = 1000 * (time.time() - t)
                    if ocp_status == 0:
                        rp.loginfo("OCP solution time {}ms".format(elapsed))
                        u = self.acados_solver.get(0, "u")

                        q = to_quaternion(u[2], u[1], current_yaw)
                        thrust = u[3]
                        if thrust < 0.1:
                            thrust = 0.1

                        att_cmd.header.stamp = rp.Time.now()
                        att_cmd.orientation.w = q[0]
                        att_cmd.orientation.x = q[1]
                        att_cmd.orientation.y = q[2]
                        att_cmd.orientation.z = q[3]
                        att_cmd.body_rate.z = u[0]
                        att_cmd.thrust = thrust

                        self.att_control_pub.publish(att_cmd)

                    else:
                        rp.logerr("OCP failed to find solution")
                        vel_cmd.header.stamp = rp.Time.now()
                        self.vel_control_pub.publish(vel_cmd)

                else:
                    vel_cmd.header.stamp = rp.Time.now()
                    self.vel_control_pub.publish(vel_cmd)

            r.sleep()


PX4NMPCNode(2.0, 40)
