#!/usr/bin/python3
import threading

import rospy as rp
import numpy as np
import quaternion

from px4_control_msgs.msg import DroneStateMarker, Setpoint, Trajectory


class StateMachineNode():
    """
      Simple state machine that checks the state and when the target is found it
      sends a setpoint relative to it. It also tracks the target's position and
      if it changes too much it updates the setpoint
    """

    def __init__(self):
        rp.init_node('state_machine_node')

        # Check that NMPC is running by checking that a service is available
        rp.loginfo('Checking that controller is up')
        rp.wait_for_service('/enable_controller')
        rp.loginfo('NMPC is up')

        # Setpoint
        self.H_marker_setpoint = np.array([[1.0, 0.0, 1.0, -0.5],
                                           [0.0, 1.0, 0.0,  0.0],
                                           [0.0, 0.0, 1.0,  1.5],
                                           [0.0, 0.0, 0.0,  1.0]])
        self.marker_setpoint_sent = False
        self.marker_position = None

        # Subscribers
        self.state_sub = rp.Subscriber(
            '/drone_state', DroneStateMarker, self.stateCallback, queue_size=1)

        # Publishers
        self.trajectory_pub = rp.Publisher(
            '/drone_trajectory', Trajectory, queue_size=1, latch=True)

        rp.spin()

    def stateCallback(self, msg):
        if msg.marker_found.data:
            if not self.marker_setpoint_sent:
                # Get marker world position
                self.marker_position = np.array([msg.marker_pose.position.x,
                                                 msg.marker_pose.position.y,
                                                 msg.marker_pose.position.z])

                marker_att = np.quaternion(msg.marker_pose.orientation.w,
                                           msg.marker_pose.orientation.x,
                                           msg.marker_pose.orientation.y,
                                           msg.marker_pose.orientation.z).normalized()

                H_world_marker = np.identity(4)
                H_world_marker[0, 3] = msg.marker_pose.position.x
                H_world_marker[1, 3] = msg.marker_pose.position.y
                H_world_marker[2, 3] = msg.marker_pose.position.z
                H_world_marker[0:3, 0:3] = quaternion.as_rotation_matrix(
                    marker_att)

                # Transform setpoint to world frame
                H_world_setpoint = np.matmul(
                    H_world_marker, self.H_marker_setpoint)

                # Prepare setpoint message and publish it
                setpoint_msg = Setpoint()
                setpoint_msg.position.x = H_world_setpoint[0, 3]
                setpoint_msg.position.y = H_world_setpoint[1, 3]
                setpoint_msg.position.z = H_world_setpoint[2, 3]
                setpoint_msg.velocity.x = 0.0
                setpoint_msg.velocity.y = 0.0
                setpoint_msg.velocity.z = 0.0
                setpoint_msg.orientation.x = 0.0
                setpoint_msg.orientation.y = 0.0
                setpoint_msg.orientation.z = np.arctan2(
                    H_world_setpoint[1, 0], H_world_setpoint[0, 0])

                traj = []
                traj.append(setpoint_msg)
                self.trajectory_pub.publish(traj)

                rp.loginfo('Setpoint sent')
                self.marker_setpoint_sent = True
            else:
                marker_current_pos = np.array([msg.marker_pose.position.x,
                                               msg.marker_pose.position.y,
                                               msg.marker_pose.position.z])

                d = marker_current_pos - self.marker_position

                if np.dot(d, d) > 0.02:
                    rp.logwarn(
                        'The marker\'s position changed too much. Sending new setpoint')
                    self.marker_setpoint_sent = False


if __name__ == '__main__':
    StateMachineNode()
