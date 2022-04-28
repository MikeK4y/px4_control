#!/usr/bin/python3
import threading

import rospy as rp

from mavros_msgs.srv import SetMode, CommandBool
from std_srvs.srv import Trigger, SetBool

from sensor_msgs.msg import Joy
from mavros_msgs.msg import PositionTarget, State

class JoyRCNode():
  def __init__(self, rate):
    rp.init_node('joy_rc_node')

    self.rate = rate

    # Check that services are available
    rp.loginfo('Checking that services are available')
    rp.wait_for_service('/mavros/set_mode')
    rp.wait_for_service('/mavros/cmd/arming')
    rp.wait_for_service('/enable_controller')
    rp.loginfo('MavROS and NMPC services are available')

    # Velocity cmds
    self.vel_x_cmd = 0.0
    self.vel_y_cmd = 0.0
    self.vel_z_cmd = 0.0
    self.vel_q_cmd = 0.0

    # Control switches
    self.arm_bttn = 0
    self.ofb_bttn = 0
    self.mpc_bttn = 0

    # Status
    self.connected = False
    self.armed = False
    self.mode = None
    self.mpc_enabled = False

    # Service clients
    self.set_mode_serv = rp.ServiceProxy('/mavros/set_mode', SetMode)
    self.arm_serv = rp.ServiceProxy('/mavros/cmd/arming', CommandBool)
    self.ctrl_serv = rp.ServiceProxy('/enable_controller', SetBool)

    # Subscribers
    self.state_sub = rp.Subscriber('/mavros/state', State, self.stateCallback, queue_size=1)
    self.joy_sub = rp.Subscriber('/joy', Joy, self.joyCallback, queue_size=1)

    # Publishers
    self.vel_cmd_pub = rp.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    t = threading.Thread(target=self.commandPublisher)
    t.start()

    rp.spin()

  def stateCallback(self, msg):
    self.connected = msg.connected
    self.armed = msg.armed
    self.mode = msg.mode

  def joyCallback(self, msg):
    self.vel_x_cmd =  msg.axes[1]
    self.vel_y_cmd =  msg.axes[0]
    self.vel_z_cmd = -msg.axes[2]
    self.vel_q_cmd =  msg.axes[3]

    self.arm_bttn = msg.buttons[0]
    self.ofb_bttn = msg.buttons[6]
    self.mpc_bttn = msg.buttons[2]

  def commandPublisher(self,):
    r = rp.Rate(self.rate)
    cmd_msg = PositionTarget()
    cmd_msg.coordinate_frame = PositionTarget().FRAME_BODY_NED
    cmd_msg.type_mask = PositionTarget().IGNORE_PX + \
                        PositionTarget().IGNORE_PY + \
                        PositionTarget().IGNORE_PZ + \
                        PositionTarget().IGNORE_AFX + \
                        PositionTarget().IGNORE_AFY + \
                        PositionTarget().IGNORE_AFZ + \
                        PositionTarget().IGNORE_YAW
    cmd_msg.velocity.x = 0.0
    cmd_msg.velocity.y = 0.0
    cmd_msg.velocity.z = 0.0
    cmd_msg.yaw_rate = 0.0

    while not rp.is_shutdown():
      if self.connected:
        # Make sure we are in the correct flight mode
        if self.ofb_bttn == 0.0 and self.mode != 'POSCTL':
          self.set_mode_serv(0, 'POSCTL')
        elif self.ofb_bttn == 1.0 and self.mode != 'OFFBOARD':
          self.set_mode_serv(0, 'OFFBOARD')

        # Arm if requested and not armed
        if self.arm_bttn == 1.0 and not self.armed:
          self.arm_serv(True)
        
        if self.mpc_bttn == 1.0 and not self.mpc_enabled:
          self.ctrl_serv(True)
          self.mpc_enabled = True
        elif self.mpc_bttn == 0.0 and self.mpc_enabled:
          self.ctrl_serv(False)
          self.mpc_enabled = False

        if self.armed and not self.mpc_enabled:
          cmd_msg.velocity.x = self.vel_x_cmd
          cmd_msg.velocity.y = self.vel_y_cmd
          cmd_msg.velocity.z = self.vel_z_cmd
          cmd_msg.yaw_rate = self.vel_q_cmd

          cmd_msg.header.stamp = rp.Time.now()
          self.vel_cmd_pub.publish(cmd_msg)

      else:
        rp.loginfo('Vehicle not connected')

      r.sleep()


if __name__ == '__main__':
    JoyRCNode(30)