import time
import rospy as rp

from mavros_msgs.srv import SetMode, CommandBool
from std_srvs.srv import SetBool

from px4_control_msgs.msg import Setpoint

# Start a node
rp.init_node('test_nmpc')
r = rp.Rate(10)

# Check that services are available
rp.wait_for_service('/mavros/set_mode')
print('/mavros/set_mode is available')

rp.wait_for_service('/mavros/cmd/arming')
print('/mavros/cmd/arming is available')

rp.wait_for_service('/enable_controller')
print('/enable_controller is available')

# Setup Publisher
setpoint_pub = rp.Publisher("/drone_setpoint", Setpoint, queue_size = 1)

setpoint_msg = Setpoint()
setpoint_msg.position.x = 1.5
setpoint_msg.position.y = -1.5
setpoint_msg.position.z = 0.5
setpoint_msg.velocity.x = 0.0
setpoint_msg.velocity.y = 0.0
setpoint_msg.velocity.z = 0.0
setpoint_msg.orientation.x = 0.0
setpoint_msg.orientation.y = 0.0
setpoint_msg.orientation.z = 0.0

for i in range(10):
  setpoint_pub.publish(setpoint_msg)
  r.sleep()

print('Published setpoint')

# Create service clients
set_mode_serv = rp.ServiceProxy('/mavros/set_mode', SetMode)
arm_serv = rp.ServiceProxy('/mavros/cmd/arming', CommandBool)
ctrl_serv = rp.ServiceProxy('/enable_controller', SetBool)

# Set to Offboard control
res = set_mode_serv(0, 'POSCTL')
time.sleep(0.5)
for i in range(50):
  res = set_mode_serv(0, 'OFFBOARD')
  if (res.mode_sent):
    print('Mode changed to OFFBOARD')
    break
  time.sleep(0.2)

# Arm
arm_serv(True)
time.sleep(0.5)
print('Armed')

# Enable controller
ctrl_serv(True)
print('Controller enabled')