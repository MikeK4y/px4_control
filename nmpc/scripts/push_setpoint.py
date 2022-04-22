import time
import rospy as rp

from mavros_msgs.srv import SetMode, CommandBool
from std_srvs.srv import Trigger, SetBool

from px4_control_msgs.msg import Setpoint

# Start a node
rp.init_node('test_nmpc')
r = rp.Rate(10)

# Setup Publisher
setpoint_pub = rp.Publisher("/drone_setpoint", Setpoint, queue_size = 1)

# Setup services
setpoint_serv = rp.ServiceProxy('/go_to_start', Trigger)
ctrl_serv = rp.ServiceProxy('/enable_controller', SetBool)

setpoint_msg = Setpoint()
setpoint_msg.position.x = 0.0
setpoint_msg.position.y = 0.0
setpoint_msg.position.z = 1.0
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

# Load trajectory
setpoint_serv()
print('Trajectory loaded')
