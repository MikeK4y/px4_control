import rospy as rp
import math
import time

from px4_control_msgs.msg import Setpoint, Trajectory

# Create a back and forth trajectory with small brakes at the ends
loops = 1.0
t_loop = 40.0
t_break = 5.0

t_traj = loops * t_loop

control_rate = 10.0

length = 2.0
altitude = 0.9

steps = int(t_traj * control_rate)
loop_steps = int(t_loop * control_rate)
break_steps = int(t_break * control_rate)

trajectory = Trajectory()

for i in range(steps):
    theta_i = (i * loops * 2 * math.pi) / steps

    setpoint = Setpoint()

    setpoint.position.y = 0.0
    setpoint.position.z = altitude

    setpoint.velocity.y = 0.0
    setpoint.velocity.z = 0.0

    setpoint.orientation.x = 0.0
    setpoint.orientation.y = 0.0
    setpoint.orientation.z = 0.0

    setpoint.position.x = 0.5 * length * math.cos(theta_i)
    setpoint.velocity.x = -0.5 * length * math.sin(theta_i)

    # Time for break
    if (i % int(0.5 * loop_steps)) == 0:
        for j in range(break_steps):
            trajectory.trajectory.append(setpoint)
    # Movement
    else:
        trajectory.trajectory.append(setpoint)

# Publish trajectory
rp.init_node('trajectory_publisher')
trajectory.header.stamp = rp.Time.now()
pub = rp.Publisher("/drone_trajectory", Trajectory, queue_size=1, latch=True)
pub.publish(trajectory)
time.sleep(1.0)
