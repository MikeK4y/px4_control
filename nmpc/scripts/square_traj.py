import rospy as rp
import math
import time

from px4_control_msgs.msg import Setpoint, Trajectory

# Create a square trajectory
#
# A----^-x--D
# -----|----
# -<--------
# --y-------
# B---------C

loops = 1.0
t_loop = 60.0
t_traj = loops * t_loop
control_rate = 10.0

width = 1.75
height = 1.25

max_horizontal_velocity = width / (t_loop / 4.0)

loop_steps = int(t_loop * control_rate)
steps = int(t_traj * control_rate)

pos_x = width / 2.0
pos_y = width / 2.0
vel_x = 0.0
vel_y = 0.0

trajectory = Trajectory()

for i in range(steps):

    # A to B
    if (i % loop_steps) < 0.25 * loop_steps:
        pos_x = pos_x - (max_horizontal_velocity / control_rate)
        pos_y = pos_y
        vel_x = -max_horizontal_velocity
        vel_y = 0.0
    # B to C
    elif (i % loop_steps) < 0.5 * loop_steps:
        pos_x = pos_x
        pos_y = pos_y - (max_horizontal_velocity / control_rate)
        vel_x = 0.0
        vel_y = -max_horizontal_velocity
    # C to D
    elif (i % loop_steps) < 0.75 * loop_steps:
        pos_x = pos_x + (max_horizontal_velocity / control_rate)
        pos_y = pos_y
        vel_x = max_horizontal_velocity
        vel_y = 0.0
    # D to A
    elif (i % loop_steps) < loop_steps:
        pos_x = pos_x
        pos_y = pos_y + (max_horizontal_velocity / control_rate)
        vel_x = 0.0
        vel_y = max_horizontal_velocity


    setpoint = Setpoint()

    setpoint.position.x = pos_x
    setpoint.position.y = pos_y
    setpoint.position.z = height

    setpoint.velocity.x = vel_x
    setpoint.velocity.y = vel_y
    setpoint.velocity.z = 0.0

    setpoint.orientation.x = 0.0
    setpoint.orientation.y = 0.0
    setpoint.orientation.z = 0.0

    trajectory.trajectory.append(setpoint)

# Add last point with zero velocities
setpoint = Setpoint()

setpoint.position.x = width / 2.0
setpoint.position.y = width / 2.0
setpoint.position.z = height

setpoint.velocity.x = 0.0
setpoint.velocity.y = 0.0
setpoint.velocity.z = 0.0

setpoint.orientation.x = 0.0
setpoint.orientation.y = 0.0
setpoint.orientation.z = 0.0

trajectory.trajectory.append(setpoint)

# Publish trajectory
rp.init_node('trajectory_publisher')
trajectory.header.stamp = rp.Time.now()
pub = rp.Publisher("/drone_trajectory", Trajectory, queue_size=1, latch=True)
pub.publish(trajectory)
time.sleep(1.0)
