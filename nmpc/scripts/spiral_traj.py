import rospy as rp
import math
import time

from px4_control_msgs.msg import Setpoint, Trajectory

# Create an ascending spiral trajectory
loops = 3.0
t_loop = 30.0
t_traj = loops * t_loop
control_rate = 10.0

radius = 1.0
start_alt = 0.75
final_alt = 2.25

steps = int(t_traj * control_rate)

z_velocity = (final_alt - start_alt) / t_traj

trajectory = Trajectory()

for i in range(steps):
    theta_i = (i * loops * 2 * math.pi) / steps

    ctheta = math.cos(theta_i)
    stheta = math.sin(theta_i)

    setpoint = Setpoint()

    setpoint.position.x = radius * ctheta
    setpoint.position.y = radius * stheta
    setpoint.position.z = start_alt + z_velocity * i / control_rate

    setpoint.velocity.x = radius * stheta
    setpoint.velocity.y = radius * ctheta
    setpoint.velocity.z = z_velocity

    setpoint.orientation.x = 0.0
    setpoint.orientation.y = 0.0
    setpoint.orientation.z = math.pi + theta_i

    trajectory.trajectory.append(setpoint)

# Add last point with zero velocities
setpoint = Setpoint()

setpoint.position.x = radius
setpoint.position.y = 0.0
setpoint.position.z = final_alt

setpoint.velocity.x = 0.0
setpoint.velocity.y = 0.0
setpoint.velocity.z = 0.0

setpoint.orientation.x = 0.0
setpoint.orientation.y = 0.0
setpoint.orientation.z = math.pi

trajectory.trajectory.append(setpoint)

# Publish trajectory
rp.init_node('trajectory_publisher')
trajectory.header.stamp = rp.Time.now()
pub = rp.Publisher("/drone_trajectory", Trajectory, queue_size=1, latch=True)
pub.publish(trajectory)
time.sleep(1.0)
