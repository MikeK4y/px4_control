#!/usr/bin/python3
import rospy as rp
import numpy as np
import random

from geometry_msgs.msg import PoseStamped


class FakeGPS():
    """
      Relays the MoCap position after adding noise and dropping most msgs
      Assuming that MoCap reports the true position this node reports:

      p_fake = p_true + random_walk + noise

      Does not affect the orientation
    """

    def __init__(self, random_walk_variance, noise_variance, drop_msgs=9, every_msgs=10):
        rp.init_node('fake_gps_node')

        # Drop rate
        self.drop_msgs = drop_msgs
        self.every_msgs = every_msgs
        self.msg_index = 0

        # Noise setup
        self.random_walk_std = pow(random_walk_variance, 0.5)
        self.random_walk = np.zeros(3)
        self.noise_std = pow(noise_variance, 0.5)

        # Subscribers
        self.mocap_sub = rp.Subscriber(
            '/vrpn_client_node/F550/pose', PoseStamped, self.mocapCallback, queue_size=1)

        # Publishers
        self.vision_pose_pub = rp.Publisher(
            '/mavros/vision_pose/pose', PoseStamped, queue_size=1)

        rp.spin()

    def mocapCallback(self, msg):
        self.msg_index += 1

        if self.msg_index > self.drop_msgs:
            # Update random walk noise
            self.random_walk[0] += random.gauss(0.0, self.random_walk_std)
            self.random_walk[1] += random.gauss(0.0, self.random_walk_std)
            self.random_walk[2] += random.gauss(0.0, self.random_walk_std)

            # Prepare message
            fake_msg = PoseStamped()
            fake_msg.header.stamp = msg.header.stamp
            fake_msg.header.frame_id = msg.header.frame_id
            fake_msg.pose.position.x = msg.pose.position.x + \
                self.random_walk[0] + random.gauss(0.0, self.noise_std)
            fake_msg.pose.position.y = msg.pose.position.y + \
                self.random_walk[1] + random.gauss(0.0, self.noise_std)
            fake_msg.pose.position.z = msg.pose.position.z + \
                self.random_walk[2] + random.gauss(0.0, self.noise_std)
            fake_msg.pose.orientation = msg.pose.orientation

            self.vision_pose_pub.publish(fake_msg)

        if self.msg_index >= self.every_msgs:
            self.msg_index = 0


if __name__ == '__main__':
    FakeGPS(1.0e-3, 1.0e-4)
