#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomToPath:
    def __init__(self):
        self.path_pub = rospy.Publisher('/rovio/path', Path, latch=True, queue_size=10)
        self.odom_sub = rospy.Subscriber('/est_odometry', Odometry, self.odom_cb, queue_size=10)
        self.path = Path()

        self.path_pub_gt = rospy.Publisher('/ground_truth/path', Path, latch=True, queue_size=10)
        self.odom_sub_gt = rospy.Subscriber('/hummingbird/ground_truth/odometry', Odometry, self.odom_cb_gt, queue_size=10)
        self.path_gt = Path()

    def odom_cb(self, msg):
        cur_pose = PoseStamped()
        cur_pose.header = msg.header
        cur_pose.pose = msg.pose.pose
        cur_pose.pose.position.x = cur_pose.pose.position.x  # Starting x position is x0 = -3 meters
        cur_pose.pose.position.z = cur_pose.pose.position.z  # Starting z position is z0 = 3 meters
        self.path.header = msg.header
        self.path.poses.append(cur_pose)
        self.path_pub.publish(self.path)

    def odom_cb_gt(self, msg):
        cur_pose = PoseStamped()
        cur_pose.header = msg.header
        cur_pose.pose = msg.pose.pose
        self.path_gt.header = msg.header
        self.path_gt.poses.append(cur_pose)
        self.path_pub_gt.publish(self.path_gt)

if __name__ == '__main__':
    rospy.init_node('odom_to_path')
    odom_to_path = OdomToPath()
    rospy.spin()
