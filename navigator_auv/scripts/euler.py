#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf.transformations as tft

def callback(pose_msg):
    # Extract the quaternion from the message
    quat = pose_msg.pose.pose.orientation
    qx = quat.x
    qy = quat.y
    qz = quat.z
    qw = quat.w

    # Convert quaternion to Euler angles
    euler = tft.euler_from_quaternion([qx, qy, qz, qw])
    
    # euler is a tuple (roll, pitch, yaw)
    roll, pitch, yaw = euler

    # Print the Euler angles
    rospy.loginfo("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw)

def listener():
    rospy.init_node('pose_euler_calculator', anonymous=True)
    
    # Subscribe to the /pose_gt topic
    rospy.Subscriber('/rexrov2/pose_gt', Odometry, callback)
   

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()
