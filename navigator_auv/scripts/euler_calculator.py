#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import  Twist
import tf.transformations as tft
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

# Define the target point (x, y) in the global frame
TARGET_X = 50
TARGET_Y = 72

def pose_callback(pose_msg):
    global pub
    # Extract robot's pose
    pose = pose_msg.pose.pose
    x = pose.position.x
    y = pose.position.y
    orientation = pose.orientation

    # Convert quaternion to Euler angles
    roll, pitch, yaw = tft.euler_from_quaternion([
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ])

    # Compute the angle to the target point
    angle_to_target = math.atan2(TARGET_Y - y, TARGET_X - x)

    # Compute the angular error
    angular_error = angle_to_target - yaw

    # Normalize the angular error to be within the range [-pi, pi]
    angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

    # Define control gains
    Kp = 0.5
    

    # Compute the control command
    angular_vel = Kp * angular_error

    # Publish the velocity command
    # twist_msg = Twist()
    # twist_msg.angular.z = angular_vel
    # cmd_vel_pub.publish(twist_msg)
    
    msg = Float64()
    msg.data = angular_error
    pub.publish(msg)

def move_robot():
    global pub

    rospy.init_node('move_robot_to_target')

   # Subscribe to the /pose_gt topic
    rospy.Subscriber('/rexrov2/pose_gt', Odometry, pose_callback)
    

    # Publisher for velocity commands
    #cmd_vel_pub = rospy.Publisher('/rexrov2/cmd_vel', Twist, queue_size=10)
    pub = rospy.Publisher('/rexrov2/global_angle', Float64, queue_size=10)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
