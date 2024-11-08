#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


class CmdVelForwarderNode:
   def __init__(self):
       # Initialize the ROS node
       rospy.init_node('cmd_vel_forwarder_node', anonymous=True)
       
       # Publisher to /rexrov2/cmd_vel at 50 Hz
       self.cmd_vel_pub = rospy.Publisher('/rexrov2/cmd_vel', Twist, queue_size=1)
       
       # Subscriber to /rexrov2/cmd_vel_heading at 20 Hz
       self.cmd_vel_heading_sub = rospy.Subscriber('/rexrov2/cmd_vel_1', Twist, self.cmd_vel_heading_callback, queue_size=10)
       
       # Store the latest command velocity
       self.latest_cmd_vel = Twist()
       
       # To handle message rates
       self.publish_rate = rospy.Rate(7)  # 50 Hz
       self.receive_rate = rospy.Rate(5)  # 20 Hz


   def cmd_vel_heading_callback(self, msg):
       # Update the latest command velocity with the new message
       self.latest_cmd_vel = msg


   def run(self):
       while not rospy.is_shutdown():
           # Publish the latest command velocity
           self.cmd_vel_pub.publish(self.latest_cmd_vel)
           
           # Sleep to maintain the publishing rate
           self.publish_rate.sleep()


if __name__ == '__main__':
   try:
       node = CmdVelForwarderNode()
       node.run()
   except rospy.ROSInterruptException:
       pass