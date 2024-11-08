#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def sonar_publisher():
    # Initialize the ROS node
    rospy.init_node('sonar_zero_publisher', anonymous=True)

    # Create a publisher for the '/rexrov2/sonar/moving' topic
    pub = rospy.Publisher('/rexrov2/sonar/moving', Float64, queue_size=10)

    # Set the publishing rate (e.g., 10 Hz)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Create a Float64 message with value 0
        msg = Float64()
        msg.data = 0.0
        
        # Publish the message
        pub.publish(msg)
        
        
        # Sleep to maintain the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        sonar_publisher()
    except rospy.ROSInterruptException:
        pass
