#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64

def move_sonar():
    rospy.init_node('move_sonar')
    pub = rospy.Publisher('/rexrov2/sonar_joint_position_controller/command', Float64, queue_size=10)

    rospy.sleep(1)  # Wait for the publisher to be ready

    rate = rospy.Rate(10)  # 10 Hz for smoother motion

    amplitude = 0.77  # Maximum angle (radians) from center
    frequency = 0.05    # Frequency of the oscillation (Hz)

    while not rospy.is_shutdown():
        # Calculate the time
        current_time = rospy.get_time()
        
        # Calculate the sinusoidal position
        position = amplitude * math.sin(2 * math.pi * frequency * current_time)
        
        # Create and publish the Float64 message
        msg = Float64()
        msg.data = position
        pub.publish(msg)
        
        ##rospy.loginfo("Publishing position: %.2f radians", position)
        
        # Sleep to maintain the rate
        rate.sleep()

if __name__ == '__main__':
    try:
        move_sonar()
    except rospy.ROSInterruptException:
        pass
