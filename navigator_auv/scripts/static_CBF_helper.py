#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped



class CBFHelper:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('CBF_Helper', anonymous=True)

        # Publisher for the unified thruster command
        self.pub = rospy.Publisher('/rexrov2/thruster_manager/input_stamped', WrenchStamped, queue_size=10)

        # Subscribers to the two thruster command topics
        self.sub1 = rospy.Subscriber('/rexrov2/thruster_manager/input_stamped_1', WrenchStamped, self.callback1)
        self.sub2 = rospy.Subscriber('/rexrov2/thruster_manager/input_stamped_2', WrenchStamped, self.callback2)

        # Variable to store the last received message from input_stamped_1
        self.last_msg1 = None

    def callback1(self, data):
        # Callback for input_stamped_1, updates the last received message
        #cbf
        self.last_msg1 = data

    def callback2(self, data):
        # Callback for input_stamped_2, publishes based on the availability of input_stamped_1
        # PID data
        if self.last_msg1 is None:
            # If no message has been received from input_stamped_1, publish data from input_stamped_2
            self.pub.publish(data)
        else:
            # If a message has been received from input_stamped_1, publish that
            self.pub.publish(self.last_msg1)
            self.last_msg1 = None  # Reset after publishing

if __name__ == '__main__':
    try:
        tm = CBFHelper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass