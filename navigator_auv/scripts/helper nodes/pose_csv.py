#!/usr/bin/env python

import rospy
import csv
import tf.transformations as tft
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class PoseToCSV:
    def __init__(self):
        # Initialize the node
        rospy.init_node('pose_to_csv', anonymous=True)
        
        # Open a CSV file to write pose data
        self.file = open('/home/user/pose_data.csv', mode='w')
        self.csv_writer = csv.writer(self.file)
        
        # Write the header row to the CSV file
        self.csv_writer.writerow(['Timestamp', 'Position_X', 'Position_Y', 'Position_Z', 'Yaw', 'H'])
        
        # Initialize variable to store the latest h value
        self.h_value = None
        
        # Subscribe to the pose topic
        self.pose_subscriber = rospy.Subscriber('/rexrov2/pose_gt', Odometry, self.pose_callback)
        
        # Subscribe to the h topic
        self.h_subscriber = rospy.Subscriber('/rexrov2/current_h', Float64, self.h_callback)
        
        # Keep the node running
        rospy.spin()
    
    def pose_callback(self, msg):
        # Extract pose data
        timestamp = msg.header.stamp.to_sec()
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to Euler angles
        euler = tft.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        
        # Extract yaw (which is the third angle in the Euler angles tuple)
        yaw = euler[2]
        
        # Use the latest h value (if available)
        h = self.h_value if self.h_value is not None else 'N/A'
        
        # Write pose data to CSV file
        self.csv_writer.writerow([
            timestamp,
            position.x,
            position.y,
            position.z,
            yaw,
            h
        ])
    
    def h_callback(self, msg):
        # Update the latest h value
        self.h_value = msg.data
    
    def __del__(self):
        # Close the CSV file when the node is shut down
        self.file.close()

if __name__ == '__main__':
    try:
        PoseToCSV()
    except rospy.ROSInterruptException:
        pass
