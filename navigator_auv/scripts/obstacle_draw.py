#!/usr/bin/env python3

import rospy
import csv
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs.point_cloud2 import read_points
from message_filters import Subscriber, ApproximateTimeSynchronizer

class DataLogger:
    def __init__(self):
        rospy.init_node('data_logger', anonymous=True)

        # Subscribers
        self.point_cloud_sub = Subscriber('/rexrov2/point_cloud', PointCloud2)
        self.pose_sub = Subscriber('/rexrov2/pose_gt', Odometry)

        # Synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.point_cloud_sub, self.pose_sub], queue_size=10, slop=1
        )
        self.sync.registerCallback(self.synchronized_callback)

        # File to save the data
        self.csv_file = '/home/user/data_log.csv'  # Update this path as needed

        # Check if the CSV file is empty and open it for appending
        self.csv_file_handle = open(self.csv_file, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file_handle)

        # Write header if the file was just created and is empty
        if self.is_file_empty(self.csv_file):
            self.csv_writer.writerow(['Timestamp', 'Pose_X', 'Pose_Y', 'Pose_Z', 'Point_X', 'Point_Y', 'Point_Z'])

    def is_file_empty(self, filepath):
        """Check if the file is empty."""
        with open(filepath, 'r') as f:
            return f.read(1) == ''

    def synchronized_callback(self, point_cloud_msg, pose_msg):
        try:
            # Get timestamp
            timestamp = rospy.Time.now()

            # Extract pose data
            pose_x = pose_msg.pose.pose.position.x
            pose_y = pose_msg.pose.pose.position.y
            pose_z = pose_msg.pose.pose.position.z

            # Extract point cloud data
            point_cloud_points = list(read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True))

            # Write data to CSV
            for point in point_cloud_points:
                self.csv_writer.writerow([timestamp.to_sec(), pose_x, pose_y, pose_z, point[0], point[1], point[2]])

        except Exception as e:
            rospy.logerr(f"Error in synchronized callback: {e}")

    def run(self):
        rospy.loginfo("Data Logger Node Running")
        rospy.spin()

    def __del__(self):
        # Close CSV file when node shuts down
        if hasattr(self, 'csv_file_handle') and self.csv_file_handle:
            self.csv_file_handle.close()

if __name__ == '__main__':
    try:
        node = DataLogger()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")
    finally:
        # Ensure the CSV file is closed if there's an interruption
        if 'node' in locals():
            del node
