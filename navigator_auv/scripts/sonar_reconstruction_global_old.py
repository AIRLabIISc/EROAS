#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from marine_acoustic_msgs.msg import ProjectedSonarImage
from sensor_msgs.msg import PointCloud2
import numpy as np
import math
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from nav_msgs.msg import Odometry

class SonarIntensityPublisher:
    def __init__(self):
        rospy.init_node('sonar_intensity_publisher', anonymous=True)

       
        # Subscriber for the sonar pivot angle
        self.pivot_sub = rospy.Subscriber('/rexrov2/sonar_joint_position_controller/command', Float64, self.pivot_callback)

        # Subscribe to the /rexrov2/pose_gt topic
        self.height_sub = rospy.Subscriber('/rexrov2/pose_gt',  Odometry, self.height_callback)

         # Subscriber for the sonar image data
        self.sonar_sub = rospy.Subscriber('/rexrov2/blueview_p900/sonar_image_raw', ProjectedSonarImage, self.sonar_callback)


        # Publisher for the processed point cloud
        self.publisher = rospy.Publisher('/rexrov2/point_cloud', PointCloud2, queue_size=10)

        # Publisher for the history point cloud
        self.history_publisher = rospy.Publisher('/rexrov2/point_cloud_history', PointCloud2, queue_size=10)

        # Initialize variables
        self.pivot_angle = 0.0  # Initialize pivot angle
        self.point_cloud_history = []  # Store point cloud history

        # Set the loop rate (e.g., 10 Hz)
        self.rate = rospy.Rate(1)

    def height_callback(self,data):
        # Extract the z position from the Pose message
        self.z_position = data.pose.pose.position.z 
        self.y_position = data.pose.pose.position.y 
        self.x_position = data.pose.pose.position.x 
        self.quat = data.pose.pose.orientation
        self.translation_vector= [self.x_position,self.y_position,self.z_position]
        # qx = quat.x
        # qy = quat.y
        # qz = quat.z
        # qw = quat.w
        #rospy.loginfo("Current z position: %f", self.z_position)

    def pivot_callback(self, msg):
        self.pivot_angle = msg.data
       

    def polar_to_cartesian(self, i, j, max_beams, max_bins):
        """
        Convert polar coordinates (i, j) to Cartesian coordinates in rover frame.
        The image appears rotated by 90 degrees, so adjust accordingly.
        """ ## this is in local frame of the robot
        angle_per_beam = 0.0030739647336304188
        angle_rad = i * angle_per_beam + math.pi / 4
        distance = j * 15 / max_bins
        x = distance * math.cos(angle_rad) 
        y = distance * math.sin(angle_rad) * math.cos(self.pivot_angle) 
        z = distance * math.sin(angle_rad) * math.sin(self.pivot_angle)   # Use the pivot angle as the z-coordinate
        return x, y, z
    
    def quaternion_to_rotation_matrix(self,q):
        w, x, y, z = q.x,q.y,q.z,q.w
        R = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x**2 + z**2), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)]
        ])
        return R
    
    def local_to_global(self,local_points, rotation_matrix, translation_vector,additional_translation):
        global_points = []
        for point in local_points:
            local_point = np.array(point)
            global_point = rotation_matrix @ local_point + translation_vector
            #global_point += additional_translation
            #rospy.loginfo(f"points:{global_point[0]},{global_point[1]},{global_point[2]}")
            global_points.append(global_point)
        return np.array(global_points)
    
        # Convert quaternion to rotation matrix
    

   

    def sonar_callback(self, msg):
        # Start timing
        start_time = rospy.get_time()

        
        # Convert the raw data into a numpy array
        data_array = np.frombuffer(msg.image.data, dtype=np.uint8)

        # Process the sonar data to calculate the average intensity
        points_contour = self.find_contour_points(data_array)

        min_range=self.find_min(points_contour)
        
        if min_range:
            rospy.loginfo(f"min_range: {min_range:.4f} at angle: {self.pivot_angle :.4f}   ")
        # Convert to Cartesian coordinates
        points_cartesian = [self.polar_to_cartesian(i, j, 512, 598) for (i, j) in points_contour] ##local pointd wrt rover

        rotation_matrix = self.quaternion_to_rotation_matrix(self.quat)
        # Transform points from local to global coordinates

        additional_translation = [0, 0 ,0]

        global_points = self.local_to_global(points_cartesian, rotation_matrix, self.translation_vector,additional_translation)

        

        
        # Create and publish PointCloud2 message
        header = msg.header
        #header.frame_id = "blueview_p900_visual_ray_link" 
        header.frame_id = "rexrov2/base_link" 
        point_cloud_msg = create_cloud_xyz32(header, global_points)
        self.publisher.publish(point_cloud_msg)

        # Store the point cloud in history
        self.point_cloud_history.extend(global_points)

        # Publish history point cloud (if needed)
        if len(self.point_cloud_history) > 0:
            history_header = msg.header
            #history_header.frame_id = "blueview_p900_visual_ray_link"
            history_header.frame_id = "world"
            history_point_cloud_msg = create_cloud_xyz32(history_header, self.point_cloud_history)
            self.history_publisher.publish(history_point_cloud_msg)

        # End timing
        end_time = rospy.get_time()
        processing_time = end_time - start_time  
        #rospy.loginfo(f"Processing time: {processing_time:.4f} seconds")
    def find_min(self, points):
        """
        Find the minimum j value from the list of contour points.
        """
        if not points:
            return None  # Return None if there are no points
        
        # Extract j values from the points and find the minimum
        min_j = min(j for i, j in points)
        return min_j
        
    def find_contour_points(self, data):
        no_of_beams = 512
        range_bin = 590 
        range_window = 3
        
        contour_points = []

        for i in range(5, no_of_beams, 1):
            ##skipping 10 and 7 for computation reasons
            for j in range(150, range_bin - 90, 4):
                window_intensities = []

                for k in range(range_window):
                    intensity1 = data[i + (j + k) * no_of_beams]
                    window_intensities.append(intensity1)
                    
                window_average_intensity = np.mean(window_intensities)
                    
                if window_average_intensity > 15:
                    contour_points.append((i, j)) 
                    break
                    
        return contour_points      

    def run(self):
        rospy.loginfo("Sonar Intensity Publisher Node Running")
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SonarIntensityPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
