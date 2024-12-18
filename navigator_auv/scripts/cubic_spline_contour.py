#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from marine_acoustic_msgs.msg import ProjectedSonarImage
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import cv2
import math
import scipy.interpolate as spi
from cv_bridge import CvBridge


### if changing sonar range , change the corresponding number of bins and range, here it's taken as 15 meters
class SonarIntensityPublisher:
    def __init__(self):
        rospy.init_node('sonar_intensity_publisher', anonymous=True)

        # Subscriber for the sonar image data
        self.sonar_sub = rospy.Subscriber('/rexrov2/blueview_p900/sonar_image_raw', ProjectedSonarImage, self.sonar_callback)

        # Publisher for the processed images
        self.publisher = rospy.Publisher('/rexrov2/detected_objects', Image, queue_size=10)

        # Initialize CvBridge for converting images
        self.bridge = CvBridge()

        # Set the loop rate (e.g., 10 Hz)
        self.rate = rospy.Rate(5)

    def polar_to_cartesian(self, i, j, max_beams, max_bins):
        """
        Convert polar coordinates (i, j) to Cartesian coordinates.
        
        """
        angle_per_beam = 0.0030739647336304188
        angle_rad = i * angle_per_beam + math.pi/4
        distance = j * 15 / max_bins
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return x, y

    def print_spline_equations(self, cs):
        # Extract the coefficients of the cubic spline
        c0, c1, c2, c3 = cs.c

        # Number of segments
        num_segments = len(c0)

        print("Cubic Spline Equations:")
        for i in range(num_segments):
            a = c3[i]
            b = c2[i]
            c = c1[i]
            d = c0[i]
            print(f"Segment {i+1}:")
            print(f"S{i}(x) = {a:.4f}(x - x{i})^3 + {b:.4f}(x - x{i})^2 + {c:.4f}(x - x{i}) + {d:.4f}")
            print()

    def find_and_plot_curve(self, x_coords, y_coords):
        # Ensure x_coords are strictly increasing
        sorted_points = sorted(zip(x_coords, y_coords))
        x_coords, y_coords = zip(*sorted_points)

        # Check if x_coords is strictly increasing
        if any(x2 <= x1 for x1, x2 in zip(x_coords, x_coords[1:])):
            rospy.logwarn("x_coords are not strictly increasing. Removing duplicates.")
            # Remove duplicates
            unique_points = []
            for x, y in zip(x_coords, y_coords):
                if not unique_points or unique_points[-1][0] != x:
                    unique_points.append((x, y))
            x_coords, y_coords = zip(*unique_points)

        # Perform cubic spline interpolation
        cs = spi.CubicSpline(x_coords, y_coords, bc_type='natural')
        self.print_spline_equations(cs)

        # Generate points for the fitted curve
        x_fit = np.linspace(min(x_coords), max(x_coords), num=500)
        y_fit = cs(x_fit)

        # Create a blank image for visualization
        img_width = 1000
        img_height = 1000
        display_image = np.zeros((img_height, img_width, 3), dtype=np.uint8)

        # Plot the fitted curve
        for i in range(len(x_fit) - 1):
            # Transform to image coordinates
            x1 = int(500 + x_fit[i] * 500 / 15)
            y1 = int(1000 - y_fit[i] * 1000 / 15)
            x2 = int(500 + x_fit[i + 1] * 500 / 15)
            y2 = int(1000 - y_fit[i + 1] * 1000 / 15)
            cv2.line(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green color line

        # Convert the image to ROS format and publish
        ros_image = self.bridge.cv2_to_imgmsg(display_image, encoding='bgr8')
        self.publisher.publish(ros_image)

        return cs

    def sonar_callback(self, msg):
        # Start timing
        start_time = rospy.get_time()

        # Convert the raw data into a numpy array
        data_array = np.frombuffer(msg.image.data, dtype=np.uint8)

        # Process the sonar data to calculate the average intensity
        points_contour = self.find_contour_points(data_array)

        # Convert to Cartesian coordinates
        points_cartesian = [self.polar_to_cartesian(i, j, 512, 598) for (i, j) in points_contour]

        # Extract x and y coordinates
        x_coords, y_coords = zip(*points_cartesian)

        # Find and plot the curve
        cs = self.find_and_plot_curve(x_coords, y_coords)

        # End timing
        end_time = rospy.get_time()
        processing_time = end_time - start_time  
        rospy.loginfo(f"Processing time: {processing_time:.4f} seconds")

    def find_contour_points(self, data):
        no_of_beams = 512
        range_bin = 590 
        range_window = 3
        
        contour_points = []

        for i in range(10, no_of_beams, 10):
            ##skipping 10 and 7 for computation reasons
            for j in range(250, range_bin - 90, 7):
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
