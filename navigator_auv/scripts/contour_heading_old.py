
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from marine_acoustic_msgs.msg import ProjectedSonarImage
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import cv2
import math
from cv_bridge import CvBridge
### if changing sonar range , change the correspoding number of bins and rnage , here its taken as 15 meters
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
        The image appears rotated by 90 degrees, so adjust accordingly.
        """
        # Image orientation correction
        #angle_per_beam = 90 / max_beams
        angle_per_beam= 0.0030739647336304188
        angle_rad = i * angle_per_beam + math.pi/4
        #angle_rad = math.radians(angle)
        distance = j*15/ max_bins
        #rospy.loginfo(f"Point - rad: {angle_rad}")
        # Compute Cartesian coordinates
        x = (distance * math.cos(angle_rad) )  # Use cos for x
        y = (distance * math.sin(angle_rad))  # Use sin for y
        #rospy.loginfo(f"Point - x: {x}, y: {y}")
            # Apply a 90-degree rotation correction
        # x_rotated = y  # Swapping x and y and flipping the new x
        # y_rotated = x   # The new y is the old x

        # # Convert to integer coordinates
        # x_int = int(x_rotated)
        # y_int = int(y_rotated)
        #returns distance in meters from sonar in cartesian
        return x, y

    def sonar_callback(self, msg):
        # Start timing
        start_time = rospy.get_time()

        # Convert the raw data into a numpy array
        data_array = np.frombuffer(msg.image.data, dtype=np.uint8)

        # Process the sonar data to calculate the average intensity
        points_contour = self.find_contour_points(data_array)

        # Create a blank image for visualization
        img_width = 1000
        img_height = 1000
        display_image = np.zeros((img_height, img_width, 3), dtype=np.uint8)

        # Plot contour points
        for (i, j) in points_contour:
            x, y = self.polar_to_cartesian(i, j, 512,598)
            #rospy.loginfo(f"Point - x: {x}, y: {y}")
            # x = min(max(x + img_width // 2, 0), img_width - 1)
            # y = min(max(y + img_height // 2, 0), img_height - 1)
            x=int(500+x*500/(15))
            y=int(1000-y*1000/15)
            #coordinate transformation for the image
            #rospy.loginfo(f"Point - x: {x}, y: {y}")
            cv2.circle(display_image, (x,y), 5, (0, 0, 255), -1)  # Red color circle
            
            
        # Convert the image to ROS format and publish
        ros_image = self.bridge.cv2_to_imgmsg(display_image, encoding='bgr8')
        self.publisher.publish(ros_image)

        # End timing
        end_time = rospy.get_time()
        processing_time = end_time - start_time  
        rospy.loginfo(f"Processing time: {processing_time:.4f} seconds")      

    def find_contour_points(self, data):
        no_of_beams = 512
        range_bin = 590 
        range_window=3
        intensity1 = 0
        
        window_average_intensity = []
        contour_points = []

        for i in range(10,no_of_beams,10):
           ##skipping 10 and 7 for computation reasons
            for j in range(250,range_bin-90,7):
                window_intensities = []

                for k in range(range_window):

                    intensity1 = data[i+ (j+k)*no_of_beams]
                    window_intensities.append(intensity1)
                    
                window_average_intensity = np.mean(window_intensities) 
                    
                if window_average_intensity > 15:
                        
                    contour_points.append((i, j)) 
                    #rospy.loginfo("count")
                    #rospy.loginfo(f"Point - i: {i}, j: {j}") 
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
