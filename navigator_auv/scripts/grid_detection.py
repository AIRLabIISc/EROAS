#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class YOLODetector:
    def __init__(self):
        self.node_name = "yolo_detector"
        rospy.init_node(self.node_name)
       
        # Initialize the CV bridge
        self.bridge = CvBridge()

        # Sonar parameters
        self.sonar_max_range = 10.0  # Maximum range of the sonar in meters
        self.sonar_interest_range = 7.0  # Range of interest in meters
       
        # Subscribe to the sonar image topic
        self.subscriber = rospy.Subscriber("/rexrov2/blueview_p900/sonar_image", Image, self.callback, queue_size=1)

        # Publisher for the processed images
        self.publisher = rospy.Publisher("/rexrov2/detected_objects", Image, queue_size=10)
   
    def callback(self, data):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
           
            # Convert the image to HSV color space
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
           
            # Define range for the reddish color and create a mask
            lower_red = np.array([0, 120, 70])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)
           
            lower_red = np.array([170, 120, 70])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)
           
            mask = mask1 + mask2
           
            # Apply Gaussian Blur
            blurred = cv2.GaussianBlur(mask, (5, 5), 0)
           
            # Find contours
            contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
           
            # Filter out small contours
            min_contour_area = 500  # Adjust this threshold as needed
            filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > min_contour_area]
           
            # Draw convex hulls around filtered contours
            hulls = [cv2.convexHull(contour) for contour in filtered_contours]
            cv2.drawContours(cv_image, hulls, -1, (0, 255, 0), 2)
           
            # Draw central vertical line and angled lines
            height, width, _ = cv_image.shape
            center_x = width // 2
            center_y = height - 1  # Bottom center

            # Grid parameters
            meters_per_pixel = self.sonar_max_range / height
            lines_of_interest = [(self.sonar_interest_range - i) / meters_per_pixel for i in range(5, 0, -1)]
           
            # Draw grid lines for range of interest (7m to 3m)
            for line in lines_of_interest:
                y = int(line)
                cv2.line(cv_image, (0, y), (width, y), (255, 255, 255), 2)

            # Central vertical line (blue dotted)
            for i in range(0, height, 10):
                cv2.line(cv_image, (center_x, center_y - i), (center_x, center_y - i - 5), (255, 0, 0), 2)
           
            # Angled lines (white or faint green filled)
            angles = [-25, -20, -15, -10, -5, 5, 10, 15, 20, 25]  # Angles in degrees
            for angle in angles:
                radians = np.deg2rad(angle)
                obstacle_within_range = False
                for i in range(0, int(self.sonar_interest_range / meters_per_pixel), 10):
                    start_y = center_y - i
                    end_y = start_y - 5
                    start_x = int(center_x + i * np.tan(radians))
                    end_x = int(center_x + (i + 5) * np.tan(radians))

                    # Check if there is an obstacle in this segment
                    if np.any(mask[start_y:end_y, start_x:end_x] > 0):
                        obstacle_within_range = True
                        break

                # Fill the beam with faint green color if no obstacle is within 7 meters
                if not obstacle_within_range:
                    polygon = np.array([
                        [center_x, center_y],
                        [int(center_x + self.sonar_interest_range / meters_per_pixel * np.tan(radians)), int(center_y - self.sonar_interest_range / meters_per_pixel)],
                        [int(center_x + self.sonar_interest_range / meters_per_pixel * np.tan(radians + np.deg2rad(5))), int(center_y - self.sonar_interest_range / meters_per_pixel)],
                        [center_x, center_y]
                    ])
                    cv2.fillPoly(cv_image, [polygon], (0, 255, 0, 50))  # Faint green color

                # Draw the dotted line
                for i in range(0, height, 10):
                    start_y = center_y - i
                    end_y = start_y - 5
                    start_x = int(center_x + i * np.tan(radians))
                    end_x = int(center_x + (i + 5) * np.tan(radians))
                    cv2.line(cv_image, (start_x, start_y), (end_x, end_y), (255, 255, 255), 2)
           
            # Convert the modified OpenCV image back to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
           
            # Publish the processed image
            self.publisher.publish(image_msg)
       
        except Exception as e:
            rospy.logerr(f"Error in YOLODetector callback: {e}")

if __name__ == '__main__':
    detector = YOLODetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down YOLO Detector Node")
    finally:
        cv2.destroyAllWindows()