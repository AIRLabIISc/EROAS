#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import argparse

class ImageSaver:
    def __init__(self, image_prefix):
        # Create a CvBridge object
        self.bridge = CvBridge()

        # Hardcoded directory path
        self.image_directory = '/home/user/sonar_images'
        self.image_prefix = image_prefix
        self.sonar_image_counter = 0
        self.camera_image_counter=1
        self.image_counter=0
        # Initialize camera image filename
        self.cam_image_filename = ''
        self.cv_cam_image= None
        # Subscribe to the image topic
       
        self.cam_image_sub = rospy.Subscriber('/rexrov2/rexrov2/camera/camera_image', Image, self.camera_callback)
        self.image_sub = rospy.Subscriber('/rexrov2/blueview_p900/sonar_image', Image, self.callback)

        # Ensure the directory exists
        if not os.path.exists(self.image_directory):
            os.makedirs(self.image_directory)

    def camera_callback(self, data):
        try:
            # Convert the ROS Image message to a cv2 image
            self.cv_cam_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Create the image filename
            self.cam_image_filename = os.path.join(self.image_directory, f'camera_{self.image_prefix}.png')
            
            #rospy.loginfo(f"Saved camera image {image_filename}")

            # Increment the camera image counter
            

        except Exception as e:
            rospy.logerr(f"Error processing camera image: {e}")

    def callback(self, data):
        try:
            # Convert the ROS Image message to a cv2 image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Create the image filename
            image_filename = os.path.join(self.image_directory, f'sonar_{self.image_prefix}.png')
            
            cv2.imwrite(image_filename, cv_image) ##sonar_image
            cv2.imwrite(self.cam_image_filename, self.cv_cam_image)  #camera image
            rospy.loginfo(f"Both Saved image {image_filename}")

            # Increment the image counter
            self.image_counter += 1

            # Shutdown the node if desired (for example, after saving one image)
            if self.image_counter > 0:
                rospy.signal_shutdown("Image Saved, Shutting down")

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Image Saver Node')
    parser.add_argument('--image_prefix', type=str, default='image', help='Prefix for image filenames')
    args = parser.parse_args()

    # Initialize the ROS node
    rospy.init_node('image_saver', anonymous=True)

    try:
        image_saver = ImageSaver(args.image_prefix)
        image_saver.spin()
    except rospy.ROSInterruptException:
        pass
