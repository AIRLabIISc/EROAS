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
        self.image_counter = 0
        self.cv_cam_image = None

        # Initialize camera image filename
        self.cam_image_filename = ''

        # Subscribe to the camera image topic
        self.cam_image_sub = rospy.Subscriber('/rexrov2/rexrov2/camera/camera_image', Image, self.camera_callback)

        # Ensure the directory exists
        if not os.path.exists(self.image_directory):
            os.makedirs(self.image_directory)

        # Timer to save images every second
        self.timer = rospy.Timer(rospy.Duration(5), self.save_images)

    def camera_callback(self, data):
        try:
            # Convert the ROS Image message to a cv2 image
            self.cv_cam_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            # Update the camera image filename
            self.cam_image_filename = os.path.join(self.image_directory, f'camera_{self.image_prefix}_{self.image_counter}.png')
        except Exception as e:
            rospy.logerr(f"Error processing camera image: {e}")

    def save_images(self, event):
        if self.cv_cam_image is not None:
            try:
                # Save the camera image
                cv2.imwrite(self.cam_image_filename, self.cv_cam_image)
                rospy.loginfo(f"Saved camera image {self.cam_image_filename}")

                # Increment the image counter
                self.image_counter += 1

                # Optionally, clear the image to save space or for further processing
                self.cv_cam_image = None

            except Exception as e:
                rospy.logerr(f"Error saving camera image: {e}")

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
