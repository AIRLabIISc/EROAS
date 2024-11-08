#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class UnderwaterImageProcessor:
    def __init__(self, texture_image_path):
        self.node_name = "underwater_image_processor"
        rospy.init_node(self.node_name)

        # Initialize the CV bridge
        self.bridge = CvBridge()
        
        # Load the water texture image
        self.texture_image = cv2.imread(texture_image_path, cv2.IMREAD_COLOR)
        
        # Subscribe to the camera topic
        self.subscriber = rospy.Subscriber("/rexrov2/rexrov2/camera/camera_image", Image, self.callback, queue_size=10)
        
        # Publisher for the modified images
        self.publisher = rospy.Publisher("/rexrov2/underwater_image_1", Image, queue_size=10)

    def add_blue_green_tint(self, image):
        # Create a blue-green tint matrix
        tint = np.full_like(image, (50, 100, 150), dtype=np.uint8)  # BGR format
        tinted_image = cv2.addWeighted(image, 0.7, tint, 0.1, 0)
        return tinted_image

    def blend_with_texture(self, image):
        # Ensure the texture has the same size as the input image
        texture_resized = cv2.resize(self.texture_image, (image.shape[1], image.shape[0]))

        # Blend the images
        blended_image = cv2.addWeighted(image, 0.7, texture_resized, 0.3, 0)
        return blended_image

    def adjust_brightness_contrast(self, image, brightness=0, contrast=30):
        # Adjust brightness and contrast
        img = np.int16(image)
        img = img * (contrast / 127 + 1) - contrast + brightness
        img = np.clip(img, 0, 255)
        img = np.uint8(img)
        return img

    def callback(self, data):
        try:
            # Convert ROS Image message to CV2 format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Add blue-green tint
            tinted_image = self.add_blue_green_tint(cv_image)
            
            # Blend with water texture
            blended_image = self.blend_with_texture(tinted_image)
            
            # Adjust brightness and contrast
            underwater_image = self.adjust_brightness_contrast(blended_image)
            
            # Convert the modified CV2 image back to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(underwater_image, "bgr8")
            
            # Publish the image with the water effect
            self.publisher.publish(image_msg)

            rospy.loginfo(f" the image width : {image_msg.width} and image height : {image_msg.height}")
        
        except Exception as e:
            rospy.logerr(f"Error in UnderwaterImageProcessor callback: {e}")

if __name__ == '__main__':
    texture_image_path = "/home/user/P1.jpg"  # Update this path
    processor = UnderwaterImageProcessor(texture_image_path)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Underwater Image Processor Node")
    finally:
        cv2.destroyAllWindows()
