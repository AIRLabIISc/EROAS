#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import torch

class YOLODetector:
    def __init__(self):
        self.node_name = "yolo_detector"
        rospy.init_node(self.node_name)
        
        # Initialize YOLO model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        
        # Initialize the CV bridge
        self.bridge = CvBridge()
        
        # Subscribe to the camera topic
        self.subscriber = rospy.Subscriber("/rexrov2/rexrov2/camera/camera_image", Image, self.callback, queue_size=1)
        
        # Publisher for the images with bounding boxes
        self.publisher = rospy.Publisher("/new/detected_objects", Image, queue_size=10)
        
        # Publisher for velocity commands
        self.cmd_vel_publisher = rospy.Publisher('rexrov2/cmd_vel', Twist, queue_size=10)
        
        # Image properties
        self.img_width = None
        self.img_height = None
    
    def callback(self, data):
        # Convert ROS Image message to CV2 format
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        self.img_width = data.width
        self.img_height = data.height
        
        # Run YOLO model
        results = self.model(cv_image)
        
        # Variable to track the closest object
        closest_object = None
        min_distance = float('inf')
        
        # Draw bounding boxes on the image
        for *xyxy, conf, cls in results.xyxy[0]:
            label = f'{results.names[int(cls)]} {conf:.2f}'
            x_center = (xyxy[0] + xyxy[2]) / 2
            y_center = (xyxy[1] + xyxy[3]) / 2
            
            # Find the closest object to the center of the image
            distance = abs(self.img_width / 2 - x_center)
            if distance < min_distance:
                min_distance = distance
                closest_object = (x_center, y_center, label)
                
            # Draw the bounding box
            cv2.rectangle(cv_image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (255, 0, 0), 2)
            cv2.putText(cv_image, label, (int(xyxy[0]), int(xyxy[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
        
        # Convert the modified CV2 image back to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        
        # Publish the image with detections
        self.publisher.publish(image_msg)
        
        # Control the rover to head towards the detected obstacle
        if closest_object:
            x_center, y_center, _ = closest_object
            self.adjust_heading(x_center,y_center)
    
    def adjust_heading(self, object_x,object_y):
        # Compute error from the center of the image
        error_heading = self.img_width / 2 - object_x
        error_height=self.img_height / 2 - object_y
        
        # Proportional control gain
        k_p = 0.005
        k_h=0.05
        
        # Compute the angular velocity
        angular_z = k_p * error_heading
        linear_z=k_h * error_height
        
        # Create and publish the Twist message
        twist = Twist()
        twist.angular.z = angular_z
        twist.linear.z=linear_z
        self.cmd_vel_publisher.publish(twist)
        # Log the dominant slope
        rospy.loginfo(f"heading error: {error_heading:.4f},z erro: {error_height:.4f}")


if __name__ == '__main__':
    detector = YOLODetector()
    rospy.spin()
