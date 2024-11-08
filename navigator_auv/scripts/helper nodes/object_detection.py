#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
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
    
    def callback(self, data):
        # Convert ROS Image message to CV2 format
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # Run YOLO model
        results = self.model(cv_image)
        
        # Draw bounding boxes on the image
        for *xyxy, conf, cls in results.xyxy[0]:
            label = f'{results.names[int(cls)]} {conf:.2f}'
            cv2.rectangle(cv_image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (255, 0, 0), 2)
            cv2.putText(cv_image, label, (int(xyxy[0]), int(xyxy[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
        
        # Convert the modified CV2 image back to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        
        # Publish the image with detections
        self.publisher.publish(image_msg)

if __name__ == '__main__':
    detector = YOLODetector()
    rospy.spin()