#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import tf.transformations as tft
from std_msgs.msg import Float64
from matplotlib.offsetbox import (OffsetImage, AnnotationBbox)

class PosePlotterWithBackground:
    def __init__(self, background_image_path, plane_image_path, shipwreck_image_path):
        rospy.init_node('pose_plotter_with_background', anonymous=True)
        
        # Load background image
        self.background_image = mpimg.imread(background_image_path)
        
        # Load obstacle images
        self.plane_image = mpimg.imread(plane_image_path)
        self.shipwreck_image = mpimg.imread(shipwreck_image_path)
        
        # Initialize data lists
        self.x_data = []
        self.y_data = []
        self.z_data = []

        # Subscriber to pose topic
        rospy.Subscriber('/rexrov2/pose_gt', Odometry, self.pose_callback)
        # Subscriber to obstacle topic
        rospy.Subscriber('/rexrov/obstacle', Float64, self.obstacle_callback)
        
        # Initialize plot
        self.fig, self.ax = plt.subplots()
        self.ax.imshow(self.background_image, extent=[-100, 100, -70, 270], aspect='equal')
        self.line, = self.ax.plot([], [], 'b-', label="Vehicle Trajectory")
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_title('Vehicle Ground Truth Pose')
        

        # Define the points S, A, B, C
        self.line_points_x = [-55,-55,55, 55]
        self.line_points_y = [160,-50, -50,200]
        labels = ['S', 'A', 'B', 'C']

        # Plot the red dotted line
        self.ax.plot(self.line_points_x, self.line_points_y, 'r--', label="Desired Track (S->A->B->C)")

        self.ax.legend()
        self.ax.grid()

        # Annotate points with labels
        for i, label in enumerate(labels):
            self.ax.annotate(label, (self.line_points_x[i], self.line_points_y[i]), textcoords="offset points", xytext=(5,-5), ha='center', color='red')

        # Variables to store obstacle states
        self.plane_present = False
        self.shipwreck_present = False

    def pose_callback(self, pose_msg):
        # Extract robot's pose
        pose = pose_msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        orientation = pose.orientation

        # Convert quaternion to Euler angles
        roll, pitch, yaw = tft.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        if len(self.x_data) == len(self.y_data):
            # Append to data lists
            self.x_data.append(x)
            self.y_data.append(y)
            self.z_data.append(z)

    def obstacle_callback(self, obstacle_msg):
        # Check the obstacle type and update states
        if obstacle_msg.data == 1:
            self.plane_present = True
            self.shipwreck_present = False
        elif obstacle_msg.data == 2:
            self.plane_present = False
            self.shipwreck_present = True
        else:
            self.plane_present = False
            self.shipwreck_present = False

    def run(self):
        # Main loop for updating the plot
        rate = rospy.Rate(10)  # 10 Hz update rate
        plt.ion()  # Turn on interactive mode
        plt.show()

        while not rospy.is_shutdown():
            # Update plot with current data
            if len(self.x_data) == len(self.y_data):
                self.line.set_xdata(self.x_data)
                self.line.set_ydata(self.y_data)

            # Draw plane image if present
            if self.plane_present:
                imagebox = OffsetImage(self.plane_image, zoom = 0.15)
                ab = AnnotationBbox(imagebox, (55, 7), frameon = False)
                self.ax.add_artist(ab)

            # Draw shipwreck image if present
            if self.shipwreck_present:
                imagebox = OffsetImage(self.shipwreck_image, zoom = 0.15)

                ad = AnnotationBbox(imagebox, (55, 174), frameon = False)
                self.ax.add_artist(ad)
            plt.draw()
            plt.pause(0.01)  # Pause briefly to update the plot

            rate.sleep()

if __name__ == '__main__':
    try:
        # Paths to images
        background_image_path =  '/home/user/soil_sand.jpg'   # Update this path
        plane_image_path = '/home/user/plane-removebg-preview.png'  # Update this path
        shipwreck_image_path = '/home/user/ship-removebg-preview.png'  # Update this path
        
        plotter = PosePlotterWithBackground(background_image_path, plane_image_path, shipwreck_image_path)
        plotter.run()
    except rospy.ROSInterruptException:
        pass
