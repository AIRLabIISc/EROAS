#!/usr/bin/env python
import rospy
import math
import tf.transformations as tft
from nav_msgs.msg import Odometry
from marine_acoustic_msgs.msg import ProjectedSonarImage
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
from scipy.signal import convolve2d

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from pcl import PointCloud
from pcl import PointXYZ
import sensor_msgs.point_cloud2 as pc2

class SonarHeadingNode:
    def __init__(self):
        rospy.init_node('sonar_heading_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/rexrov2/cmd_vel_1', Twist, queue_size=10)
        self.publisher = rospy.Publisher('/rexrov2/detected_objects', Image, queue_size=10)
        rospy.Subscriber('/rexrov2/blueview_p900/sonar_image_raw', ProjectedSonarImage, self.sonar_image_raw_callback)
        # Publisher for sonar joint angles
        self.joint_pub = rospy.Publisher('/sonar/joint', Float64, queue_size=10)
        
        # Subscriber for point cloud data
        #self.point_cloud_sub = rospy.Subscriber('/point/cloud', PointCloud2, self.point_cloud_callback)
        
        rospy.Subscriber('/rexrov2/pose_gt', Odometry, self.pose_callback)

        
        self.beam_directions = []
        self.ranges = []
        self.data = []
        self.ping_info = None 
        self.criti_cal = False
        self.dominant_slope = None
        self.right_avg_slope=None
        self.left_avg_slope=None
        self.global_angle = 0
        self.turn_around=False
        self.right_goal = False
        self.left_goal = False
        self.avg_right_slope = None
        self.avg_left_slope = None
        self.target_x = 43
        self.target_y = 63
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.data_available = False
        self.collect_3d_data_flag = False
        self.latest_scan_available=False
        self.start_3d_time=None


    def pose_callback(self,pose_msg):
        # Extract robot's pose
        pose = pose_msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        orientation = pose.orientation
        # Convert quaternion to Euler angles
        roll, pitch, yaw = tft.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        # Compute the angle to the target point
        angle_to_target = math.atan2(self.target_y - y, self.target_x - x)

        # Compute the angular error
        angular_error = angle_to_target - yaw

        # Normalize the angular error to be within the range [-pi, pi]
        self.global_angle = math.atan2(math.sin(angular_error), math.cos(angular_error))

        if (-math.pi/2) <self.global_angle< (math.pi/2):
            self.global_angle=math.pi/2 + self.global_angle
            self.turn_around=False
        elif (math.pi/2) <self.global_angle< (math.pi) :
            self.global_angle=3.14
            self.turn_around=False
        elif -(math.pi) <self.global_angle< -(math.pi/2):
            self.global_angle=0
            self.turn_around=False

        #rospy.loginfo(f"global_angle {self.global_angle:.4f}")

        # start_time = rospy.get_time()
        # rospy.loginfo(f"global angle {self.global_angle:.4f}")
        # self.process_data()
        # end_time = rospy.get_time()
        # rospy.loginfo(f" ")
        # rospy.loginfo(f"process time {end_time -start_time:.4f}")
        # rospy.loginfo(f" ")



        


    def sonar_image_raw_callback(self, data):
        # Extract beam directions, range bin values, and image data from sonar_image_raw
        self.beam_directions = data.beam_directions
        self.ranges = data.ranges
        self.ping_info = data.ping_info
        # Convert the raw data into a numpy array
        if self.beam_directions and self.ranges and data.image.data and self.ping_info:
            #start_time = rospy.get_time()
            self.data_raw = np.frombuffer(data.image.data, dtype=np.uint8)
            self.data_available = True
            


    def polar_to_cartesian(self, i, j, max_beams, max_bins):
        """
        Convert polar coordinates (i, j) to Cartesian coordinates.
        The image appears rotated by 90 degrees, so adjust accordingly.
        """
        #here max distance is assigned 15
        # rospy.loginfo(f"cartesian")
        angle_per_beam = 0.0030739647336304188
        angle_rad = i * angle_per_beam + math.pi/4
        distance = j * 15 / max_bins
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return x, y
    
    def print_polynomial_coefficients(self, coeffs):
        print("Polynomial Coefficients:")
        print(f"y = {coeffs[0]:.4f}x^2 + {coeffs[1]:.4f}x + {coeffs[2]:.4f}")

    def compute_polynomial_derivative(self, coeffs):
        a, b, c = coeffs
        return [2*a, b]

    def find_and_plot_curve(self, x_coords, y_coords):
        # Ensure x_coords are strictly increasing
        sorted_points = sorted(zip(x_coords, y_coords))
        x_coords, y_coords = zip(*sorted_points)
        rospy.loginfo(f"plotter")
        # Check if x_coords is strictly increasing
        if any(x2 <= x1 for x1, x2 in zip(x_coords, x_coords[1:])):
            rospy.logwarn("x_coords are not strictly increasing. Removing duplicates.")
            # Remove duplicates
            unique_points = []
            for x, y in zip(x_coords, y_coords):
                if not unique_points or unique_points[-1][0] != x:
                    unique_points.append((x, y))
            x_coords, y_coords = zip(*unique_points)

        # Perform polynomial interpolation (2nd degree)
        coeffs = np.polyfit(x_coords, y_coords, 2)
        self.print_polynomial_coefficients(coeffs)

        # Compute derivative coefficients
        derivative_coeffs = self.compute_polynomial_derivative(coeffs)

        # Generate points for the fitted curve
        x_fit = np.linspace(min(x_coords), max(x_coords), num=500)
        y_fit = np.polyval(coeffs, x_fit)

        # # Create a blank image for visualization
        # img_width = 1000
        # img_height = 1000
        # display_image = np.zeros((img_height, img_width, 3), dtype=np.uint8)

        # # Plot the fitted curve
        # for i in range(len(x_fit) - 1):
        #     # Transform to image coordinates
        #     x1 = int(500 + x_fit[i] * 500 / 15)
        #     y1 = int(1000 - y_fit[i] * 1000 / 15)
        #     x2 = int(500 + x_fit[i + 1] * 500 / 15)
        #     y2 = int(1000 - y_fit[i + 1] * 1000 / 15)
        #     cv2.line(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green color line

        # Calculate slopes at unique contour points
        self.slopes = []
        positive_tangent_slopes = []
        negative_tangent_slopes = []

        for x, y in zip(x_coords, y_coords):
            tangent_slope = np.polyval(derivative_coeffs, x)
            if x > 0:
                positive_tangent_slopes.append(tangent_slope)
            elif x < 0:
                negative_tangent_slopes.append(tangent_slope)

        # If you also want to store the slopes in self.slopes, ensure it's defined as well
        self.slopes = positive_tangent_slopes + negative_tangent_slopes
                # Compute average slopes
        avg_positive_slope = np.mean(positive_tangent_slopes) if positive_tangent_slopes else 0
        avg_negative_slope = np.mean(negative_tangent_slopes) if negative_tangent_slopes else 0
        self.avg_right_slope=avg_positive_slope
        self.avg_left_slope=avg_negative_slope
        # Compare magnitudes
        magnitude_positive = abs(avg_positive_slope)
        magnitude_negative = abs(avg_negative_slope)

        if magnitude_positive > magnitude_negative:
            dominant_slope = avg_positive_slope
        else:
            dominant_slope = avg_negative_slope

        # Log the dominant slope
        #rospy.loginfo(f"Dominant Slope (higher magnitude): {dominant_slope:.4f}")

        # # Convert the image to ROS format and publish
        # ros_image = self.bridge.cv2_to_imgmsg(display_image, encoding='bgr8')
        
        # self.publisher.publish(ros_image)

        
        
        
    def process_data(self):	
        if not self.turn_around:
            #starti_time = rospy.get_time()
            
            obstacle_free_direction = None
            beam_count = 512    
            range_count = len(self.ranges)
            # sound_speed = self.ping_info.sound_speed
            # sample_rate = self.ping_info.frequency
            threshold = 15

            self.data = np.array(self.data_raw).reshape((range_count, beam_count))
            
            #rospy.loginfo(f"range_count:{range_count}")
            
            obstacle_free_beam_numbers = []
            self.criti_cal = False
            range_window = 3


            contour_points = []
            # Iterate over the range of beam indices
            for i in range(0,beam_count, 5):
                critical_obstacle_detected = False
                # Iterate over the range of range bin indices
                for j in range(10, range_count - 40, 5):
                    # Extract the window of intensities
                    window_intensities = self.data[j:j + range_window, i]
                    
                    # Compute the average intensity
                    window_average_intensity = np.mean(window_intensities)
                    
                    # Check against the threshold
                    if window_average_intensity > threshold:
                        contour_points.append((i, j))
                        critical_obstacle_detected = True
                        break  # Exit the loop if the condition is met
                
                if not critical_obstacle_detected:
                    obstacle_free_beam_numbers.append(i)

            # kernel = np.ones((3, 3)) / 9

            # threshold = 15

            # convolved_data = convolve2d(self.data, kernel, mode='same',  boundary='fill', fillvalue=0)
              
            # mask = convolved_data > threshold
            # min_row_indices = np.full(convolved_data.shape[1], convolved_data.shape[0])

            # # Compute minimum row indices per beam
            # for col in range(convolved_data.shape[1]):
            #     min_row_indices[col] = np.min(np.where(mask[:, col], np.arange(convolved_data.shape[0]), convolved_data.shape[0]))

            # # Filter contour points
            # contour_points = [(beam_number, int(min_bin)) for beam_number, min_bin in enumerate(min_row_indices) if min_bin < convolved_data.shape[0]]

            # # Determine obstacle-free beams
            # all_beam_numbers = set(range(convolved_data.shape[1]))
            # obstacle_beams = set(min_row_indices[min_row_indices < convolved_data.shape[0]])
            # obstacle_free_beam_numbers = list(all_beam_numbers - obstacle_beams)

            
                
            rospy.loginfo(f" len of of obstacle freee beams: {obstacle_free_beam_numbers} ")

            is_covered=False
            
            if obstacle_free_beam_numbers:
                is_covered, go_beam_no = self.check_for_10_degree_coverage(obstacle_free_beam_numbers)
                
            if is_covered:
                rospy.loginfo(f"find the gap")
                self.publish_heading(go_beam_no,1)

            else:
                rospy.loginfo(f"no gap")
                #self.publish_heading(256,False)
                boundedness_number= self.check_for_boundedness(obstacle_free_beam_numbers)
                
                if boundedness_number == 1 and self.right_goal:
                    #slope_based_navigator(avg_right_slope)
                    
                    self.publish_heading(10,1)
                    #turn in direction of goal
                    pass
                    # move rover in angle to positive slope
                elif boundedness_number == 1 and self.left_goal:
                    self.publish_heading(500,1)
                    #slope_based_navigator(avg_left_slope)
                    #turn in direction of goal
                    pass
                    # move rover in angle to negative slope
                elif boundedness_number == 2 and self.left_goal:
                    self.publish_heading(10,1)
                    #slope_based_navigator(avg_right_slope)
                    # move rover in angle to positive slope
                    pass
                elif boundedness_number == 2 and self.right_goal:
                    self.publish_heading(500,1)
                    #slope_based_navigator(avg_left_slope)
                    # move rover in angle to negative slope
                    pass

                    

                        
                else:
                    rospy.loginfo(f"contour_1")               

                    points_cartesian = [self.polar_to_cartesian(i, j, beam_count, range_count) for (i, j) in contour_points] 

                    # def check if contour 10 meters away then enter 3d navigation mode

                    if points_cartesian:# Extract x and y coordinates
                        x_coords, y_coords = zip(*points_cartesian)

                        # Find and plot the curve
                        self.find_and_plot_curve(x_coords, y_coords)
                
                    if self.left_goal:
                        #check contour slope
                        self.publish_heading(self.avg_left_slope,2)
                        #slope_based_navigator(avg_right_slope)
                        # move rover in angle to positive slope
                    
                    elif self.right_goal:
                        #check contour slope
                        self.publish_heading(self.avg_right_slope,2)
                        #slope_based_navigator(avg_left_slope)
                        # move rover in angle to negative slope
                            





            #endi_time = rospy.get_time()
            #rospy.loginfo(f"Process data duration: {endi_time - starti_time} seconds")
        if self.turn_around:
            self.publish_heading(256,False)



    def check_for_boundedness(self, obstacle_free_beam_numbers):
        self.right_goal = False
        self.left_goal = False
        if not self.turn_around:

            # Check if the angle is between 1.5 and 3.14
            if 1.5 <= self.global_angle <= 3.14:
                self.right_goal = True
                self.left_goal = False
            # Check if the angle is between 0 and 1.5
            elif 0 <= self.global_angle < 1.5:
                self.left_goal = True
                self.right_goal=False


                # Check if beam 0 is present
            has_beam_0 = 0 in obstacle_free_beam_numbers
            # Check if beam 512 is present
            has_beam_512 = 512 in obstacle_free_beam_numbers
            
            # Determine the return value based on the presence of the beams
            if has_beam_0 and self.right_goal:
                return 1
            elif has_beam_512 and self.left_goal:
                return 1
            elif has_beam_0 and self.left_goal:
                return 2
            elif has_beam_512 and self.right_goal:
                return 2
            
            else :
                rospy.loginfo(f"detected object not bounded")
                return 4
                
    def move_sonar(self):
        if self.collect_3d_data_flag:
            # Collect data if within the current cycle
            current_time = rospy.get_time() - self.start_3d_time
            cycle_duration = 2 * math.pi / self.frequency

            # Compute the sine wave position
            z_position = self.amplitude * math.sin(2 * math.pi * self.frequency * current_time) + self.offset
            degree_angle = z_position * (180 / math.pi)
    
            # Round to the closest multiple of 3
            self.sonar_angle = round(degree_angle / 3) * 3
            
            # Publish the current joint angle
            self.joint_pub.publish(Float64(z_position))
            rospy.loginfo(f"Moving sonar to z = {z_position}")

            if current_time < cycle_duration:
                # Convert PointCloud2 data to a list of (x, y, z) tuples
                # pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
                # self.obstacle_array.extend(pc_data)  # Add points to the obstacle array
                # rospy.loginfo(f"Added {len(pc_data)} points to obstacle array.")
            else:
                # Reset obstacle array and start time after a full cycle
                rospy.loginfo(f"Obstacle array contains {len(self.obstacle_array)} points before clearing.")
                self.latest_full_scan = self.obstacle_array
                self.latest_scan_available=True
                self.obstacle_array = []  # Clear collected data
                rospy.loginfo("Cleared obstacle array after completing a full cycle.")
                self.stop_data_collection()
                #self.start_3d_time = rospy.get_time()  # Reset start time for the next cycle
    
    def start_data_collection(self):
        self.collect_3d_data_flag = True

    def stop_data_collection(self):
        self.collect_3d_data_flag = False
        rospy.loginfo(f"Stopping data collection. Obstacle array contains {len(self.obstacle_array)} points.")
        # Optionally reset the obstacle array or process it here
        # For example: self.obstacle_array = []



    
    def navigate_3d(self):
        

        #move sonr up and down

        self.start_3d_time = rospy.get_time()
        self.start_data_collection()
        self.move_sonar()
        self.process_3d_data()
        mid_beam=[]
        for i in range(len(obstacle_free_beam_numbers) - required_beams ):
            if obstacle_free_beam_numbers[i + required_beams ] - obstacle_free_beam_numbers[i]  == 5 *required_beams :
                mid_index = i + required_beams // 2
                mid_beam.append(obstacle_free_beam_numbers[mid_index])


        if mid_beam:
            # Find the beam closest to the global angle
            closest_to_target = min(mid_beam, key=lambda x: abs(x - target_beam_number))



        # if self.latest_scan_available: # stop code flow until its available

            

        #     rover_position = np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
        
        #     # # Transform the point cloud data to rover's local coordinates
        #     # # Here we assume the pose gives us the relative z-coordinate, so we just subtract it.
        #     # relative_pc = self.latest_full_scan - rover_position
            
        #     # # Extract z-coordinates
        #     # z_coords = relative_pc[:, 2]

        #     #  # Determine the extent of the obstacle in the vertical direction
        #     # z_min = np.min(z_coords)
        #     # z_max = np.max(z_coords)

        #     # rospy.loginfo(f"Obstacle extent in the vertical direction: min z = {z_min}, max z = {z_max}")

        #     # if abs(z_min) > abs(z_max):
        #     #     rospy.loginfo("Obstacle extends more downward relative to the rover.")
        #     #     #go up
        #     #     #decide motion 
        #     # else:
        #     #     rospy.loginfo("Obstacle extends more upward relative to the rover.")
        #     #     #go down
        #     #     #decide motion


        #     self.latest_scan_available=False

        #check z value in extremea

        #choose least direction vlue is less than a threshold z , based on that move sonar

        #move sonar downward looking, see till where its empty(remmber that point) i can go till there

        #go in that z value

        #
        

    def process_3d_data(self):

        beam_count = 512    
        range_count = len(self.ranges)
        # sound_speed = self.ping_info.sound_speed
        # sample_rate = self.ping_info.frequency
        threshold = 15

        self.data = np.array(self.data_raw).reshape((range_count, beam_count))

        # Define the indices for slicing
        beam_start = 150
        beam_end = 360
        range_start = 30
        range_end = 500

        # Define the interval for skipping data points
        range_interval = 3
        beam_interval = 10

        # Slice the array with steps
        data_slice = self.data[range_start:range_end:range_interval, beam_start:beam_end:beam_interval]

        # Calculate the average value
        average_value = np.mean(data_slice)

        if average_value<2:

            self.go_vertical_angle.append(int(self.sonar_angle))  
            
                
    

    def check_for_10_degree_coverage(self, obstacle_free_beam_numbers):
        # Constants
        total_beams = 512
        angle_per_beam = 90.0 / total_beams
        angle_range_start = 0.7853  # Starting angle of the beam range
        angle_range_end = 2.35619 
        rad_per_beam=1.57 / total_beams
        
        #if  self.criti_cal:
          #  required_angle = 30.0
        #if  not self.criti_cal:
        required_angle = 25.0
        
          
        
        # Calculate the number of beams needed to cover 10 degrees
        required_beams = int(required_angle / (angle_per_beam*5))
        
        # Calculate the global angle in degrees to beam number
        def global_angle_to_beam_number(global_angle):
            # Convert angle to beam number
            
            if angle_range_start <= global_angle <= angle_range_end:  ## convert the angle range to radians or will give error
                return int((global_angle - angle_range_start) / rad_per_beam)
            elif global_angle>2.35619:
                return 512  # Global angle out of the beam range, add logic for side beams, ie (0,45) and (135,180)
            elif global_angle<0.7853:
                return 0
    
        
        # Sort the beam numbers
        obstacle_free_beam_numbers.sort()
        # rospy.loginfo(f" beam:{obstacle_free_beam_numbers}")
        
        # Check for continuous beams
        mid_beam=[]
        for i in range(len(obstacle_free_beam_numbers) - required_beams ):
            if obstacle_free_beam_numbers[i + required_beams ] - obstacle_free_beam_numbers[i]  == 5 *required_beams :
                mid_index = i + required_beams // 2
                mid_beam.append(obstacle_free_beam_numbers[mid_index])
                
        
        target_beam_number = global_angle_to_beam_number(self.global_angle)
        #target_beam_number=512-target_beam_number
        rospy.loginfo(f"target beam:{ target_beam_number}")
        


        ##here instead of checking for beam closest to 256 check for beam closest to angle towards the global, if angle toward global is from 180 to 0
        # if the angle toward global is from 0 to -180 degree then rotate the rover such that it looks toward the global ie bw 0 to 180 degree and not backside        
        
        
        # if mid_beam:
        #     # Find the beam closest to 256
        #     closest_to_center = min(mid_beam, key=lambda x: abs(x - 256))
        #     return True, closest_to_center
        #rospy.loginfo(f"len of mid beam:{len(mid_beam)}")
        if mid_beam:
            # Find the beam closest to the global angle
            closest_to_target = min(mid_beam, key=lambda x: abs(x - target_beam_number))
            
            
            return True, closest_to_target

        return False, None  

    def publish_heading(self, beam_number,move):
        if not self.turn_around:
            # Assume the total angle formed by all beams is 90 degrees
            if move==1 :
                total_angle_degrees = 90.0
                # Calculate the angle formed by each beam
                num_beams = 512
                angle_per_beam_degrees = total_angle_degrees / num_beams

                # Calculate the desired heading in degrees based on the beam number with 0 as center beam-> to get -ve velocity
                desired_heading_degrees = (beam_number - (num_beams // 2)) * angle_per_beam_degrees
            
                # Convert the desired heading to radians
                desired_heading_radians = math.radians(desired_heading_degrees)

                #if  self.criti_cal:
                Kp = 0.12 # Proportional gain (tuning parameter)
                Kv=0.35
                #if  not self.criti_cal:
                #  Kp = 0.2 # Proportional gain (tuning parameter)
                    #Kv=0.6
            
                # Calculate the angular velocity
                angular_velocity = Kp * desired_heading_radians
                
                linear_velocity_y=0
                linear_velocity_x=0
                if move==1:
                    linear_velocity_x=Kv*(1.6-abs(desired_heading_radians))
                
                #rospy.loginfo(f"linear_velocity_x: {linear_velocity_x} ")
                
                # Log the beam number and the desired heading for debugging
                rospy.loginfo(f"heading for beam number: {beam_number} ")
                rospy.loginfo(f"global angle: {self.global_angle} ")

            
            elif move==2:
                desired_heading_radians=beam_number##here beam number is desired hading value in slope directly from the call
                angular_velocity=0.1*desired_heading_radians 
                linear_velocity_x= 0.0
                linear_velocity_y= 0.0
                # linear_velocity_y= -linear_velocity_x/self.dominant_slope
                rospy.loginfo(f"entered contour based navigation with desired slope: {self.dominant_slope}, Vx: {linear_velocity_x}, linear velocity: {linear_velocity_y}")

        if self.turn_around:  
            
            angular_velocity=1.0
            linear_velocity_x= 0.0
            linear_velocity_y= 0.0
        
        
        # # Create the Twist message
        twist = Twist()
        twist.angular.z = angular_velocity  # Set the angular velocity
        twist.linear.x = linear_velocity_x
        twist.linear.y = linear_velocity_y
        #Publish the Twist message
        self.cmd_vel_pub.publish(twist)

        

    def run(self):
        #rospy.spin()
        while not rospy.is_shutdown():
            if self.data_available:
                start_time = rospy.get_time()
                self.process_data()
                end_time = rospy.get_time()
                rospy.loginfo(f" ")
                #rospy.loginfo(f"process time {end_time -start_time:.4f}")
                #self.data_available = False  # Reset the flag after processing
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = SonarHeadingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

