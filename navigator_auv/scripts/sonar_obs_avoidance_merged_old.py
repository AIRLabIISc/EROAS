#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32
from marine_acoustic_msgs.msg import ProjectedSonarImage
from geometry_msgs.msg import Twist

class SonarHeadingNode:
    def __init__(self):
        rospy.init_node('sonar_heading_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/rexrov2/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/rexrov2/blueview_p900/sonar_image_raw', ProjectedSonarImage, self.sonar_image_raw_callback)
        rospy.Subscriber('/rexrov2/dominant_slope', Float32, self.dominant_slope_callback)

        
        self.beam_directions = []
        self.ranges = []
        self.data = []
        self.ping_info = None
        self.criti_cal = False
        self.dominant_slope = None

    def sonar_image_raw_callback(self, data):
        # Extract beam directions, range bin values, and image data from sonar_image_raw
        self.beam_directions = data.beam_directions
        self.ranges = data.ranges
        self.data = data.image.data
        self.ping_info = data.ping_info

    def dominant_slope_callback(self, msg):
        self.dominant_slope = msg.data
        #rospy.loginfo(f"Received Dominant Slope: {self.dominant_slope}")

    def process_data(self):	
        #starti_time = rospy.get_time()
        if self.beam_directions and self.ranges and self.data and self.ping_info:
            obstacle_free_direction = None
            beam_count = 512
            range_count = len(self.ranges)
            sound_speed = self.ping_info.sound_speed
            sample_rate = self.ping_info.frequency
            rospy.loginfo(f"range_count:{range_count}")
            
            obstacle_free_beam_numbers = []
            self.criti_cal = False
            
            for i in range(beam_count-20):  #each beam
                critical_obstacle_detected = False
                for j in range(50,range_count-90):#skip last 20 beams,,,, each row 
                    #range_bin_midpoint = (j + 0.5) * sound_speed / (2.0 * sample_rate)
                    if self.data[beam_count*j+i] > 20:
                        critical_obstacle_detected = True
                        #self.criti_cal = True
                        
                        break
                if not critical_obstacle_detected:
                   obstacle_free_beam_numbers.append(i)
                        
            if not obstacle_free_beam_numbers:
                for i in range(beam_count):
                    obstacle_detected = False
                    for j in range(range_count-20):#skip last 20 beams
                        #range_bin_midpoint = (j + 0.5) * sound_speed / (2.0 * sample_rate)
                        if self.data[beam_count*j+i] > 20:
                            obstacle_detected = True
                    if not obstacle_detected:
                        obstacle_free_beam_numbers.append(i)
                                      
                
            
            if obstacle_free_beam_numbers:
                is_covered, go_beam_no = self.check_for_10_degree_coverage(obstacle_free_beam_numbers)
                if is_covered:
                    self.publish_heading(go_beam_no,True)
                else:
                    self.publish_heading(256,False)
        #endi_time = rospy.get_time()
        #rospy.loginfo(f"Process data duration: {endi_time - starti_time} seconds")

    def check_for_10_degree_coverage(self, obstacle_free_beam_numbers):
        # Constants
        total_beams = 512
        angle_per_beam = 90.0 / total_beams
        
        #if  self.criti_cal:
          #  required_angle = 30.0
        #if  not self.criti_cal:
        required_angle = 25.0
        
          
        
        # Calculate the number of beams needed to cover 10 degrees
        required_beams = int(required_angle / angle_per_beam)
        
        # Sort the beam numbers
        obstacle_free_beam_numbers.sort()
        
        # Check for continuous beams
        mid_beam=[]
        for i in range(len(obstacle_free_beam_numbers) - required_beams + 1):
            if obstacle_free_beam_numbers[i + required_beams - 1] - obstacle_free_beam_numbers[i] + 1 == required_beams:
                mid_index = i + required_beams // 2
                mid_beam.append(obstacle_free_beam_numbers[mid_index])
                
        if mid_beam:
            # Find the beam closest to 256
            closest_to_center = min(mid_beam, key=lambda x: abs(x - 256))
            return True, closest_to_center

        return False, None  

    def publish_heading(self, beam_number,move):
        # Assume the total angle formed by all beams is 90 degrees
        if move:
            total_angle_degrees = 90.0
            # Calculate the angle formed by each beam
            num_beams = 512
            angle_per_beam_degrees = total_angle_degrees / num_beams

            # Calculate the desired heading in degrees based on the beam number with 0 as center beam-> to get -ve velocity
            desired_heading_degrees = (beam_number - (num_beams // 2)) * angle_per_beam_degrees
        
            # Convert the desired heading to radians
            desired_heading_radians = math.radians(desired_heading_degrees)

            #if  self.criti_cal:
            Kp = 0.3 # Proportional gain (tuning parameter)
            Kv=1
            #if  not self.criti_cal:
              #  Kp = 0.2 # Proportional gain (tuning parameter)
                #Kv=0.6
        
            # Calculate the angular velocity
            angular_velocity = Kp * desired_heading_radians
            linear_velocity_x=Kv*(1.6-abs(desired_heading_radians))
            linear_velocity_y=0

            
            # Log the beam number and the desired heading for debugging
            rospy.loginfo(f"Publishing heading for beam number: {beam_number} with desired heading (radians): {desired_heading_radians}, angular velocity: {angular_velocity}, linear velocity: {linear_velocity_x}, is critical:{self.criti_cal:}")

           
        else:
            
            angular_velocity=0
            linear_velocity_x= 0.2
            linear_velocity_y= -linear_velocity_x/self.dominant_slope
            rospy.loginfo(f"entered contour based navigation with desired slope: {self.dominant_slope}, Vx: {linear_velocity_x}, linear velocity: {linear_velocity_y}")

            
        
        
        # Create the Twist message
        twist = Twist()
        twist.angular.z = angular_velocity  # Set the angular velocity
        twist.linear.x = linear_velocity_x
        twist.linear.y = linear_velocity_y
         # Publish the Twist message
        self.cmd_vel_pub.publish(twist)

        

    def run(self):
        rate = rospy.Rate(20)  # 10 Hz
        while not rospy.is_shutdown():
            self.process_data()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SonarHeadingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

