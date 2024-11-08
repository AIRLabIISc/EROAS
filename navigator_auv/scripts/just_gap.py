#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from marine_acoustic_msgs.msg import ProjectedSonarImage
from geometry_msgs.msg import Twist

class SonarHeadingNode:
    def __init__(self):
        rospy.init_node('only_gap', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/rexrov2/cmd_vel_1', Twist, queue_size=10)
        rospy.Subscriber('/rexrov2/blueview_p900/sonar_image_raw', ProjectedSonarImage, self.sonar_image_raw_callback)
        rospy.Subscriber('/rexrov2/global_angle', Float64, self.global_angle_callback)

        self.beam_directions = []
        self.ranges = []
        self.data = []
        self.ping_info = None
        self.criti_cal = False
        self.global_angle = 0
        self.turn_around = False
       

    def global_angle_callback(self,msg):
        #rospy.loginfo(f"Received global angle message: {msg.data}")
        self.global_angle = msg.data
          
        if (-math.pi/2) <self.global_angle< (math.pi/2):
            self.global_angle=math.pi/2 - self.global_angle
            self.turn_around=False
        elif (math.pi/2) <self.global_angle< (math.pi) :
            self.global_angle=0
            self.turn_around=False
        elif -(math.pi) <self.global_angle< -(math.pi/2):
            self.global_angle=3.14
            self.turn_around=False

    def sonar_image_raw_callback(self, data):
        # Extract beam directions, range bin values, and image data from sonar_image_raw
        self.beam_directions = data.beam_directions
        self.ranges = data.ranges
        self.data = data.image.data
        self.ping_info = data.ping_info 

    def process_data(self):
        if not self.turn_around:
            #starti_time = rospy.get_time()
            if self.beam_directions and self.ranges and self.data and self.ping_info:
                obstacle_free_direction = None
                beam_count = 512
                range_count = len(self.ranges)
                sound_speed = self.ping_info.sound_speed
                sample_rate = self.ping_info.frequency
                
                #rospy.loginfo(f"range_count:{range_count}")
                
                obstacle_free_beam_numbers = []
                self.criti_cal = False
                
                for i in range(beam_count-20):  #each beam
                    critical_obstacle_detected = False
                    for j in range(10,range_count-90,2):#skip last 20 beams,,,, each row 
                        #range_bin_midpoint = (j + 0.5) * sound_speed / (2.0 * sample_rate)
                        if self.data[beam_count*j+i] > 20:
                            critical_obstacle_detected = True
                            #self.criti_cal = True
                            
                            break
                    if not critical_obstacle_detected:
                        obstacle_free_beam_numbers.append(i)  ##appends the beam number with no obstacle

                if obstacle_free_beam_numbers:
                    is_covered, go_beam_no = self.check_for_10_degree_coverage(obstacle_free_beam_numbers)
                    if is_covered:
                        self.publish_heading(go_beam_no,True)
                    else:
                        self.publish_heading(256,False)
            #endi_time = rospy.get_time()
            #rospy.loginfo(f"Process data duration: {endi_time - starti_time} seconds")
        if self.turn_around:
            self.publish_heading(256,False)

    def check_for_10_degree_coverage(self, obstacle_free_beam_numbers):
        # Constants
        total_beams = 512
        angle_per_beam = 90.0 / total_beams
        angle_range_start = 0.7853  # Starting angle of the beam range
        angle_range_end = 2.35619 
        rad_per_beam=1.57 / total_beams
        
        required_angle = 5.0
        
        # Calculate the number of beams needed to cover 10 degrees
        required_beams = int(required_angle / angle_per_beam)

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
        
        # Check for continuous beams
        mid_beam=[]
        for i in range(len(obstacle_free_beam_numbers) - required_beams + 1):
            if obstacle_free_beam_numbers[i + required_beams - 1] - obstacle_free_beam_numbers[i] + 1 == required_beams:
                mid_index = i + required_beams // 2
                mid_beam.append(obstacle_free_beam_numbers[mid_index])
        
        target_beam_number = global_angle_to_beam_number(self.global_angle)
        target_beam_number=512-target_beam_number
        rospy.loginfo(f"global_angle:{self.global_angle},target beam:{ target_beam_number}")
        if target_beam_number > 384:
            rospy.loginfo("Turn extreme left")
        elif target_beam_number < 384 and target_beam_number > 256:
            rospy.loginfo("Turn slight left")
        elif target_beam_number < 256 and target_beam_number > 128:
            rospy.loginfo("Turn slight right")
        else:
            rospy.loginfo("Turn extreme right")

        if mid_beam:
            # Find the beam closest to the global angle
            closest_to_target = min(mid_beam, key=lambda x: abs(x - target_beam_number))
            return True, closest_to_target

        return False, None  

    def publish_heading(self, beam_number,move):
        if not self.turn_around:
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

                # Calculate the angular velocity
                angular_velocity = Kp * desired_heading_radians
                linear_velocity_x=Kv*(1.6-abs(desired_heading_radians))
                linear_velocity_y=0

        if self.turn_around:  
            angular_velocity=1.0
            linear_velocity_x= 0.0
            linear_velocity_y= 0.0
         
        # Create the Twist message
        twist = Twist()
        twist.angular.z = angular_velocity  # Set the angular velocity
        twist.linear.x = linear_velocity_x
        twist.linear.y = linear_velocity_y
         # Publish the Twist message
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"Traveling to beam number : {beam_number}")

    def run(self):
        rate = rospy.Rate(5)  # 10 Hz
        while not rospy.is_shutdown():
            self.process_data()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SonarHeadingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

