#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from marine_acoustic_msgs.msg import ProjectedSonarImage
from geometry_msgs.msg import Twist

class SonarHeadingNode:
    def __init__(self):
        rospy.init_node('sonar_heading_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/rexrov2/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/rexrov2/blueview_p900/sonar_image_raw', ProjectedSonarImage, self.sonar_image_raw_callback)
        rospy.Subscriber('/rexrov2/dominant_slope', Float32, self.dominant_slope_callback)
        rospy.Subscriber('/rexrov2/global_angle', Float64, self.global_angle_callback)
        rospy.Subscriber('/rexrov2/avg_right_slope', Float32, self.avg_right_slope_callback)
        rospy.Subscriber('/rexrov2/avg_left_slope', Float32, self.avg_left_slope_callback)

        
        self.beam_directions = []
        self.ranges = []
        self.data = []
        self.ping_info = None
        self.criti_cal = False
        self.dominant_slope = None
        self.global_angle = 0
        self.turn_around=False
        self.right_goal = False
        self.left_goal = False
        self.avg_right_slope = None
        self.avg_left_slope = None

    def avg_right_slope_callback(self, msg):
        self.avg_right_slope = msg.data
        

    def avg_left_slope_callback(self, msg):
        self.avg_left_slope = msg.data
        

    
    def global_angle_callback(self,msg):
        #rospy.loginfo(f"Received global angle message: {msg.data}")
        self.global_angle = msg.data
        #rospy.loginfo(f"global_angle:{self.global_angle}")
        ##use this to check for beam direction
        #check if goal is behind the rover(0,-3.14), if so turn on turn_around flag
        
        
        # if (3*math.pi/4) < abs(self.global_angle):
        #     self.turn_around=True
        #     #self.global_angle=self.global_angle+6.28
        # elif (-math.pi/2) <self.global_angle< (math.pi/2):
        #     self.global_angle=math.pi/2 - self.global_angle
        #     self.turn_around=False
        # elif (math.pi/2) <self.global_angle< (3*math.pi/4) :
        #     self.global_angle=0
        #     self.turn_around=False
        # elif -(3*math.pi/4) <self.global_angle< -(math.pi/2):
        #     self.global_angle=3.14
        #     self.turn_around=False

    
            
        if (-math.pi/2) <self.global_angle< (math.pi/2):
            self.global_angle=math.pi/2 - self.global_angle
            self.turn_around=False
        elif (math.pi/2) <self.global_angle< (math.pi) :
            self.global_angle=0
            self.turn_around=False
        elif -(math.pi) <self.global_angle< -(math.pi/2):
            self.global_angle=3.14
            self.turn_around=False


        #rospy.loginfo(f"global_angle:{self.global_angle}")
        #else send the value



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
                            
                # if not obstacle_free_beam_numbers:  ##will only enter if there are no obstacle free beams
                #     for i in range(beam_count):
                #         obstacle_detected = False
                #         for j in range(range_count-20):#skip last 20 beams
                #             #range_bin_midpoint = (j + 0.5) * sound_speed / (2.0 * sample_rate)
                #             if self.data[beam_count*j+i] > 20:
                #                 obstacle_detected = True
                #         if not obstacle_detected:
                #             obstacle_free_beam_numbers.append(i)
                                        
                    
                
                if obstacle_free_beam_numbers:
                    is_covered, go_beam_no = self.check_for_10_degree_coverage(obstacle_free_beam_numbers)
                    if is_covered:
                        self.publish_heading(go_beam_no,1)
                        rospy.loginfo(f"find the gap")
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
                            self.publish_heading(10,1)
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
                        
                        elif boundedness_number == 4 and self.left_goal:
                            self.publish_heading(400,2)
                            #slope_based_navigator(avg_right_slope)
                            # move rover in angle to positive slope
                            pass
                        elif boundedness_number == 4 and self.right_goal:
                            self.publish_heading(100,2)
                            #slope_based_navigator(avg_left_slope)
                            # move rover in angle to negative slope
                            pass





                        # if boundedness_number == 3:
                        #     # go in direction of goal

            #endi_time = rospy.get_time()
            #rospy.loginfo(f"Process data duration: {endi_time - starti_time} seconds")
        if self.turn_around:
            self.publish_heading(256,False)

    def slope_based_navigator(self,slope):
        #convert the slope to heading angle

        angular_velocity= 0.3* slope


    def check_for_boundedness(self, obstacle_free_beam_numbers):
        self.right_goal = False
        self.left_goal = False
        if not self.turn_around:

            # Check if the angle is between 1.5 and 3.14
            if 1.5 <= self.global_angle <= 3.14:
                self.right_goal = True
            # Check if the angle is between 0 and 1.5
            elif 0 <= self.global_angle < 1.5:
                self.left_goal = True


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
        #rospy.loginfo(f"global_angle:{self.global_angle},target beam:{ target_beam_number}")


        ##here instead of checking for beam closest to 256 check for beam closest to angle towards the global, if angle toward global is from 180 to 0
        # if the angle toward global is from 0 to -180 degree then rotate the rover such that it looks toward the global ie bw 0 to 180 degree and not backside        
        
        
        # if mid_beam:
        #     # Find the beam closest to 256
        #     closest_to_center = min(mid_beam, key=lambda x: abs(x - 256))
        #     return True, closest_to_center
        if mid_beam:
            # Find the beam closest to the global angle
            closest_to_target = min(mid_beam, key=lambda x: abs(x - target_beam_number))
            return True, closest_to_target

        return False, None  

    def publish_heading(self, beam_number,move):
        if not self.turn_around:
            # Assume the total angle formed by all beams is 90 degrees
            if move==1 or move==2:
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
                
                linear_velocity_y=0
                linear_velocity_x=0
                if move==1:
                    linear_velocity_x=Kv*(1.6-abs(desired_heading_radians))
                
                
                # Log the beam number and the desired heading for debugging
                rospy.loginfo(f"heading for beam number: {beam_number} ")

            
            elif move==2:
                
                angular_velocity=0
                linear_velocity_x= 0.2
                #linear_velocity_y= -linear_velocity_x/self.dominant_slope
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

