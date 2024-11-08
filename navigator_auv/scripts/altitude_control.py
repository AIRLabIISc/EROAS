
#!/usr/bin/env python3


import rospy
from uuv_sensor_ros_plugins_msgs.msg import DVL
from geometry_msgs.msg import Twist


class AUVNavigation:
   def __init__(self):
       rospy.init_node('auv_navigation', anonymous=True)


       # Initialize publishers and subscribers
       self.cmd_vel_pub = rospy.Publisher('/rexrov2/cmd_vel', Twist, queue_size=10)
       self.altitude_sub = rospy.Subscriber('/rexrov2/dvl', DVL, self.altitude_callback)


       self.current_velocity = Twist()  # Store current velocity command
       self.altitude = None  # Current altitude
       self.desired_altitude = 5.0  # Desired altitude in meters
       self.kp_altitude = 2.0  # Proportional gain for altitude control
       self.altitude_tolerance = 0.1  # Allowable error in altitude to consider it stable
       self.stabilized = False  # Track if altitude is stable
       self.rate = rospy.Rate(10)  # Set loop rate to 10 Hz


   def altitude_callback(self, msg):
       # Extract altitude from the DVL message
       self.altitude = msg.altitude
       rospy.loginfo(f"Received altitude: {self.altitude} meters")


   def control_altitude(self):
       # Proportional control to maintain desired altitude
       if self.altitude is not None:
           altitude_error = self.desired_altitude - self.altitude
           vertical_velocity = self.kp_altitude * altitude_error


           # Constrain vertical velocity to avoid reverse movement
           if vertical_velocity < 0:
               vertical_velocity = max(vertical_velocity, -0.5)  # Limit descent speed
           else:
               vertical_velocity = min(vertical_velocity, 0.5)  # Limit ascent speed


           # Update the vertical component of the velocity command
           self.current_velocity.linear.z = vertical_velocity


           # Check if altitude is within tolerance
           if abs(altitude_error) < self.altitude_tolerance:
               self.stabilized = True
               rospy.loginfo("Altitude stabilized")
           else:
               self.stabilized = False


           # Log the altitude control details
           rospy.loginfo(f"Altitude error: {altitude_error}, Vertical velocity command: {vertical_velocity}")


   def move_forward(self):
       # Set a positive forward movement velocity
       self.current_velocity.linear.x = 1.0  # Move forward
       self.current_velocity.angular.z = 0.0  # No rotation


   def run(self):
       rospy.loginfo("AUV Navigation Node Running")
       while not rospy.is_shutdown():
           # Control altitude
           self.control_altitude()


           # Ensure forward movement is set
           self.move_forward()


           # Log the current velocity
           rospy.loginfo(f"Current Velocity: Linear {self.current_velocity.linear.x} m/s, Vertical {self.current_velocity.linear.z} m/s")


           # Publish the current velocity command
           self.cmd_vel_pub.publish(self.current_velocity)


           # Sleep to maintain the loop rate
           self.rate.sleep()


if __name__ == '__main__':
   try:
       node = AUVNavigation()
       node.run()
   except rospy.ROSInterruptException:
       pass
