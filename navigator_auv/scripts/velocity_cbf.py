#!/usr/bin/env python3

import rospy
import numpy as np
import cvxpy as cp
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Twist
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
import tf
from std_msgs.msg import Float64
from sensor_msgs.point_cloud2 import create_cloud_xyz32
#from scipy.optimize import minimize

class ObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node', anonymous=True)
        
        # Subscribers
        self.vel_sub = rospy.Subscriber('/rexrov2/cmd_vel_1', Twist, self.vel_callback)
        self.pc_sub = rospy.Subscriber('/rexrov2/point_cloud', PointCloud2, self.point_cloud_callback)
        self.pose_sub = rospy.Subscriber('/rexrov2/pose_gt', Odometry, self.pose_callback)
        self.sonar_sub = rospy.Subscriber('/rexrov2/sonar/moving', Float64, self.sonar_callback)
        self.h_pub = rospy.Publisher('/rexrov2/current_h', Float64, queue_size=10)
        self.closest_publisher = rospy.Publisher('/rexrov2/closest_point', PointCloud2, queue_size=10)
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/rexrov2/cmd_vel', Twist, queue_size=10)
        
        # CBF parameters
        self.safe_distance = 0  # Minimum safe distance from obstacles
        self.R_o=2
        self.radius=15
        self.kappa =0.09
        self.kappa1 =0.09

        self.filtered_points = np.empty((0, 3))
        # Variables
        self.current_pose = None
        self.current_vel = None
        self.point_cloud = None
        self.vehicle_pose =None
        self.current_h= float('inf')
        self.sonar_moving=False
        self.xy_cbf=False
        self.xz_cbf=False
        self.closest_points=[]
        self.quaternion=None

        
        rospy.spin()

    def sonar_callback(self, msg):
        # print("sonar callback")
        sonar_state = msg.data
        if sonar_state ==1 :
            self.sonar_moving=False
            self.xy_cbf=False
            self.xz_cbf=True
            self.R_o=2
            # print("xz")
        if sonar_state == 2:
            self.sonar_moving=True
            self.xy_cbf=False
            self.xz_cbf=False
            self.R_o=4
            # print("xy")
        if sonar_state == 0:
            self.sonar_moving=False
            self.xy_cbf=True
            self.xz_cbf=False
            self.R_o=4
            # print("xy")

    
    def vel_callback(self, msg):
        self.v_alg = msg
        self.process_data(self.v_alg)


    def point_cloud_callback(self, msg):
        #rospy.loginfo("pc Data received..")

        self.last_h = self.current_h
        
        if self.vehicle_pose is None:
            rospy.loginfo("Vehicle pose not yet received")
            

        # Convert point cloud to numpy array
        # rospy.loginfo(" ")
        # rospy.loginfo(" ")
        # rospy.loginfo("000000000000000")
        # rospy.loginfo("point call back")
        # rospy.loginfo("0000000000000000")
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        new_points_float = np.array(list(pc_data))
        new_points = np.round(new_points_float).astype(int)

        # Extract vehicle position
        vx, vy, vz = self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z
        
        
        # Concatenate existing filtered points with new points
        if len(self.filtered_points) > 0 and len(new_points) > 0:
            filtered_points_np = np.array(self.filtered_points)
            all_points = np.vstack((filtered_points_np, new_points))
            
        elif len(new_points) > 0 :
            all_points = np.array(new_points)

        elif len(self.filtered_points) > 0 :
            all_points = np.array(self.filtered_points)

        else:
            all_points = np.empty((0, 3))


        # Compute distances for all points
        if len(all_points) > 0:
            all_points = np.unique(all_points, axis=0)
            # rospy.loginfo(f"len of all_points: {len(self.filtered_points)}")
            # rospy.loginfo(f"all_points: {self.filtered_points}")
            distances = np.linalg.norm(all_points - np.array([vx, vy, vz]), axis=1)
            smallest_distance = np.min(distances) if distances.size > 0 else float('inf')
            rospy.loginfo(f"Smallest distance: {smallest_distance} ")
            if smallest_distance > self.radius:
                smallest_distance = float('inf')

            # Filter points within the radius
            within_radius_mask = distances <= self.radius
            self.filtered_points = all_points[within_radius_mask].tolist()

        else :
            self.filtered_points = np.empty((0, 3))
            #rospy.loginfo(f"Total stored points: {len(self.filtered_points)}")
            # rospy.loginfo(f"Smallest distance: {smallest_distance}")
        #rospy.loginfo(f"filtered points: {self.filtered_points}")   

    
    
    def pose_callback(self, msg):
        
        if self.xy_cbf or self.xz_cbf or self.sonar_moving:
            # rospy.loginfo(f"current u : {self.current_u}")
            # Store vehicle pose (assuming position in global frame)
        
            self.vehicle_pose = msg.pose.pose.position
            # self.current_u = msg.twist.twist.linear.x
            # self.current_v = msg.twist.twist.linear.y
            # self.current_w = msg.twist.twist.linear.z
            # self.current_r = msg.twist.twist.angular.z
            orientation_q = msg.pose.pose.orientation
            quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            
            self.quaternion= quaternion

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.yaw = euler[2]  # Only yaw is needed for 2D velocity transformations



            vxx, vyy, vzz = self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z
            distance = np.linalg.norm(self.filtered_points - np.array([vxx, vyy, vzz]), axis=1)
            #smallest_distance = np.min(distance) if distance.size > 0 else float('inf')
            
            if distance.size > 0:
                # Find the smallest distance
                smallest_distance = np.min(distance)
                
                # Find all indices where the distance equals the smallest distance
                min_indices = np.where(distance == smallest_distance)[0]
                
                # Choose the first point (or handle multiple points as needed)
                self.closest_point = self.filtered_points[min_indices[0]]
                
            else:
                smallest_distance = float('inf')
                self.closest_point = None

            # If the smallest distance is greater than the radius, set it to infinity
            if smallest_distance > self.radius:
                smallest_distance = float('inf')
                self.closest_point = None

            self.closest_obstacle_distance = smallest_distance
            #rospy.loginfo(f"closest point X:{self.closest_point[0]}, Z : {self.closest_point[2]}")
            
            # self.closest_points=self.closest_point

            # if len(self.closest_points) > 0:
            #     history_header = msg.header
            #     #history_header.frame_id = "blueview_p900_visual_ray_link"
            #     history_header.frame_id = "world"
            #     history_point_cloud_msg = create_cloud_xyz32(history_header, self.closest_points)
            #     self.closest_publisher.publish(history_point_cloud_msg)

            # Compute current_h, if closest_point is not None
            self.current_h = (self.closest_obstacle_distance**2) - (self.R_o**2 )




    def compute_h_dot_xy(self,Vel_1,Vel_2):
        # Compute the time derivative of h(x) based on control inputs T = [Tx, Ty, Tz, Tpsi]
        

        # Time derivative of the CBF
        #self.h_dot_xy_t = np.dot(h_state_dot, x_dot)
        #h_state_dot=np.array([(self.closest_point[0]-self.vehicle_pose.x)/self.closest_obstacle_distance ,(self.closest_point[1]-self.vehicle_pose.y)/self.closest_obstacle_distance ])
        h_state_dot=np.array([2*(self.vehicle_pose.x-self.closest_point[0]),2*(self.vehicle_pose.y - self.closest_point[1])])
       
        self.h_dot_xy_t = np.dot(h_state_dot, np.transpose(np.array([Vel_1,Vel_2])))
        return self.h_dot_xy_t
    

    

    def quaternion_to_rotation_matrix(self,quaternion):
        """
        Convert a quaternion into a rotation matrix.
        
        Parameters:
        quaternion (tuple): A tuple representing the quaternion (x, y, z, w).
        
        Returns:
        np.ndarray: A 3x3 rotation matrix.
        """
        x, y, z, w = quaternion
        # Calculate rotation matrix elements
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
        ])



    def global_to_local_3d(self,global_coord):
        """
        Convert global 3D coordinates to local coordinates relative to the vehicle's pose.
        
        Parameters:
        global_coord (tuple): The global coordinates (x, y, z) to convert.
        vehicle_pose (dict): A dictionary containing 'position' and 'orientation' of the vehicle.
                            'position' is a tuple (x, y, z) and 'orientation' is a quaternion (x, y, z, w).

        Returns:
        local_coord (tuple): The local coordinates (x_local, y_local, z_local).
        """
        
        # Extract vehicle position and orientation from vehicle_pose
        vehicle_position = self.vehicle_pose # (x, y, z)
        orientation_q = self.quaternion             # (x, y, z, w)

        # Convert quaternion to rotation matrix
        rotation_matrix = self.quaternion_to_rotation_matrix(orientation_q)

        # Convert global coordinates to local coordinates
        global_x, global_y, global_z = global_coord
        vehicle_x, vehicle_y, vehicle_z = vehicle_position.x,vehicle_position.y,vehicle_position.z

        # Shift global coordinates to vehicle's position
        shifted_global = np.array([global_x - vehicle_x, global_y - vehicle_y, global_z - vehicle_z])

        # Rotate the shifted coordinates using the inverse of the rotation matrix
        local_coord = rotation_matrix.T @ shifted_global

        return local_coord




    
    def compute_h_dot_xz(self,Vel_1,Vel_2):
        # Compute the time derivative of h(x) based on control inputs T = [Tx, Ty, Tz, Tpsi]
        

        # Time derivative of the CBF
        #self.h_dot_xz_t = np.dot(h_state_dot, x_dot)
        #h_state_dot=np.array([(self.closest_point[0]-self.vehicle_pose.x)/self.closest_obstacle_distance ,(self.closest_point[1]-self.vehicle_pose.y)/self.closest_obstacle_distance ])
        local_cordi= self.global_to_local_3d(self.closest_point)
        
        h_state_dot=np.array([2*(-local_cordi[0]),2*(-local_cordi[2])])
       
        self.h_dot_xz_t = np.dot(h_state_dot, np.transpose(np.array([Vel_1,Vel_2])))
        return self.h_dot_xz_t



    def transform_velocity_to_global_xy(self, local_velocity_x, local_velocity_y):
        # Perform the transformation using yaw (from pose_gt)
        global_velocity_x = local_velocity_x * np.cos(self.yaw) - local_velocity_y * np.sin(self.yaw)
        global_velocity_y = local_velocity_x * np.sin(self.yaw) + local_velocity_y * np.cos(self.yaw)
        return global_velocity_x, global_velocity_y
    
    def transform_velocity_to_global_xz(self, local_velocity_x, local_velocity_z):
        # Perform the transformation using yaw (from pose_gt)
        global_velocity_x = local_velocity_x * np.cos(self.yaw) 
        
        return global_velocity_x, local_velocity_z
    

    def transform_velocity_to_local_xy(self, global_velocity_x, global_velocity_y):
        # Perform the inverse transformation using yaw (from pose_gt)
        local_velocity_x = global_velocity_x * np.cos(self.yaw) + global_velocity_y * np.sin(self.yaw)
        local_velocity_y = -global_velocity_x * np.sin(self.yaw) + global_velocity_y * np.cos(self.yaw)
        return local_velocity_x, local_velocity_y
    
    def transform_velocity_to_local_xz(self, global_velocity_x, global_velocity_z):
        # Perform the inverse transformation using yaw (from pose_gt)
        local_velocity_x = global_velocity_x * np.cos(self.yaw) 
        
        return local_velocity_x, global_velocity_z
        
   

    def cbf_optimization_xy(self,velll):
        # Convert point cloud to numpy array
        
        # Define optimization variables
        linear_velocity_x = cp.Variable()
        linear_velocity_y = cp.Variable()

    

        desired_velocity_x, desired_velocity_y = self.transform_velocity_to_global_xy(velll.linear.x, velll.linear.y)
    

        # Define the objective: minimize deviation from desired velocities
        objective = cp.Minimize(cp.square(linear_velocity_x - desired_velocity_x) + cp.square(linear_velocity_y - desired_velocity_y))

        # Define the constraint: ensure obstacle avoidance
       
        constraint = self.compute_h_dot_xy(linear_velocity_x ,linear_velocity_y) + self.kappa * (self.current_h - 0.5)
        
        constraints = [constraint >= 0]

        # Formulate the problem
        problem = cp.Problem(objective, constraints)

        try:
            # Solve the problem
            problem.solve()

            # Check if the problem was solved successfully
            if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                # Get the optimized velocities
              
                optimized_velocity_x,optimized_velocity_y = self.transform_velocity_to_local_xy(linear_velocity_x.value, linear_velocity_y.value)
    

                safe_velocity = np.array([optimized_velocity_x, optimized_velocity_y])
                h_dddot=self.compute_h_dot_xy(optimized_velocity_x ,optimized_velocity_y) 
                optimal_constraint = h_dddot + self.kappa * (self.current_h - 0.5)
                # rospy.loginfo(f"optimal_constraint: {optimal_constraint}")
                # rospy.loginfo(f"compute_h_dot_xy: {h_dddot}")
                # rospy.loginfo(f"current h: {self.current_h}")
                #rospy.loginfo(f"closest distance: {self.closest_obstacle_distance}")
                # print("Optimization successful")
            else:
                print("Optimization failed: No optimal solution found")
                # Set both velocities to 0 if optimization fails
                safe_velocity = np.array([0.0, 0.0])

        except Exception as e:
            print(f"An error occurred during optimization: {e}")
            # Set both velocities to 0 if an error occurs
            safe_velocity = np.array([0.0, 0.0])

        return safe_velocity
    





    def cbf_optimization_xz(self,velll):
        # Convert point cloud to numpy array
        rospy.loginfo("entered xz optimization")
        
        # Define optimization variables
        linear_velocity_x = cp.Variable()
        linear_velocity_z = cp.Variable()

    

        desired_velocity_x, desired_velocity_z = velll.linear.x, velll.linear.z
    

        # Define the objective: minimize deviation from desired velocities
        objective = cp.Minimize((cp.square(linear_velocity_x - desired_velocity_x)) + cp.square(linear_velocity_z - desired_velocity_z))

        # Define the constraint: ensure obstacle avoidance
       
        constraint = self.compute_h_dot_xz(linear_velocity_x ,linear_velocity_z) + self.kappa1 * (self.current_h - 0.5)
        
        constraints = [constraint >= 0]

        # Formulate the problem
        problem = cp.Problem(objective, constraints)

        try:
            # Solve the problem
            problem.solve()

            # Check if the problem was solved successfully
            if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                # Get the optimized velocities
              
                optimized_velocity_x,optimized_velocity_z = linear_velocity_x.value, linear_velocity_z.value
    

                safe_velocity = np.array([optimized_velocity_x, optimized_velocity_z])
                h_dddot=self.compute_h_dot_xz(optimized_velocity_x ,optimized_velocity_z) 
                optimal_constraint = h_dddot + self.kappa * (self.current_h - 0.5)
                rospy.loginfo(f"closest distance: {self.closest_obstacle_distance}")
                # rospy.loginfo(f"optimal_constraint: {optimal_constraint}")
                # rospy.loginfo(f"compute_h_dot_xz: {h_dddot}")
                # rospy.loginfo(f"current h: {self.current_h}")
                rospy.loginfo(f"z alg :{self.v_alg.linear.z}")
                rospy.loginfo(f"z opti :{optimized_velocity_z}")
                rospy.loginfo(f"x alg :{self.v_alg.linear.x}")
                rospy.loginfo(f"x opti :{optimized_velocity_x}")
                
                # rospy.loginfo(f"obj function: {cp.square(optimized_velocity_x - self.v_alg.linear.x) + cp.square(optimized_velocity_z - self.v_alg.linear.z)}")
                print("Optimization successful")
            else:
                print("Optimization failed: No optimal solution found")
                # Set both velocities to 0 if optimization fails
                safe_velocity = np.array([0.0, 0.0])

        except Exception as e:
            print(f"An error occurred during optimization: {e}")
            # Set both velocities to 0 if an error occurs
            safe_velocity = np.array([0.0, 0.0])

        return safe_velocity
    



    def process_data(self,valg):
        # Compute safe control input using CBF
        if self.xy_cbf:
            #rospy.loginfo(f"h {self.current_h}")
            if not self.current_h == float('inf'):
                v_safe = self.cbf_optimization_xy(valg)
                
                
                #rospy.loginfo(f"xyyyyyy cbf")
                # Publish safe control input
                self.publish_safe_control_input_xy(v_safe)
                
            
            else:
                v_safe = np.array([valg.linear.x, valg.linear.y])
                #print("else loop")
                self.publish_safe_control_input_xy(v_safe)
        # if self.xz_cbf: ## if sonar is moving
        #     if not self.current_h == float('inf'):
        #         rospy.loginfo(f"xzz cbf")
        #         v_safe = self.cbf_optimization_xz(valg)
                
        #         #rospy.loginfo(f"publishing optimised")
        #         # Publish safe control input
        #         self.publish_safe_control_input_xz(v_safe)
                
            
        #     else:
        #         v_safe = np.array([valg.linear.x, valg.linear.z])
        #         #print("else loop")
        #         self.publish_safe_control_input_xz(v_safe)
        #     # else: condition when rexrov stopped for motion       
        if self.xz_cbf:
            #rospy.loginfo(f"h {self.current_h}")
            if not self.current_h == float('inf'):
                v_safe = self.cbf_optimization_xz(valg)
                
                
                #rospy.loginfo(f"xyyyyyy cbf")
                # Publish safe control input
                self.publish_safe_control_input_xz(v_safe)
                
            
            else:
                v_safe = np.array([valg.linear.x, valg.linear.z])
                #print("else loop")
                self.publish_safe_control_input_xz(v_safe)



    def publish_safe_control_input_xy(self, velocity):
        # Create the Twist message
        twist = Twist()
        
        # rospy.loginfo(f"change in velcotiy x: { self.v_alg.linear.x - velocity[0]}")
        # rospy.loginfo(f"change in velcotiy y: { self.v_alg.linear.y - velocity[1]}")
        # rospy.loginfo(f"opti in velcotiy x: { velocity[0]}")
        # rospy.loginfo(f"opti in velcotiy y: { velocity[1] }")
        twist.angular.z = self.v_alg.angular.z  # Set the angular velocity
        twist.linear.x = velocity[0]
        twist.linear.y = velocity[1]
        
         # Publish the Twist message
        self.cmd_vel_pub.publish(twist)

    def publish_safe_control_input_xz(self, velocity):
        # Create the Twist message
        twist = Twist()
        
        # rospy.loginfo(f"change in velcotiy x: { self.v_alg.linear.x - velocity[0]}")
        # rospy.loginfo(f"change in velcotiy y: { self.v_alg.linear.y - velocity[1]}")
        # rospy.loginfo(f"opti in velcotiy x: { velocity[0]}")
        # rospy.loginfo(f"opti in velcotiy y: { velocity[1] }")
        self.h_pub.publish(self.current_h)
        twist.linear.x = velocity[0]
        twist.linear.z = velocity[1]
        
         # Publish the Twist message
        self.cmd_vel_pub.publish(twist)
            
        
if __name__ == '__main__':
    try:
        ObstacleAvoidanceNode()
    except rospy.ROSInterruptException:
        pass
