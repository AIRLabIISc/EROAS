#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import WrenchStamped
from scipy.optimize import minimize
from marine_acoustic_msgs.msg import ProjectedSonarImage
from collections import deque
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from sensor_msgs.msg import PointCloud2, JointState
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2 as pc2
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Float64



class CBFControlNode:
    def __init__(self):
        rospy.init_node('just_cbf', anonymous=True)
        
        
        self.vehicle_pose = None
        self.radius = 20

        self.filtered_points = np.empty((0, 3))

        # Initialize variables
        self.beam_directions = []
        self.ranges = []
        self.data = []
        self.ping_info = None
        self.obstacle_coordinates = []
        self.closest_obstacle_distance = float('inf')
        self.kappa = 10  # Tuning parameter for CBF
        self.R_o = 3.0  # Safety radius around obstacles
        self.mass = 1863  # Mass of the REXROV
        self.I_z = 691.23  # Moment of inertia around the z-axis


        self.last_time = rospy.get_time()
        self.last_h = float('inf')
        self.dh=0
        self.last_u = 0.001
        self.last_v = 0.001 
        self.last_w = 0.001
        self.last_r = 0.001
        self.consts_1=None
        self.h_dot_t=None
        self.all_points=None

        self.current_Th=None
        self.prev_Th=None

        self.current_h = float('inf')
        self.current_u = 0 
        self.current_v = 0
        self.current_w = 0
        self.current_r = 0


        # Dynamic model coefficients
        self.X_dot_u = 779.79
        self.Y_dot_v = 1222
        self.Z_dot_w = 3959.9
        self.N_dot_r = 224.32
        self.X_u = -74.82
        self.Y_v = -69.48
        self.Z_w = -782.4
        self.N_r = -105
        self.X_uu = -748.22
        self.Y_vv = -992.53
        self.Z_ww = -1821.01
        self.N_rr = -523.27

        self.T_alg = np.zeros(4)  # Initialize desired control input as zero
        self.T_roll= 0
        self.T_pitch= 0
        self.T_z=0
        #self.obstacle_history = deque(maxlen=1000)  # This is the length of storage unit 

        # Publisher for optimized body forces
        self.optimized_control_pub = rospy.Publisher('/rexrov2/thruster_manager/input_stamped', WrenchStamped, queue_size=10)
        # self.pub_on = rospy.Publisher('/rexrov2/global_angle', Float64, queue_size=10)
        
        # Subscribers using message_filters
        thrust_input_sub = Subscriber('/rexrov2/thruster_manager/input_stamped_1', WrenchStamped)
        self.pc_sub = rospy.Subscriber('/rexrov2/point_cloud', PointCloud2, self.point_cloud_callback)
        pose_sub = Subscriber('/rexrov2/pose_gt', Odometry)
        
        # ApproximateTimeSynchronizer to sync messages
        self.at_sync = ApproximateTimeSynchronizer([thrust_input_sub, pose_sub], queue_size=10, slop=0.03)
        self.at_sync.registerCallback(self.callback_all)


    def callback_all(self, thrust_input, pose):

        # rospy.loginfo(" ")
        rospy.loginfo("all Data received..")
        # rospy.loginfo(" ")

        # rospy.loginfo("callback_all triggered")
        # rospy.loginfo("Thrust Input Timestamp: %s", thrust_input.header.stamp)
        # rospy.loginfo("Pose Timestamp: %s", pose.header.stamp)
        
        # rospy.loginfo("Pose timestamp: %s", pose.header.stamp)
        self.pose_callback(pose)
        #self.point_cloud_callback(point_cloud)

        self.control_input_callback(thrust_input)

        self.process_data()
        
        
        

    def pose_callback(self, msg):
        
        
        # rospy.loginfo(f"current u : {self.current_u}")
        # Store vehicle pose (assuming position in global frame)
        self.last_u = self.current_u
        self.last_v = self.current_v
        self.last_w = self.current_w
        self.last_r = self.current_r

        self.vehicle_pose = msg.pose.pose.position
        self.current_u = msg.twist.twist.linear.x
        self.current_v = msg.twist.twist.linear.y
        self.current_w = msg.twist.twist.linear.z
        self.current_r = msg.twist.twist.angular.z

        vxx, vyy, vzz = self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z
        distance = np.linalg.norm(self.filtered_points - np.array([vxx, vyy, vzz]), axis=1)
        smallest_distance = np.min(distance) if distance.size > 0 else float('inf')
        if smallest_distance > self.radius:
            smallest_distance = float('inf')

        self.closest_obstacle_distance = smallest_distance
        
        self.current_h = self.closest_obstacle_distance - self.R_o

        

    def point_cloud_callback(self, msg):
        #rospy.loginfo("pc Data received..")

        self.last_h = self.current_h
        
        if self.vehicle_pose is None:
            rospy.loginfo("Vehicle pose not yet received")
            return

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
            rospy.loginfo(f"len of all_points: {len(self.filtered_points)}")
            distances = np.linalg.norm(all_points - np.array([vx, vy, vz]), axis=1)
            smallest_distance = np.min(distances) if distances.size > 0 else float('inf')
            if smallest_distance > self.radius:
                smallest_distance = float('inf')

            # Filter points within the radius
            within_radius_mask = distances <= self.radius
            self.filtered_points = all_points[within_radius_mask].tolist()

        else :
            self.filtered_points = np.empty((0, 3))
            #rospy.loginfo(f"Total stored points: {len(self.filtered_points)}")
            #rospy.loginfo(f"Smallest distance: {smallest_distance}")

        
    
    def control_input_callback(self, data):
        # Extract current control inputs from the existing algorithm
        #rospy.loginfo("control Data received..")
        self.prev_Th=self.current_Th

        self.T_alg = np.array([
            data.wrench.force.x,  # Tx
            data.wrench.force.y,  # Ty
              # Tz
            data.wrench.torque.z  # Tpsi
        ])
        self.T_z=data.wrench.force.z
        self.T_roll=data.wrench.torque.x
        self.T_pitch=data.wrench.torque.y

        # Process the data after receiving new control inputs
        
    def compute_state_derivative_of_h(self):
        self.current_time = rospy.get_time()

        dh = self.current_h - self.last_h
        self.dt = self.current_time - self.last_time
        du = self.current_u - self.last_u
        dv = self.current_v - self.last_v
        dw = self.current_w - self.last_w
        dr = self.current_r - self.last_r
        
        self.dh=dh
        # epsilon = 1e-6 

        # du = epsilon if du ==0 else du 
        # dv = epsilon if dv == 0 else dv
        # dw = epsilon if dw == 0 else dw
        # dr = epsilon if dr == 0 else dr

        #rospy.loginfo(f"dh : {dh}, du : {du}, dv : {dv}, dw : {dw}")
        # h_state_dot = np.array([dh/du, dh/dv, dh/dw, dh/dr])

        if du==0 or dv==0  or dw==0 or dr==0 :
            h_state_dot = np.array([float('inf'),float('inf'),float('inf')])

        else:
            h_state_dot = np.array([dh/du, dh/dv,dh/dr])


        return np.transpose(h_state_dot)
        
    def compute_h_dot(self, T, h_state_dot):
        # Compute the time derivative of h(x) based on control inputs T = [Tx, Ty, Tz, Tpsi]
        Tx, Ty,Tpsi= T
        
        # Compute vehicle dynamics based on REXROV equations
       # Compute vehicle dynamics based on REXROV equations
        u_dot = (Tx + (self.X_u + self.X_uu * np.abs(self.current_u)) * self.current_u + ( self.mass * self.current_r * self.current_v ) - ( self.Y_dot_v * self.current_r * self.current_v) / (self.mass - self.X_dot_u))
        v_dot = (Ty + (self.Y_v + self.Y_vv * np.abs(self.current_v)) * self.current_v + ( self.mass * self.current_u * self.current_r ) + ( self.X_dot_u * self.current_r * self.current_u) / (self.mass - self.Y_dot_v))
        #w_dot = (Tz + (self.Z_w + self.Z_ww * np.abs(self.current_w)) * self.current_w - (self.B - self.W)) / (self.mass - self.Z_dot_w)
        r_dot = (Tpsi + (self.N_r + self.N_rr * np.abs(self.current_r)) * self.current_r + ((self.Y_dot_v - self.X_dot_u) * self.current_u * self.current_v)) / (self.I_z - self.N_dot_r)


        # Vehicle state derivatives
        x_dot = np.array([u_dot, v_dot,r_dot])

        #rospy.loginfo(f"x_dot: {x_dot}")

        # Time derivative of the CBF
        #self.h_dot_t = np.dot(h_state_dot, x_dot)
        self.h_dot_t = np.dot(h_state_dot, x_dot)
        return self.h_dot_t

    def cbf_optimization(self, T_alg):
        # Solve optimization problem to find safe body forces
        def objective(T):
            # Minimize distance between current and desired inputs
            return np.linalg.norm(T - T_alg)**2
        

        def constraint(T):
            # Ensure CBF condition is satisfied
            h_state_dot = self.compute_state_derivative_of_h()
            h_dot =self.compute_h_dot(T, h_state_dot)
            consts_1= h_dot + self.kappa * self.current_h
            #rospy.loginfo(f"constrain n Value: {consts_1}")
            return consts_1
            #rospy.loginfo(f"Constraint Value: {const_1}")
             
            #return self.compute_h_dot(T,self.compute_state_derivative_of_h()) + self.kappa * self.current_h

        
        constraints = {'type': 'ineq', 'fun': constraint}
        options = {'disp': True, 'maxiter': 100}
        result = minimize(objective, T_alg,constraints=constraints, method='SLSQP', options=options)

        # rospy.loginfo(f"Optimization result : Success = {result.success}, Message = {result.message}, Fun={result.fun}")
        # rospy.loginfo(f"Optimized Control Inputs: T_safe={result.x if result.success else 0000}")
        # rospy.loginfo(f"Objective Function Value: {result.fun}")
        

        rospy.loginfo(f"h_dot: {self.h_dot_t}")
        #rospy.loginfo(f"constrain n Value: {consts_1}")
        rospy.loginfo(f"current_h: {self.current_h}")
   
        if result.success:
            #rospy.loginfo("Optimization is happening...")
            return result.x

        else:
            #rospy.logwarn("Optimization failed, using original input.")
            return T_alg

    def process_data(self):
        # Compute safe control input using CBF
        if self.prev_Th is not None:
            

            if not self.current_h == float('inf'):
                T_safe = self.cbf_optimization(self.T_alg)
                self.current_Th=T_safe
                rospy.loginfo(f"publishing optimised")
                # Publish safe control input
                self.publish_safe_control_input(T_safe)
                
            
            else:
                self.publish_safe_control_input(self.T_alg)
                self.current_Th=self.T_alg
        else:
            rospy.loginfo(f"1st command")
            self.publish_safe_control_input(self.T_alg)
            self.current_Th=self.T_alg

    def publish_safe_control_input(self, T_safe):
        # Create WrenchStamped message for optimized control input
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = rospy.Time.now()
        wrench_msg.header.frame_id = "/rexrov2/base_link"
        wrench_msg.wrench.force.x = T_safe[0]
        wrench_msg.wrench.force.y = T_safe[1]
        wrench_msg.wrench.force.z = self.T_z
        wrench_msg.wrench.torque.x = self.T_roll  # Assuming no roll torque control
        wrench_msg.wrench.torque.y = self.T_pitch  # Assuming no pitch torque control
        wrench_msg.wrench.torque.z = self.T_alg[2]

        # Publish the optimized control input
        self.optimized_control_pub.publish(wrench_msg)
        rospy.loginfo(f"change in  optimized control input: Tx={T_safe[0]-self.T_alg[0]}, Ty={T_safe[1]-self.T_alg[1]}, Tz={T_safe[2]-self.T_alg[2]}")
        #rospy.loginfo(f"Publishing PID control input: Tx={self.T_alg[0]}, Ty={self.T_alg[1]}, Tz={self.T_alg[2]}, Tpsi={self.T_alg[3]}")
        #rospy.loginfo(f"Smallest distance: {self.closest_obstacle_distance}")
        
        # rospy.loginfo(f"dh: {self.dh}")
        # rospy.loginfo(f"h_dot: {self.h_dot_t}")
        # rospy.loginfo(f"constarin: {self.consts_1}")
        # rospy.loginfo(f" ")
        # rospy.loginfo(f" ")

    def run(self):
        rospy.spin()  # Keep the node running
        self.last_time = self.current_time 

if __name__ == '__main__':
    try:
        node = CBFControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
