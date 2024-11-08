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



class CBFControlNode:
    def __init__(self):
        rospy.init_node('just_cbf', anonymous=True)
        
        # Publisher for optimized body forces
        self.optimized_control_pub = rospy.Publisher('/rexrov2/thruster_manager/input_stamped', WrenchStamped, queue_size=10)
        self.optimized_control_pub = rospy.Publisher('/rexrov2/cbf_on', WrenchStamped, queue_size=10)
        
        # Subscribers for sonar data and thruster manager input
        rospy.Subscriber('/rexrov2/thruster_manager/input_stamped_1', WrenchStamped, self.control_input_callback)
        
        self.pc_sub = rospy.Subscriber('/rexrov2/point_cloud', PointCloud2, self.point_cloud_callback)
        
        self.pose_sub = rospy.Subscriber('/rexrov2/pose_gt', Odometry, self.pose_callback)

        self.vehicle_pose = None
        self.radius = 10

        self.filtered_points = []

        # Initialize variables
        self.beam_directions = []
        self.ranges = []
        self.data = []
        self.ping_info = None
        self.obstacle_coordinates = []
        self.closest_obstacle_distance = float('inf')
        self.kappa = 1.0  # Tuning parameter for CBF
        self.R_o = 1.0  # Safety radius around obstacles
        self.mass = 1863  # Mass of the REXROV
        self.I_z = 691.23  # Moment of inertia around the z-axis


        self.last_time = rospy.get_time()
        self.last_h = float('inf')
        self.last_u = 0.001
        self.last_v = 0.001 
        self.last_w = 0.001
        self.last_r = 0.001

        self.current_h = float('inf')
        self.current_u = None 
        self.current_v = None
        self.current_w = None
        self.current_r = None


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
        self.T_roll=0
        self.T_pitch=0


        self.obstacle_history = deque(maxlen=10000)  # This is the length of storage unit 

    def pose_callback(self, msg):
        # rospy.loginfo("Data received..")
        # rospy.loginfo(f"current u : {self.current_u}")
        # Store vehicle pose (assuming position in global frame)

        #store the previsous values
        self.last_u = self.current_u
        self.last_v = self.current_v
        self.last_w = self.current_w
        self.last_r = self.current_r

        #now update the value
        self.vehicle_pose = msg.pose.pose.position
        self.current_u = msg.twist.twist.linear.x
        self.current_v = msg.twist.twist.linear.y
        self.current_w = msg.twist.twist.linear.z
        self.current_r = msg.twist.twist.angular.z

        


    def point_cloud_callback(self, msg):
        #store the previsous h value
        self.last_h = self.current_h

        if self.vehicle_pose is None:
            rospy.loginfo("Vehicle pose not yet received")
            return

        # Convert point cloud to numpy array
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        new_points = np.array(list(pc_data))

        # Extract vehicle position
        vx, vy, vz = self.vehicle_pose.x, self.vehicle_pose.y, self.vehicle_pose.z

        # Concatenate existing filtered points with new points
        if len(self.filtered_points) > 0:
            filtered_points_np = np.array(self.filtered_points)
            all_points = np.vstack((filtered_points_np, new_points))
            
        else:
            all_points = new_points

        # Compute distances for all points
        distances = np.linalg.norm(all_points - np.array([vx, vy, vz]), axis=1)
        smallest_distance = np.min(distances) if distances.size > 0 else float('inf')
        if smallest_distance > self.radius:
            smallest_distance = float('inf')

        # Filter points within the radius
        within_radius_mask = distances <= self.radius
        self.filtered_points = all_points[within_radius_mask].tolist()

        

        rospy.loginfo(f"Total stored points: {len(self.filtered_points)}")
        rospy.loginfo(f"Smallest distance: {smallest_distance}")

        self.closest_obstacle_distance = smallest_distance

        rospy.loginfo(f"closest_obstacle_distance:{self.closest_obstacle_distance}")
        #now update the value
        self.current_h = self.closest_obstacle_distance - self.R_o

    
    def control_input_callback(self, data):
        # Extract current control inputs from the existing algorithm
        self.T_alg = np.array([
            data.wrench.force.x,  # Tx
            data.wrench.force.y,  # Ty
            data.wrench.force.z,  # Tz
            data.wrench.torque.z  # Tpsi
        ])

        self.T_roll=data.wrench.torque.x
        self.T_pitch=data.wrench.torque.y

        # Process the data after receiving new control inputs
        self.process_data()

    def compute_h(self):
        # Compute Control Barrier Function h(x)
        h = self.closest_obstacle_distance - self.R_o
        
        return h
    
    def compute_state_derivative_of_h(self):
        self.current_time = rospy.get_time()
        


        dh = self.current_h - self.last_h
        self.dt = self.current_time - self.last_time
        du = self.current_u - self.last_u
        dv = self.current_v - self.last_v
        dw = self.current_w - self.last_w
        dr = self.current_r - self.last_r

        rospy.loginfo(f"h:{self.current_h}")
        rospy.loginfo(f"dh: {dh},new_u: {du},old_: {dv},dw: {dw},")

        if du==0 or dv==0  or dw==0 or dr==0 :
            h_state_dot = np.array([float('inf'),float('inf'),float('inf'),float('inf')])

        else:
            h_state_dot = np.array([dh/du, dh/dv, dh/dw, dh/dr])

        

        

        return np.transpose(h_state_dot)
        

    def compute_h_dot(self, T, h_state_dot):
        # Compute the time derivative of h(x) based on control inputs T = [Tx, Ty, Tz, Tpsi]
        Tx, Ty, Tz, Tpsi = T
        
        # Compute vehicle dynamics based on REXROV equations
        u_dot = (Tx + (self.X_u + self.X_uu * np.abs(Tx)) * Tx + self.mass * Tpsi * Ty - self.Y_dot_v * Tpsi * Ty) / (self.mass - self.X_dot_u)
        v_dot = (Ty + (self.Y_v + self.Y_vv * np.abs(Ty)) * Ty + self.mass * Tpsi * Tx + self.X_dot_u * Tpsi * Tx) / (self.mass - self.Y_dot_v)
        w_dot = (Tz + (self.Z_w + self.Z_ww * np.abs(Tz)) * Tz - (self.mass - self.mass * 9.81)) / (self.mass - self.Z_dot_w)
        r_dot = (Tpsi + (self.N_r + self.N_rr * np.abs(Tpsi)) * Tpsi + (self.Y_dot_v - self.X_dot_u) * Tx * Ty) / (self.I_z - self.N_dot_r)

        # Vehicle state derivatives
        x_dot = np.array([u_dot, v_dot, w_dot, r_dot])

        #rospy.loginfo(f"x_dot: {x_dot}")

        # Time derivative of the CBF
        #self.h_dot_t = np.dot(h_state_dot, x_dot)
        return np.dot(h_state_dot, x_dot)

    def cbf_optimization(self, T_alg):
        # Solve optimization problem to find safe body forces
        def objective(T):
            # Minimize distance between current and desired inputs
            return np.linalg.norm(T - T_alg)**2

        def constraint(T):
            # Ensure CBF condition is satisfied
            return self.compute_h_dot(T,self.compute_state_derivative_of_h()) + self.kappa * self.current_h

        
        constraints = {'type': 'ineq', 'fun': constraint}
        options = {'disp': True, 'maxiter': 1000}
        result = minimize(objective, T_alg,constraints=[constraints], method='SLSQP', options=options)

        #rospy.loginfo(f"T: {T_alg}, compute_state_derivative_of_h: {self.compute_state_derivative_of_h()}, compute_h_dot: {self.compute_h_dot(T_alg,self.compute_state_derivative_of_h())}")
   
        if result.success:
            return result.x
        else:
            rospy.logwarn("Optimization failed, using original input.")
            return T_alg

    def process_data(self):
        # Compute safe control input using CBF
        T_safe = self.cbf_optimization(self.T_alg)

        # Publish safe control input
        self.publish_safe_control_input(T_safe)

    def publish_safe_control_input(self, T_safe):
        # Create WrenchStamped message for optimized control input
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = rospy.Time.now()
        wrench_msg.header.frame_id = "/rexrov2/base_link"
        wrench_msg.wrench.force.x = T_safe[0]
        wrench_msg.wrench.force.y = T_safe[1]
        wrench_msg.wrench.force.z = T_safe[2]
        wrench_msg.wrench.torque.x = self.T_roll  # Assuming no roll torque control
        wrench_msg.wrench.torque.y = self.T_pitch  # Assuming no pitch torque control
        wrench_msg.wrench.torque.z = T_safe[3]

        # Publish the optimized control input
        self.optimized_control_pub.publish(wrench_msg)
        rospy.loginfo(f"Published optimized control input: Tx={T_safe[0]}, Ty={T_safe[1]}, Tz={T_safe[2]}, Tpsi={T_safe[3]}")

    def run(self):
        rospy.spin()  # Keep the node running
        self.last_time = self.current_time 

if __name__ == '__main__':
    try:
        node = CBFControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
