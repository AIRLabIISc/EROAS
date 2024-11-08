import numpy as np
import casadi as ca
import matplotlib.pyplot as plt

# Time step
dt = 0.01

# PID controller parameters
kp = 2.0
ki = 0.1
kd = 0.5

# Control input limits
v_const = 0.5       # Constant linear speed
omega_max = np.pi   # Maximum angular speed

# Obstacle parameters
obs = [4.0, 4.0]    # Obstacle position (x, y)
r_obs = 0.5         # Obstacle radius
r_safe = 2.0        # Safety margin
R = r_obs + r_safe  # Total safety radius

# Goal position
goal = [8.0, 8.0]

# Initial state (x, y, theta)
x = 0.0
y = 0.0
theta = 0.0

# PID controller state
E = 0.0
old_e = 0.0

# Trajectory storage
trajectory = []
cbf_trajectory = []

h_values = []
h_dot_plus_kh_values = []
control_inputs = []
dist_to_obstacle_values = []

def pid_controller(x, y, theta, goal, E, old_e):
    dx = goal[0] - x
    dy = goal[1] - y
    g_theta = np.arctan2(dy, dx)

    alpha = g_theta - theta
    err = np.arctan2(np.sin(alpha), np.cos(alpha))

    ep = err
    ei = E + err * dt
    ed = (err - old_e) / dt

    omega = kp * ep + ki * ei + kd * ed
    omega = np.clip(omega, -omega_max, omega_max)

    E = ei
    old_e = err

    return omega, E, old_e

def cbf_filter(x, y, theta, omega_pid, obs, R, return_debug=False):
    opti = ca.Opti()
    omega = opti.variable()

    x_casadi = opti.parameter()
    y_casadi = opti.parameter()
    theta_casadi = opti.parameter()
    opti.set_value(x_casadi, x)
    opti.set_value(y_casadi, y)
    opti.set_value(theta_casadi, theta)

    distance_to_obstacle = ca.sqrt((x_casadi - obs[0])**2 + (y_casadi - obs[1])**2)
    h = distance_to_obstacle - R

    # Calculate the derivative of h with respect to x, y
    dh_dx = ca.gradient(distance_to_obstacle, x_casadi)
    dh_dy = ca.gradient(distance_to_obstacle, y_casadi)

    # Dynamics of the unicycle model affecting h_dot
    h_dot = dh_dx * v_const * ca.cos(theta_casadi) + dh_dy * v_const * ca.sin(theta_casadi) + omega

    # Objective: Minimize deviation from PID omega
    opti.minimize(0.5 * (omega - omega_pid)**2)

    # CBF constraint: Ensure safety
    k1 = 5.0
    fnction = h_dot + k1 * h
    opti.subject_to(fnction >= 0)

    # Control input bounds
    opti.subject_to(opti.bounded(-omega_max, omega, omega_max))

    # Solver options
    p_opts = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
    s_opts = {}
    opti.solver('ipopt', p_opts, s_opts)

    try:
        sol = opti.solve()
        omega_opt = sol.value(omega)
        if return_debug:
            return omega_opt, sol.value(h), sol.value(h_dot) + k1 * sol.value(h)
        else:
            return omega_opt
    except RuntimeError:
        if return_debug:
            return omega_pid, 0, 0  # If solver fails, use PID output
        else:
            return omega_pid

# Simulation parameters
max_steps = 10000
tolerance = 0.1
goal_reached = False

# Main simulation loop
for step in range(max_steps):
    dist_to_goal = np.hypot(goal[0] - x, goal[1] - y)
    if dist_to_goal < tolerance:
        print("Goal reached!")
        goal_reached = True
        break

    omega_pid, E, old_e = pid_controller(x, y, theta, goal, E, old_e)
    dist_to_obs = np.hypot(obs[0] - x, obs[1] - y)

    if dist_to_obs <= R + 1.0:
        omega_cbf, h, h_dot_plus_kh = cbf_filter(x, y, theta, omega_pid, obs, R, return_debug=True)
        cbf_trajectory.append([x, y])
        h_values.append(h)
        h_dot_plus_kh_values.append(h_dot_plus_kh)
    else:
        omega_cbf = omega_pid

    x += v_const * np.cos(theta) * dt
    y += v_const * np.sin(theta) * dt
    theta += omega_cbf * dt
    theta = np.arctan2(np.sin(theta), np.cos(theta))

    trajectory.append([x, y])
    control_inputs.append(omega_cbf)
    dist_to_obstacle_values.append(dist_to_obs)

# Convert lists to numpy arrays for plotting
trajectory = np.array(trajectory)
cbf_trajectory = np.array(cbf_trajectory) if len(cbf_trajectory) > 0 else None

# Plot trajectories and obstacles
plt.figure(figsize=(12, 12))
plt.subplot(2, 3, 1)
plt.plot(trajectory[:, 0], trajectory[:, 1], label='Trajectory', color='blue')
plt.scatter(goal[0], goal[1], c='red', marker='*', s=100, label='Goal')
plt.scatter(obs[0], obs[1], c='orange', s=100, label='Obstacle')
plt.gca().add_patch(plt.Circle((obs[0], obs[1]), r_obs, color='orange', alpha=0.5))
plt.gca().add_patch(plt.Circle((obs[0], obs[1]), R, color='red', alpha=0.3, linestyle='--'))
plt.xlabel('X position')
plt.ylabel('Y position')
plt.title('Simulation of Unicycle Model with CBF')
plt.legend()
plt.grid(True)

# Additional debugging information
plt.subplot(2, 3, 2)
plt.plot(h_values, label='Barrier Function h')
plt.title('Barrier Function h over Time')
plt.xlabel('Time Step')
plt.ylabel('h')
plt.grid(True)
plt.legend()

plt.subplot(2, 3, 3)
plt.plot(h_dot_plus_kh_values, label='h_dot + k*h')
plt.title('Derivative of Barrier Function over Time')
plt.xlabel('Time Step')
plt.ylabel('h_dot + k*h')
plt.grid(True)
plt.legend()

plt.subplot(2, 3, 4)
plt.plot(control_inputs, label='Control Input Omega')
plt.title('Control Input over Time')
plt.xlabel('Time Step')
plt.ylabel('Omega')
plt.grid(True)
plt.legend()

plt.subplot(2, 3, 5)
plt.plot(dist_to_obstacle_values, label='Distance to Obstacle')
plt.title('Distance to Obstacle over Time')
plt.xlabel('Time Step')
plt.ylabel('Distance (m)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
