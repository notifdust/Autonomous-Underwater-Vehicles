#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
import casadi as ca
import numpy as np

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
        # MPC Parameters
        self.N = 20  # Prediction horizon
        self.dt = 0.1 # Time step (controller frequency)
        
        # State: [z, theta, w, q] (depth, pitch, vert_vel, pitch_rate)
        # Control: [Fz, My] (vertical_force, pitch_torque)
        # This is a simplified 2D (depth/pitch) model for clarity.
        
        self.opti = ca.Opti()
        self.X = self.opti.variable(4, self.N + 1) # State trajectory
        self.U = self.opti.variable(2, self.N)     # Control trajectory
        self.X0 = self.opti.parameter(4, 1)        # Initial state
        self.X_ref = self.opti.parameter(4, 1)     # Target state (setpoint)

        # Simplified AUV model (dynamics)
        mass, inertia_y = 100.0, 40.0
        damping_z, damping_q = 50.0, 20.0
        
        def dynamics(x, u):
            z, theta, w, q = x[0], x[1], x[2], x[3]
            Fz, My = u[0], u[1]
            
            # Assume 1N net positive buoyancy
            F_buoyancy = -1.0 
            
            z_dot = w * ca.cos(theta) 
            theta_dot = q
            w_dot = (Fz + F_buoyancy - damping_z * w) / mass
            q_dot = (My - damping_q * q) / inertia_y
            
            dxdt = ca.vertcat(z_dot, theta_dot, w_dot, q_dot)
            return x + self.dt * dxdt # Euler integration

        # Define the optimization problem
        cost = 0
        Q = ca.diag([10.0, 5.0, 1.0, 1.0]) # State cost
        R = ca.diag([0.1, 0.1])            # Control cost

        self.opti.subject_to(self.X[:, 0] == self.X0)
        for k in range(self.N):
            self.opti.subject_to(self.X[:, k+1] == dynamics(self.X[:, k], self.U[:, k]))
            state_error = self.X[:, k] - self.X_ref
            control_effort = self.U[:, k]
            cost += ca.mtimes([state_error.T, Q, state_error])
            cost += ca.mtimes([control_effort.T, R, control_effort])
            
        self.opti.subject_to(self.opti.bounded(-100.0, self.U[0, :], 100.0)) # Fz limit
        self.opti.subject_to(self.opti.bounded(-50.0,  self.U[1, :], 50.0))  # My limit

        self.opti.minimize(cost)
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        self.opti.solver('ipopt', opts)

        self.target_depth = 0.0
        self.current_state = np.zeros(4)
        
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(Float64, '/target/depth', self.depth_sp_callback, 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, '/controller/wrench_command', 10)
        
        self.timer = self.create_timer(self.dt, self.run_mpc)
        self.get_logger().info('MPC Controller node started.')

    def odom_callback(self, msg):
        # (This is simplified, you'd need a quaternion-to-euler conversion)
        z = msg.pose.pose.position.z
        theta = 0.0 # Placeholder for pitch
        w = msg.twist.twist.linear.z
        q = msg.twist.twist.angular.y
        self.current_state = np.array([z, theta, w, q])

    def depth_sp_callback(self, msg):
        self.target_depth = msg.data

    def run_mpc(self):
        self.opti.set_value(self.X0, self.current_state)
        target_state = np.array([self.target_depth, 0, 0, 0])
        self.opti.set_value(self.X_ref, target_state)
        
        try:
            sol = self.opti.solve()
            optimal_u = sol.value(self.U[:, 0])
            
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = self.get_clock().now().to_msg()
            wrench_msg.wrench.force.z = optimal_u[0]
            wrench_msg.wrench.torque.y = optimal_u[1]
            self.wrench_pub.publish(wrench_msg)
            
        except Exception as e:
            self.get_logger().warn(f"MPC Solver failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()