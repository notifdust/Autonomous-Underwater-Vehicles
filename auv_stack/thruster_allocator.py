#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray
import numpy as np

class ThrusterAllocator(Node):
    def __init__(self):
        super().__init__('thruster_allocator')
        
        # Thruster Configuration Matrix (TCM)
        # This maps 4 thruster forces [T1, T2, T3, T4] to a 6DOF wrench
        # [Fx, Fy, Fz, Mx, My, Mz]
        # IMPORTANT NOTE : if you use this code, you MUST change this to suit you hardware
        self.B = np.array([
        #   T1(fwd,port) T2(fwd,stbd) T3(vert,fwd) T4(vert,aft)
            [1,           1,           0,           0],      # Fx (Surge)
            [0,           0,           0,           0],      # Fy (Sway)
            [0,           0,           1,           1],      # Fz (Heave)
            [0,           0,          -0.5,         0.5],    # Mx (Roll)
            [0,           0,           0,           0],      # My (Pitch)
            [-0.5,        0.5,         0,           0]       # Mz (Yaw)
        ])
        
        # Calculate the Moore-Penrose pseudo-inverse
        self.B_pinv = np.linalg.pinv(self.B)
        
        self.create_subscription(WrenchStamped, '/controller/wrench_command', self.wrench_callback, 10)
        self.thruster_pub = self.create_publisher(Float64MultiArray, '/thruster_commands', 10)
        self.get_logger().info('Thruster Allocator node started.')

    def wrench_callback(self, msg):
        tau = np.array([
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
        ])
        
        # u = B_inv * tau
        u = self.B_pinv @ tau
        
        # (Here you would add saturation/clipping to thruster limits)
        # u = np.clip(u, -100.0, 100.0) 
        
        thruster_msg = Float64MultiArray()
        thruster_msg.data = u.tolist()
        self.thruster_pub.publish(thruster_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterAllocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()