#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3 
import numpy as np

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.SAFETY_RADIUS = 5.0 # 5 meters
        
        self.create_subscription(LaserScan, '/sonar_scan', self.sonar_callback, 10)
        # Publishes a "repulsive vector" for the MPC to use as a constraint
        self.repulsion_pub = self.create_publisher(Vector3, '/obstacle_repulsion', 10)
        self.get_logger().info('Obstacle Avoidance node started.')

    def sonar_callback(self, msg):
        repulsion_vector = Vector3()
        min_dist = float('inf')
        
        for i, dist in enumerate(msg.ranges):
            if dist < min_dist:
                min_dist = dist
            
            if dist < self.SAFETY_RADIUS:
                angle = msg.angle_min + i * msg.angle_increment
                # Simple potential field: force is 1/r^2
                magnitude = (1.0 / dist**2)
                repulsion_vector.x -= magnitude * np.cos(angle)
                repulsion_vector.y -= magnitude * np.sin(angle)
        
        if min_dist < self.SAFETY_RADIUS:
            self.repulsion_pub.publish(repulsion_vector)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()