#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import time

class MissionState:
    START = 0
    DIVING = 1
    ON_STATION = 2
    SURFACING = 3
    END = 4

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        self.state = MissionState.START
        
        self.target_depth = -20.0 # meters
        self.current_depth = 0.0
        
        self.depth_sp_pub = self.create_publisher(Float64, '/target/depth', 10)
        self.heading_sp_pub = self.create_publisher(Float64, '/target/heading', 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        self.timer = self.create_timer(1.0, self.run_state_machine)
        self.get_logger().info('Mission Planner node started.')

    def odom_callback(self, msg):
        self.current_depth = msg.pose.pose.position.z

    def run_state_machine(self):
        if self.state == MissionState.START:
            self.get_logger().info("Mission starting... Commanding dive to 20m.")
            self.state = MissionState.DIVING
            
        elif self.state == MissionState.DIVING:
            self.depth_sp_pub.publish(Float64(data=self.target_depth))
            self.heading_sp_pub.publish(Float64(data=90.0)) # Hold 90 deg heading
            
            if abs(self.current_depth - self.target_depth) < 0.5:
                self.get_logger().info("Target depth reached. Holding station for 30s.")
                self.station_start_time = time.time()
                self.state = MissionState.ON_STATION
                
        elif self.state == MissionState.ON_STATION:
            self.depth_sp_pub.publish(Float64(data=self.target_depth))
            self.heading_sp_pub.publish(Float64(data=90.0))
            
            if (time.time() - self.station_start_time) > 30.0:
                self.get_logger().info("Station keeping complete. Surfacing.")
                self.state = MissionState.SURFACING
                
        elif self.state == MissionState.SURFACING:
            self.depth_sp_pub.publish(Float64(data=0.0))
            
            if abs(self.current_depth - 0.0) < 0.5:
                self.get_logger().info("Surface reached. Mission complete.")
                self.state = MissionState.END
                self.timer.cancel()
                
        elif self.state == MissionState.END:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()