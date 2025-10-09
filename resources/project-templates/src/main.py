#!/usr/bin/env python3
"""
TensorFleet Drone Control Script
Basic example for autonomous flight operations
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time


class DroneController(Node):
    """Basic drone controller using ROS 2"""
    
    def __init__(self):
        super().__init__('tensorfleet_drone_controller')
        
        # Publishers
        self.position_pub = self.create_publisher(
            PoseStamped, 
            '/drone/setpoint_position', 
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/drone/status',
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('TensorFleet Drone Controller initialized')
        
        # Flight state
        self.current_waypoint = 0
        self.waypoints = [
            (0.0, 0.0, 5.0),   # Takeoff
            (10.0, 0.0, 5.0),  # Move forward
            (10.0, 10.0, 5.0), # Move right
            (0.0, 10.0, 5.0),  # Move back
            (0.0, 0.0, 5.0),   # Return to start
            (0.0, 0.0, 0.0),   # Land
        ]
    
    def control_loop(self):
        """Main control loop - publishes setpoints"""
        if self.current_waypoint < len(self.waypoints):
            x, y, z = self.waypoints[self.current_waypoint]
            
            # Create position setpoint
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            
            self.position_pub.publish(pose)
            
            # Publish status
            status = String()
            status.data = f"Waypoint {self.current_waypoint + 1}/{len(self.waypoints)}: ({x}, {y}, {z})"
            self.status_pub.publish(status)
            
            self.get_logger().info(status.data)
            
            # Simple waypoint progression (in real scenario, wait for position reached)
            time.sleep(5)  # Wait 5 seconds per waypoint
            self.current_waypoint += 1
        else:
            self.get_logger().info('Mission complete!')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    controller = DroneController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

