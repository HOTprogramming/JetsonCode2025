#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from networktables import NetworkTables
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
import time

class ROS2NetworkTablesBridge(Node):
    def __init__(self):
        super().__init__('ros2_networktables_bridge')
        
        # Initialize NetworkTables
        self.server_ip = '10.0.67.2'  # Replace with your NT server IP
        NetworkTables.initialize(server=self.server_ip)
        
        # Wait for NetworkTables connection
        while not NetworkTables.isConnected():
            self.get_logger().info("Waiting for NetworkTables connection...")
            time.sleep(1)
        self.get_logger().info("Connected to NetworkTables!")
        
        # Get NetworkTables tables - this will create the table if it doesn't exist
        self.tag_table = NetworkTables.getTable('AprilTags')
        
        # Create ROS subscriber for AprilTag detections
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_callback,
            10
        )
        self.get_logger().info("Subscribed to /tag_detections")
        
    def tag_callback(self, msg):
        """Callback for AprilTag detection data"""
        # Extract timestamp seconds and nanoseconds
        timestamp_sec = msg.header.stamp.sec
        timestamp_nanosec = msg.header.stamp.nanosec
        
        # Post to NetworkTables
        self.tag_table.putNumber('timestamp_sec', timestamp_sec)
        self.tag_table.putNumber('timestamp_nanosec', timestamp_nanosec)
        
        # Log the data
        self.get_logger().info(f'Received timestamp: {timestamp_sec}.{timestamp_nanosec} seconds')
        
        # Also log if we got any detections
        num_detections = len(msg.detections)
        self.tag_table.putNumber('num_detections', num_detections)
        self.get_logger().info(f'Number of detections: {num_detections}')

        # If there are detections, log the first one's ID and position
        if num_detections > 0:
            first_detection = msg.detections[0]
            self.tag_table.putNumber('latest_tag_id', first_detection.id)
            
            # Get position data (if available in the message)
            try:
                pose = first_detection.pose.pose
                position = pose.position
                self.tag_table.putNumber('tag_x', position.x)
                self.tag_table.putNumber('tag_y', position.y)
                self.tag_table.putNumber('tag_z', position.z)
                self.get_logger().info(f'Tag ID {first_detection.id} at position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}')
            except AttributeError:
                self.get_logger().info(f'Tag ID {first_detection.id} detected (position data not available)')

def main(args=None):
    rclpy.init(args=args)
    bridge = ROS2NetworkTablesBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
