#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from networktables import NetworkTables
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
import time
import json
import numpy as np
from pytransform3d.transformations import transform_from_pq, invert_transform, transform

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
        self.camera_table = self.tag_table.getSubTable('Camera')
        
        # Load the field map of AprilTag positions
        with open('2025-reefscape.json', 'r') as file:
            self.field_map = json.load(file)
        
        # Map AprilTag IDs to their absolute poses on the field
        self.tag_map = {
            tag["ID"]: {
                "position": (
                    tag["pose"]["translation"]["x"],
                    tag["pose"]["translation"]["y"],
                    tag["pose"]["translation"]["z"]
                ),
                "orientation": (
                    tag["pose"]["rotation"]["quaternion"]["X"],
                    tag["pose"]["rotation"]["quaternion"]["Y"],
                    tag["pose"]["rotation"]["quaternion"]["Z"],
                    tag["pose"]["rotation"]["quaternion"]["W"]
                )
            }
            for tag in self.field_map["tags"]
        }
        
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

        # List of tags we care about
        tags_of_interest = [6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22]
        detected_tags = []

        # If there are detections, process them
        if num_detections > 0:
            for detection in msg.detections:
                if detection.id in tags_of_interest:
                    tag_id = detection.id
                    tag_pose = detection.pose.pose.pose
                    position = tag_pose.position
                    orientation = tag_pose.orientation

                    # Get the detected pose of the tag relative to the camera
                    detected_pose_relative_to_camera = (
                        (position.x, position.y, position.z),
                        (orientation.x, orientation.y, orientation.z, orientation.w)
                    )

                    # Calculate the camera's pose on the field
                    try:
                        camera_position, camera_orientation = self.calculate_camera_pose(
                            tag_id, detected_pose_relative_to_camera
                        )

                        # Store the detected tag information
                        detected_tags.append({
                            'id': tag_id,
                            'position': (position.x, position.y, position.z),
                            'orientation': (orientation.x, orientation.y, orientation.z, orientation.w)
                        })

                        # Post the camera's absolute pose to NetworkTables as a single array [position, orientation]
                        camera_data = list(camera_position) + list(camera_orientation.flatten())
                        self.camera_table.putNumberArray('Camera Absolute Pose', camera_data)

                    except ValueError as e:
                        self.get_logger().error(str(e))

            # Post the list of detected tags to NetworkTables as a single array [tag, position, orientation, tag, position, orientation, ...]
            tag_data = []
            for tag in detected_tags:
                tag_data.extend([tag['id']] + list(tag['position']) + list(tag['orientation']))
            self.tag_table.putNumberArray('Detected Tags Data', tag_data)

    def calculate_camera_pose(self, tag_id, detected_pose_relative_to_camera):
        """
        Calculate the camera's pose on the field using an AprilTag detection.

        :param tag_id: ID of the detected AprilTag.
        :param detected_pose_relative_to_camera: (position, orientation) of the tag relative to the camera.
        :return: Camera pose on the field as a (position, orientation) tuple.
        """
        if tag_id not in self.tag_map:
            raise ValueError(f"Tag ID {tag_id} not found in the map.")

        # Get the absolute pose of the tag on the field
        tag_absolute_pose = self.tag_map[tag_id]
        tag_to_field = transform_from_pq(tag_absolute_pose["position"] + tag_absolute_pose["orientation"])

        # Get the detected pose of the tag relative to the camera
        tag_to_camera = transform_from_pq(detected_pose_relative_to_camera[0] + detected_pose_relative_to_camera[1])

        # Calculate the camera's pose relative to the field
        camera_to_field = transform(tag_to_field, invert_transform(tag_to_camera))

        # Extract the camera's position and orientation from the transformation matrix
        camera_position = camera_to_field[:3, 3]
        camera_orientation = camera_to_field[:3, :3]  # Rotation matrix
        return camera_position, camera_orientation

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