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
        self.server_ip = '127.0.0.1'  # Replace with your NT server IP
        NetworkTables.initialize(server=self.server_ip)
        
        # Wait for NetworkTables connection
        while not NetworkTables.isConnected():
            self.get_logger().info("Waiting for NetworkTables connection...")
            time.sleep(1)
        self.get_logger().info("Connected to NetworkTables!")
        
        # Get the root NetworkTables table
        self.root_table = NetworkTables.getTable('AprilTags')
        
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

        # Tags of interest
        self.tags_of_interest = [6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22]

        # Camera topics to subscribe to
        self.camera_topics = ['/camera1/apriltag_detections', '/camera2/apriltag_detections']  # Add more as needed

        # Create a subtable for each camera
        self.camera_tables = {}
        for topic in self.camera_topics:
            camera_name = topic.split('/')[1]  # Extract camera name from topic (e.g., 'camera1')
            self.camera_tables[camera_name] = self.root_table.getSubTable(camera_name)

        # ROS2 subscribers for each camera
        self.subscriptions = []
        for topic in self.camera_topics:
            self.subscriptions.append(
                self.create_subscription(
                    AprilTagDetectionArray,
                    topic,
                    lambda msg, topic=topic: self.apriltag_callback(msg, topic),
                    10
                )
            )

    def apriltag_callback(self, msg, topic):
        camera_name = topic.split('/')[1]  # Extract camera name from topic
        detected_tags = []
        total_distance = 0.0
        num_tags = 0

        # If there are detections, process them
        for detection in msg.detections:
            tag_id = detection.id[0]
            if tag_id not in self.tags_of_interest:
                continue

            position = (
                detection.pose.pose.position.x,
                detection.pose.pose.position.y,
                detection.pose.pose.position.z
            )
            orientation = (
                detection.pose.pose.orientation.x,
                detection.pose.pose.orientation.y,
                detection.pose.pose.orientation.z,
                detection.pose.pose.orientation.w
            )

            try:
                camera_position, camera_orientation = self.calculate_camera_pose(tag_id, (position, orientation))

                # Store the detected tag information
                detected_tags.append({
                    'id': tag_id,
                    'position': position,
                    'orientation': orientation
                })

                # Calculate distance to the tag
                distance = np.linalg.norm(position)
                total_distance += distance
                num_tags += 1

                # Post the camera's absolute pose to NetworkTables under the camera's subtable
                camera_data = list(camera_position) + list(camera_orientation)
                self.camera_tables[camera_name].putNumberArray('Absolute Pose', camera_data)

            except ValueError as e:
                self.get_logger().warn(str(e))

        # Post the list of detected tags to NetworkTables under the camera's subtable
        tag_data = []
        for tag in detected_tags:
            tag_data.extend([tag['id']] + list(tag['position']) + list(tag['orientation']))
        self.camera_tables[camera_name].putNumberArray('Detected Tags Data', tag_data)

        # Calculate and post the average distance to all tags under the camera's subtable
        if num_tags > 0:
            average_distance = total_distance / num_tags
            self.camera_tables[camera_name].putNumber('Average Distance', average_distance)

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
        camera_to_field = transform(tag_to_field, invert_transform(tag_to_camera))

        camera_orientation = camera_to_field[:3, :3]
        trace = np.trace(camera_orientation)
        if trace > 0:
            qw = np.sqrt(1.0 + trace) / 2.0
            qx = (camera_orientation[2, 1] - camera_orientation[1, 2]) / (4.0 * qw)
            qy = (camera_orientation[0, 2] - camera_orientation[2, 0]) / (4.0 * qw)
            qz = (camera_orientation[1, 0] - camera_orientation[0, 1]) / (4.0 * qw)
        else:
            if camera_orientation[0, 0] > camera_orientation[1, 1] and camera_orientation[0, 0] > camera_orientation[2, 2]:
                s = 2.0 * np.sqrt(1.0 + camera_orientation[0, 0] - camera_orientation[1, 1] - camera_orientation[2, 2])
                qw = (camera_orientation[2, 1] - camera_orientation[1, 2]) / s
                qx = 0.25 * s
                qy = (camera_orientation[0, 1] + camera_orientation[1, 0]) / s
                qz = (camera_orientation[0, 2] + camera_orientation[2, 0]) / s
            elif camera_orientation[1, 1] > camera_orientation[2, 2]:
                s = 2.0 * np.sqrt(1.0 + camera_orientation[1, 1] - camera_orientation[0, 0] - camera_orientation[2, 2])
                qw = (camera_orientation[0, 2] - camera_orientation[2, 0]) / s
                qx = (camera_orientation[0, 1] + camera_orientation[1, 0]) / s
                qy = 0.25 * s
                qz = (camera_orientation[1, 2] + camera_orientation[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + camera_orientation[2, 2] - camera_orientation[0, 0] - camera_orientation[1, 1])
                qw = (camera_orientation[1, 0] - camera_orientation[0, 1]) / s
                qx = (camera_orientation[0, 2] + camera_orientation[2, 0]) / s
                qy = (camera_orientation[1, 2] + camera_orientation[2, 1]) / s
                qz = 0.25 * s

        return (camera_to_field[:3, 3], (qx, qy, qz, qw))

def main(args=None):
    rclpy.init(args=args)
    bridge = ROS2NetworkTablesBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info("Shutting down...")
    finally:
        # Clean up
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()