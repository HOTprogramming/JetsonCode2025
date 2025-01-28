#!/usr/bin/python3
from networktables import NetworkTables
import time
import json
import numpy as np
from pytransform3d.transformations import (
    transform_from_pq, 
    invert_transform, 
    transform)
import random

class NetworkTablesBridge:
    def __init__(self):
        # Initialize NetworkTables
        self.server_ip = '127.0.0.1'  # Replace with your NT server IP
        NetworkTables.initialize(server=self.server_ip)

        # Wait for NetworkTables connection
        while not NetworkTables.isConnected():
            print("Waiting for NetworkTables connection...")
            time.sleep(1)
        print("Connected to NetworkTables!")

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

    def simulate_tag_detections(self):
        """Simulate AprilTag detections with fake data and send to NetworkTables"""
        # Generate a random number of detections (between 1 and 5)
        num_detections = random.randint(1, 5)
        print(f"Simulating {num_detections} fake AprilTag detections")

        # List of tags we care about
        tags_of_interest = [6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22]
        detected_tags = []

        # Generate fake detections
        for _ in range(num_detections):
            tag_id = random.choice(tags_of_interest)  # Random tag ID from the list
            position = (
                random.uniform(-1.0, 1.0),  # Random position
                random.uniform(-1.0, 1.0),
                random.uniform(-1.0, 1.0)
            )
            orientation = (
                random.uniform(-1.0, 1.0),  # Random orientation
                random.uniform(-1.0, 1.0),
                random.uniform(-1.0, 1.0),
                random.uniform(-1.0, 1.0)
            )

            # Process each tag
            try:
                camera_position, camera_orientation = self.calculate_camera_pose(tag_id, (position, orientation))

                # Store the detected tag information
                detected_tags.append({
                    'id': tag_id,
                    'position': position,
                    'orientation': orientation
                })

                # Post the camera's absolute pose to NetworkTables
                camera_data = list(camera_position) + list(camera_orientation.flatten())
                self.camera_table.putNumberArray('Camera Absolute Pose', camera_data)

            except ValueError as e:
                print(str(e))

        # Post the list of detected tags to NetworkTables
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

def main():
    bridge = NetworkTablesBridge()
    
    try:
        while True:
            bridge.simulate_tag_detections()
            time.sleep(1.0)  # Simulate detections every 1 second
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == '__main__':
    main()
