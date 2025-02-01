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
import transformations as tf


#CAMERA 1 CONFIG
camera1_name = "camera1"
camera1_position = (
			0.3,
			0,
			0
		)

camera1_orientation_q = tf.quaternion_from_euler(0, 0, 90)
camera1_orientation = (
	camera1_orientation_q[0],
	camera1_orientation_q[1],
	camera1_orientation_q[2],
	camera1_orientation_q[3]
)

#CAMERA 2 CONFIG
camera2_name = "camera2"
camera2_position = (
			1,
			0,
			0
		)

camera2_orientation_q = tf.quaternion_from_euler(0, 0, 0)
camera2_orientation = (
	camera2_orientation_q[0],
	camera2_orientation_q[1],
	camera2_orientation_q[2],
	camera2_orientation_q[3]
)



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
					tag["pose"]["rotation"]["quaternion"]["W"],
					tag["pose"]["rotation"]["quaternion"]["X"],
					tag["pose"]["rotation"]["quaternion"]["Y"],
					tag["pose"]["rotation"]["quaternion"]["Z"]
				)
			}
			for tag in self.field_map["tags"]
		}

		self.camera_names = ['camera1', 'camera2']  # Add more as needed

		self.camera_transforms = {
    		"camera1": transform_from_pq(camera1_position + camera1_orientation),
    		"camera2": transform_from_pq(camera2_position + camera2_orientation)  # Modify as needed
		}

		# Tags of interest
		self.tags_of_interest = [6, 7, 8, 9, 10, 11, 12, 17, 18, 19, 20, 21, 22]
	
		# Get the root NetworkTables table and create subtables for each camera
		self.root_table = NetworkTables.getTable('AprilTags')
		self.camera_tables = {
			camera_name: self.root_table.getSubTable(camera_name)
			for camera_name in self.camera_names
		}

	def simulate_tag_detections(self):
		"""Simulate AprilTag detections for each camera with fake data and send to NetworkTables"""
		for camera_name in self.camera_names:
			# Generate a random number of detections (between 1 and 5)
			num_detections = 1 #random.randint(1, 5)
			# print(f"Simulating {num_detections} fake AprilTag detections for {camera_name}")

			detected_tags = []
			total_distance = 0.0
			num_tags = 0

			# Generate fake detections
			for _ in range(num_detections):
				tag_id = random.choice(self.tags_of_interest)
				position = (
					random.uniform(-1.0, 1.0),
					random.uniform(-1.0, 1.0),
					random.uniform(-1.0, 1.0)
				)
				orientation = (
					random.uniform(-1.0, 1.0),
					random.uniform(-1.0, 1.0),
					random.uniform(-1.0, 1.0),
					random.uniform(-1.0, 1.0)
				)

				try:
					tag_id = random.randint(1, 22)
					# tag_id = 7
					position = (
						2.0,
						0.0,
						self.tag_map[tag_id]["position"][2]
					)
					orientation = (
						0.0,
						0.0,
						0.0,
						1.0
					)

					camera_position, camera_orientation = self.calculate_camera_pose(tag_id, (position, orientation))
					robot_position, robot_orientation = self.transform_camera_to_robot((camera_position, camera_orientation), camera_name)

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

					# Post the camera's absolute pose to NetworkTables
					camera_data = list(camera_position) + list(camera_orientation)
					self.camera_tables[camera_name].putNumberArray('Absolute Pose', camera_data)
					robot_data = list(robot_position) + list(robot_orientation)
					self.camera_tables[camera_name].putNumberArray('Absolute Robot Pose', robot_data)

				except ValueError as e:
					print(str(e))

			self.camera_tables[camera_name].putNumber('Detected Tags Number', len(detected_tags))

			# Post the list of detected tags to NetworkTables
			tag_data = []
			for tag in detected_tags:
				tag_data.extend([tag['id']] + list(tag['position']) + list(tag['orientation']))
			self.camera_tables[camera_name].putNumberArray('Detected Tags Data', tag_data)

			# Calculate and post the average distance to all tags
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
		# print(tag_absolute_pose)
		tag_to_field = transform_from_pq(tag_absolute_pose["position"] + tag_absolute_pose["orientation"])
		# print(tag_to_field)

		# Get the detected pose of the tag relative to the camera
		camera_to_tag = transform_from_pq(detected_pose_relative_to_camera[0] + detected_pose_relative_to_camera[1])
		camera_to_field = np.matmul((tag_to_field), invert_transform(camera_to_tag))

		quaternion = tf.quaternion_from_matrix(camera_to_field)

		return (camera_to_field[:3, 3], (quaternion[0], quaternion[1], quaternion[2], quaternion[3]))


	def transform_camera_to_robot(self, camera_pose, camera_name):
		"""
		Transforms a camera's pose from field coordinates to robot coordinates.

		:param camera_pose: (position, orientation) tuple of the camera in field coordinates.
		:param camera_name: Name of the camera.
		:return: Camera pose relative to the robot as (position, orientation).
		"""
		if camera_name not in self.camera_transforms:
			raise ValueError(f"Camera {camera_name} not found in transformations.")

		# Transform camera pose to robot frame
		camera_position = (camera_pose[0][0], camera_pose[0][1], camera_pose[0][2])
		field_to_camera = transform_from_pq(camera_position + camera_pose[1])
		
		camera_to_robot = invert_transform(self.camera_transforms[camera_name])
		robot_pose = np.dot(field_to_camera, camera_to_robot)
		

		quaternion = tf.quaternion_from_matrix(robot_pose)
		return (robot_pose[:3, 3], (quaternion[0], quaternion[1], quaternion[2], quaternion[3]))



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