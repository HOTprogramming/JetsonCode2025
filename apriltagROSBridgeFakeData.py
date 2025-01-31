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

		# Tags of interest
		self.tags_of_interest = [6, 7, 8, 9, 10, 11, 12, 17, 18, 19, 20, 21, 22]

		# Define camera configurations
		self.camera_names = ['camera1', 'camera2']  # Add more as needed

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
					tag_id = random.randint(0, 22)
					position = (
						2.0,
						0.0,
						0.2
					)
					orientation = (
						0.0,
						0.0,
						0.0,
						1.0
					)

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

					# Post the camera's absolute pose to NetworkTables
					camera_data = list(camera_position) + list(camera_orientation)
					self.camera_tables[camera_name].putNumberArray('Absolute Pose', camera_data)

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
		# camera_to_field = np.matmul(camera_to_tag, invert_transform(tag_to_field))
		camera_to_field = np.dot((tag_to_field), invert_transform(camera_to_tag))
		# camera_to_field = np.matmul(invert_transform(tag_to_field), camera_to_tag)

		camera_orientation = camera_to_field[:3, :3]
		quaternion = tf.quaternion_from_matrix(camera_to_field)

		qw = quaternion[0]
		qx = quaternion[1]
		qy = quaternion[2]
		qz = quaternion[3]


		# trace = np.trace(camera_orientation)
		# if trace > 0:
		# 	qw = np.sqrt(1.0 + trace) / 2.0
		# 	qx = (camera_orientation[2, 1] - camera_orientation[1, 2]) / (4.0 * qw)
		# 	qy = (camera_orientation[0, 2] - camera_orientation[2, 0]) / (4.0 * qw)
		# 	qz = (camera_orientation[1, 0] - camera_orientation[0, 1]) / (4.0 * qw)
		# else:
		# 	if camera_orientation[0, 0] > camera_orientation[1, 1] and camera_orientation[0, 0] > camera_orientation[2, 2]:
		# 		s = 2.0 * np.sqrt(1.0 + camera_orientation[0, 0] - camera_orientation[1, 1] - camera_orientation[2, 2])
		# 		qw = (camera_orientation[2, 1] - camera_orientation[1, 2]) / s
		# 		qx = 0.25 * s
		# 		qy = (camera_orientation[0, 1] + camera_orientation[1, 0]) / s
		# 		qz = (camera_orientation[0, 2] + camera_orientation[2, 0]) / s
		# 	elif camera_orientation[1, 1] > camera_orientation[2, 2]:
		# 		s = 2.0 * np.sqrt(1.0 + camera_orientation[1, 1] - camera_orientation[0, 0] - camera_orientation[2, 2])
		# 		qw = (camera_orientation[0, 2] - camera_orientation[2, 0]) / s
		# 		qx = (camera_orientation[0, 1] + camera_orientation[1, 0]) / s
		# 		qy = 0.25 * s
		# 		qz = (camera_orientation[1, 2] + camera_orientation[2, 1]) / s
		# 	else:
		# 		s = 2.0 * np.sqrt(1.0 + camera_orientation[2, 2] - camera_orientation[0, 0] - camera_orientation[1, 1])
		# 		qw = (camera_orientation[1, 0] - camera_orientation[0, 1]) / s
		# 		qx = (camera_orientation[0, 2] + camera_orientation[2, 0]) / s
		# 		qy = (camera_orientation[1, 2] + camera_orientation[2, 1]) / s
		# 		qz = 0.25 * s

		return (camera_to_field[:3, 3], (qw, qx, qy, qz))

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