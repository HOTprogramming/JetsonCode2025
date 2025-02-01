import rclpy
from rclpy.node import Node
from networktables import NetworkTables
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
import time
import json
import numpy as np
from pytransform3d.transformations import transform_from_pq, invert_transform, transform
import transformations as tf

'''
HOW TO RUN ON JETSON:
1. Copy this file and '2025-reefscape.json' onto jetson
2. Copy the files into the docker container
	a. 'mv apriltagROSBridge.py /mnt/nova_ssd/workspaces/isaac_ros_dev/pythonScripts/apriltagROSBridge.py'
	b. I am recalling that location from memory; the start is correct but the middle may not be. You should be able to tab out the location and find it. Good luck!
	c. Make sure you move both files to the same folder
3. Open a docker container
4. Make 'apriltagROSBridge.py' executable
	a. 'chmod +x apriltagROSBridge.py'
5. Run 'apriltagROSBridge.py'
	a. './apriltagROSBridge.py'
	b. After that failed, install the libraries with 'pip install {library}' (pynetworktables, pytransform3d, transformations, etc.)
	c. If it says you are missing 'rclpy', run it in the docker container ._.
'''

network_tables_ip = '10.0.67.2'

#This is a temporary camera configuration setup. A future implementation could be a separate config file to load in


#CAMERA 1 CONFIG
camera1_name = "camera1" #this is the name that will be published to network tables
camera1_ros_topic = "camera1" #this is where your unique namespace for this camera goes
camera1_position = ( #x, y, z in meters
			0,
			0,
			0
		)

camera1_orientation_q = tf.quaternion_from_euler(0, 0, 90) #roll, pitch , yaw

camera1_orientation = (
	camera1_orientation_q[0],
	camera1_orientation_q[1],
	camera1_orientation_q[2],
	camera1_orientation_q[3]
)

#CAMERA 2 CONFIG
camera2_name = "camera2" #this is the name that will be published to network tables
camera2_ros_topic = "camera2" #this is where your unique namespace for this camera goes
camera2_position = ( #x, y, z in meters
			0,
			0,
			0
		)

camera2_orientation_q = tf.quaternion_from_euler(0, 0, 0) #roll, pitch , yaw

camera2_orientation = (
	camera2_orientation_q[0],
	camera2_orientation_q[1],
	camera2_orientation_q[2],
	camera2_orientation_q[3]
)

#CAMERA 3 CONFIG
camera3_name = "camera3" #this is the name that will be published to network tables
camera3_ros_topic = "camera3" #this is where your unique namespace for this camera goes
camera3_position = ( #x, y, z in meters
			0,
			0,
			0
		)

camera3_orientation_q = tf.quaternion_from_euler(0, 0, 0) #roll, pitch , yaw

camera3_orientation = (
	camera3_orientation_q[0],
	camera3_orientation_q[1],
	camera3_orientation_q[2],
	camera3_orientation_q[3]
)



class ROS2NetworkTablesBridge(Node):
	def __init__(self):
		super().__init__('ros2_networktables_bridge')
		
		# Initialize NetworkTables
		self.server_ip = network_tables_ip  # Replace with your NT server IP
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

		self.camera_names = [camera1_name, camera2_name, camera3_name]  # Add more as needed

		self.camera_transforms = {
    		camera1_name: transform_from_pq(camera1_position + camera1_orientation),
    		camera2_name: transform_from_pq(camera2_position + camera2_orientation),
			camera3_name: transform_from_pq(camera3_position + camera3_orientation)  # Modify as needed
		}

		# ROS2 subscribers for each camera
		self.subscriptions = []
		for camera in self.camera_names:
			self.subscriptions.append(
				self.create_subscription(
					AprilTagDetectionArray,
					f'{camera}/tag_detections',
					self.apriltag_callback(camera),
					10
				)
			)

		#IF THE ABOVE IS SENDING ERRORS, DEBUG IT OR TRY BELOW, CANT TEST ON WINDOWS PC SORRY (:

		# self.subscription1 = self.create_subscription(
		# 	AprilTagDetectionArray,
		# 	f'{camera1_name}/tag_detections', #this should work?
		# 	self.apriltag_callback(camera1_name), #I really hope its sending self and msg here :/
		# 	10
		# )

		# self.subscription2 = self.create_subscription(
		# 	AprilTagDetectionArray,
		# 	f'{camera2_name}/tag_detections',
		# 	self.apriltag_callback(camera1_name),
		# 	10
		# )

		# self.subscription3 = self.create_subscription(
		# 	AprilTagDetectionArray,
		# 	f'{camera3_name}/tag_detections', 
		# 	self.apriltag_callback(camera1_name),
		# 	10
		# )

	def apriltag_callback(self, msg, camera_name):
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
				detection.pose.pose.orientation.w,
				detection.pose.pose.orientation.x,
				detection.pose.pose.orientation.y,
				detection.pose.pose.orientation.z
			)

			try:
				camera_position, camera_orientation = self.calculate_camera_pose(tag_id, (position, orientation))
				robot_position, robot_orientation = self.transform_camera_to_robot(self, (camera_position, camera_orientation), camera_name)

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

				# Post the robot's absolute pose to NetworkTables under the camera's subtable
				# camera_data = list(camera_position) + list(camera_orientation)
				# self.camera_tables[camera_name].putNumberArray('Absolute Pose', camera_data)

				robot_data = list(robot_position) + list(robot_orientation)
				self.camera_tables[camera_name].putNumberArray('Absolute Robot Pose', robot_data)

			except ValueError as e:
				self.get_logger().warn(str(e))

		self.camera_tables[camera_name].putNumber('Detected Tags Number', len(detected_tags))

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
		tag_absolute_pose = self.tag_map[tag_id] #Obtain tag's pose from 2025-reefscape.json map
		tag_to_field = transform_from_pq(tag_absolute_pose["position"] + tag_absolute_pose["orientation"]) #Create pose matrix of the tag's absolute pose

		# Get the detected pose of the tag relative to the camera
		camera_to_tag = transform_from_pq(detected_pose_relative_to_camera[0] + detected_pose_relative_to_camera[1]) #Create pose matrix of camera to tag

		camera_to_field = np.matmul((tag_to_field), invert_transform(camera_to_tag)) #Calculate the absolute pose of camera

		quaternion = tf.quaternion_from_matrix(camera_to_field) #Obtain rotation from pose matrix (as a quaternion)

		return (camera_to_field[:3, 3], (quaternion[0], quaternion[1], quaternion[2], quaternion[3])) #Return position (right column of matrix) and orientation (as a quaternion)

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
		camera_position = (camera_pose[0][0], camera_pose[0][1], camera_pose[0][2]) #Obtain camera position from pose (have to convert matrix to a tuple)
		field_to_camera = transform_from_pq(camera_position + camera_pose[1]) #Create pose matrix of camera's absolute pose
		
		robot_pose = np.dot(field_to_camera, invert_transform(self.camera_transforms[camera_name])) #Calculate the absolute pose of robot
		
		quaternion = tf.quaternion_from_matrix(robot_pose) #Obtain rotation from pose matrix (as a quaternion)

		return (robot_pose[:3, 3], (quaternion[0], quaternion[1], quaternion[2], quaternion[3])) #Return position (right column of matrix) and orientation (as a quaternion)
	

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