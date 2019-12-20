#!/usr/bin/env python
import sys
from baxter_interface import Limb

import rospy
import numpy as np
import traceback
from sorting_helper import *
from sorting_helper import table, default_pose

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped, Quaternion, QuaternionStamped, Vector3, Vector3Stamped

from geometry_msgs.msg import Point
from color_gradient_vision.msg import ColorAndPositionPairs, ColorAndPosition
import tf
from tf.transformations import quaternion_from_euler
import tf2_ros
import image_geometry

from sensor_msgs.msg import CameraInfo
from path_planner import PathPlanner

from ar_track_alvar_msgs.msg import AlvarMarkers


#camera_coords should be an [x, y] pair
def find_cube_coords(camera_coords, camera_model, camera_transform, listener, table_height):
	#table_height = -0.24 #TODO determine table height accurately.

	

	#print("pixel coordinates", camera_coords)

	# projection_matrix = np.array([[1007.739501953125, 0, 617.3479149530467, 0],
	# [0, 1011.606872558594, 407.8221878260792, 0],
	# [0, 0, 1, 0]])
	# projection_matrix = np.array([[404.864031743, 0.0, 630.990886129, 0.0],
	# 				[ 0.0, 404.864031743, 442.820007847, 0.0],
	# 				[0.0, 0.0, 1.0, 0.0]])


	#print("camera_transform", camera_transform)
	trans = camera_transform.transform

	q = trans.rotation
	q = np.array([q.x, q.y, q.z, q.w])
	rot = tf.transformations.quaternion_matrix(q)
	rot = rot[:3, :3]
	# print(rot)

	t = trans.translation
	trans = np.array([t.x, t.y, t.z])



	# camera_point = np.array([camera_coords[0], camera_coords[1], 1])
	# projection_matrix = projection_matrix[:, :3]
	# spacial_vec = np.linalg.solve(projection_matrix, camera_point)
	# spacial_vec /= np.linalg.norm(spacial_vec)
	#print("lin alg camera vec", spacial_vec)

	raw_x, raw_y = camera_coords[0], camera_coords[1]
	x, y = camera_model.rectifyPoint((raw_x, raw_y))
	camera_vec = camera_model.projectPixelTo3dRay((x, y))
	camera_vec3 = Vector3(camera_vec[0], camera_vec[1], camera_vec[2])
	#print("tf camera vec", camera_vec)

	header = camera_transform.header
	header.frame_id = camera_transform.child_frame_id
	stamped_camera_vec = Vector3Stamped(header, camera_vec3)

	stamped_test_vec = Vector3Stamped(header, Vector3(1, 0, 0))
	rotated_test_vec = listener.transformVector3("base", stamped_test_vec)
	#print("ros rotated", rotated_test_vec.vector)
	#print("rotation", rot)


	#print(stamped_camera_vec)
	base_vec = listener.transformVector3("base", stamped_camera_vec)



	#print("from ros", base_vec)


	#potential points will be of the form T + (Rx)t
	#base_vec_basic = np.dot(rot, spacial_vec)
	#base_vec = np.dot(rot, np.array(camera_vec))
	#print("manual", base_vec)


	#print("comparing base vec from ros transform and manual")
	#print(base_vec)
	#print(base_vec_basic)


	b = base_vec.vector
	t = (table_height - trans[2]) / b.z

	base_vec_np = np.array([b.x, b.y, b.z])

	#print(base_vec_np)

	print("camera vec length", t)

	cube_pos = t * base_vec_np + trans
	return cube_pos

def get_camera_transform(tfBuffer):
	# print("getting camera transform")
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform('base', 'reference/left_hand_camera', rospy.Time())
			return trans
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			print e
			continue

def get_height(listener):
	print("waiting for ar pose")
	ar_marker_info = rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)
	print("got ar info")
	#print(ar_marker_info.markers)
	#print(ar_marker_info.markers)
	#ar_marker_info.markers[0].header = ar_marker_info.header
	ar_marker_info.markers[0].header.frame_id = "/left_hand_camera"
	print(ar_marker_info.markers[0])
	#base_pose = listener.transformPose("base", ar_marker_info.markers[0])
	# print("got base pose")
	# print(base_pose)

	return ar_marker_info.markers[0].pose.pose.position.z

def get_ar_pose():
	print("waiting for ar pose")
	ar_marker_info = rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)
	print("got ar info")
	#print(ar_marker_info.markers)
	#ar_marker_info.markers[0].header = ar_marker_info.header
	if(len(ar_marker_info.markers) < 1):
		return None
	ar_marker_info.markers[0].header.frame_id = "/left_hand_camera"
	print(ar_marker_info.markers[0])
	#base_pose = listener.transformPose("base", ar_marker_info.markers[0])
	# print("got base pose")
	# print(base_pose)

	return ar_marker_info.markers[0].pose

def process_cubes(cubes, tfBuffer, listener, table_height):
	transform = get_camera_transform(tfBuffer)
	# print("printing cubes info")
	# print(type(cubes))
	# print(cubes)
	processed_cubes = []
	for cube in cubes:
		y, x, hue = cube.x, cube.y, cube.R
		cube_pos = find_cube_coords((x, y), camera_model, transform, listener, table_height)
		new_cube = (cube_pos[0], cube_pos[1], hue)
		processed_cubes.append(new_cube)
	# print(processed_cubes)
	return processed_cubes

def move_to_default(planner):
	#Move right arm to default pose.
	while not rospy.is_shutdown():
		try:
			plan = planner.plan_to_pose(default_pose, [])
			# print(plan)
			raw_input("Press <Enter> to move the right arm to default state: ")
			result = planner.execute_plan(plan)
			# print(result)
			if not result:
				raise Exception("Execution failed")
		except Exception as e:
			print(e)
		else:
			break

def test():
	cubes = [(0.6, -0.3, "red"), (0.5, -0.2, "blue"), (0.67, -0.25, "green")]
	cube_path = get_cube_path(cubes, color_piles)
	manipulator_path = get_manipulator_path(cube_path, default_pose)

	print(cube_path)
	print(manipulator_path)

def graph_test():
	cube = (0.6, -0.3, 80)
	cubes = [cube]
	
	manipulator_height = 0.0
	
	cube_path = get_cube_path_hue(cubes, table, manipulator_height)
	print("cube_path", cube_path)

	manipulator_path = get_manipulator_path(cube_path, default_pose)
	positions = [x.pose.position for x in manipulator_path]

	print("waypoints", positions)
	plt.plot([a.x for a in positions], [b.y for b in positions], 'ro')
	plt.plot([0.45, 0.45, 0.8, 0.8], [-0.45, -0.27, -0.45, -0.27], 'b')


if __name__ == '__main__':

	rospy.init_node('sort_cubes_node')

	tfBuffer = tf2_ros.Buffer()
	listener_a = tf2_ros.TransformListener(tfBuffer)
	listener = tf.TransformListener()

	#height = get_height(listener)
	#print("height", height)


	#If projection matrix seems incorrect, get values from
	#"/right_hand_camera/camera_info"
	print("getting camera info")
	camera_info = rospy.wait_for_message("/cameras/left_hand_camera/camera_info", CameraInfo)
	#print(camera_info)
	camera_model = image_geometry.PinholeCameraModel()
	camera_model.fromCameraInfo(camera_info)

	#print corresponding pixel coordinates for different 3d points. 
	# while(True):
	# 	raw_input("press enter to continue")
	# 	print("center pixel of camera", camera_model.project3dToPixel((0, 0, 1)))
	# 	print("upper pixel of camera", camera_model.project3dToPixel((0, 0.3, 1)))
	# 	print("side pixel of camera", camera_model.project3dToPixel((0.3, 0, 1)))

	# assert False

	table_height = -0.14 #get_height()
	print("table height", table_height)



	# while not rospy.is_shutdown():
	# 	message = rospy.wait_for_message("/colors_and_position", ColorAndPositionPairs)
	# 	cubes = message.pairs
	# 	# cubes = [cubes[0]]
	# 	# cubes[0].x = 630
	# 	# cubes[0].y = 442
	# 	print("3D cube coordinates")
	# 	cubes = process_cubes(cubes, tfBuffer, listener)

	# 	print("found cubes", cubes)

	# 	raw_input("press enter to relocate cubes")




	#listener = tf.Transformer()
	# PATH PLANNER FOR LEFT ARM
	planner_left = PathPlanner("left_arm")

	planner = PathPlanner("right_arm")
	# orien_const = OrientationConstraint()
	# orien_const.link_name = "right_gripper";
	# orien_const.header.frame_id = "base";
	# orien_const.orientation.y = -1.0;
	# orien_const.absolute_x_axis_tolerance = 0.1;
	# orien_const.absolute_y_axis_tolerance = 0.1;
	# orien_const.absolute_z_axis_tolerance = 0.1;
	# orien_const.weight = 1.0;

	


	table_size = np.array([0.6, 1.2, 0.03])
	table_pose = PoseStamped()
	table_pose.header.frame_id = "base"

	#x, y, and z position
	table_pose.pose.position.x = 0.5

	table_pose.pose.position.z = -0.27 # for baxter


	#Orientation as a quaternion
	table_pose.pose.orientation.w = 1.0
	planner.add_box_obstacle(table_size, "table", table_pose)

	# # Move right arm to four corners
	# raw_input("move left arm out of way")
	# # Move left arm out of the way
	# while not rospy.is_shutdown():
	# 	try:
	# 		plan = planner_left.plan_to_pose(vision_pose_passive, [])
	# 		# print(plan)
	# 		#raw_input("Press <Enter> to move the left arm to passive vision state (out of the way): ")
	# 		result = planner_left.execute_plan(plan)
	# 		# print(result)
	# 		if not result:
	# 			raise Exception("Execution failed")
	# 	except Exception as e:
	# 		print(e)
	# 	else:
	# 		break

	# for corner in table_corners:
	# 	raw_input("move to next corner")
	# 	while not rospy.is_shutdown():
	# 		try:
	# 			plan = planner.plan_to_pose(corner, [])
	# 			# print(plan)
	# 			#raw_input("Press <Enter> to move the left arm to passive vision state (out of the way): ")
	# 			result = planner.execute_plan(plan)
	# 			# print(result)
	# 			if not result:
	# 				raise Exception("Execution failed")
	# 		except Exception as e:
	# 			print(e)
	# 		else:
	# 			break

	while not rospy.is_shutdown():

		raw_input("press enter to start")

		move_to_default(planner)

		# Move left arm to vision pose.
		while not rospy.is_shutdown():
			try:
				plan = planner_left.plan_to_pose(vision_pose, [])
				print(plan)
				raw_input("Press <Enter> to move the left arm to vision state: ")
				result = planner_left.execute_plan(plan)
				print(result)
				if not result:
					raise Exception("Execution failed")
			except Exception as e:
				print(e)
			else:
				break

		raw_input("Press enter if camera correctly positioned")

		#Get positions of cubes.
		message = rospy.wait_for_message("/colors_and_position", ColorAndPositionPairs)
		cubes = message.pairs

		while len(cubes) <= 0:
			print("Waiting for multiple cubes")
			message = rospy.wait_for_message("/colors_and_position", ColorAndPositionPairs)
			cubes = message.pairs
			rospy.sleep(1)

		print("got cube positions", cubes)
		print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
		print("total cubes detected:", len(cubes))

		cubes = process_cubes(cubes, tfBuffer, listener, table_height)

		manipulator_height = -0.11 #get_height() + 0.04

		


		# print("Finding ar tag")
		# a = get_ar_pose()
		# while(a is None):
		# 	print("Waiting for ar tag")
		# 	a = get_ar_pose()
		# 	rospy.sleep(1)
		# a = a.pose.position
		# center_pos = np.array([a.x, a.y, manipulator_height])
		# center_pose = get_pose(center_pos, default_orientation)

		# cubes = [cubes[0]]
		# cubes[0].x = 630
		# cubes[0].y = 442
		#print("processing cubes")

		#print("processed cubes")
		# first_cube = cubes[0]
		# cube_size = np.array([0.02, 0.02, 0.02])
		# cube_pose = PoseStamped()
		# cube_pose.header.frame_id = "base"
		# cube_pose.pose.position.x = first_cube[0]
		# cube_pose.pose.position.z = -0.22 # for baxter
		# cube_pose.pose.orientation.w = 1.0
		#planner.remove_obstacle("cube 0")
		# planner.add_box_obstacle(cube_size, "cube 0", cube_pose)
		# ctr += 1

		# cubes[0] = (0.41, -0.4, 100)
		# print("first cube", cubes[0][0])
		# cubes = [cubes[0]]

		# Move left arm out of the way
		while not rospy.is_shutdown():
			try:
				plan = planner_left.plan_to_pose(vision_pose_passive, [])
				# print(plan)
				#raw_input("Press <Enter> to move the left arm to passive vision state (out of the way): ")
				result = planner_left.execute_plan(plan)
				# print(result)
				if not result:
					raise Exception("Execution failed")
			except Exception as e:
				print(e)
			else:
				break



		
		
		# center_pos[2] = 100
		# cubes = [center_pos]

		cube_path = get_cube_path_hue(cubes, table, manipulator_height)
		#print("cube_path", cube_path)

		manipulator_path = get_manipulator_path(cube_path, default_pose)
		positions = [x.pose.position for x in manipulator_path]

		#print("waypoints", positions)

		# max_cube = cubes[0]
		# max_val = -float("inf")

		# for cube in cubes:
		# 	if(cube[1] >= max_val):
		# 		max_cube = cube

		current_pose = default_pose

		for cube in cubes:
			center_pos = np.array([cube[0], cube[1], manipulator_height + 0.06])
			center_pose = get_pose(center_pos, default_orientation)
			#center_waypoints = [default_pose, center_pose]
			#move_to_default(planner)
			
			while not rospy.is_shutdown():
				try:
					plan = planner.plan_to_pose(center_pose, [])
					# print(plan)
					raw_input("Press <Enter> to move the right arm above another cube: ")
					result = planner.execute_plan(plan)
					# print(result)
					if not result:
						raise Exception("Execution failed")
				except Exception as e:
					print(e)
				else:
					break

		move_to_default(planner)

		# default_pose.pose.position.x += 0.1
		# manipulator_path[0] = default_pose
		# new_coords = default_coords + np.array([0.1, 0, 0])
		# manipulator_path[0].pose.position = Point(*new_coords)
		# manipulator_path[0].pose.orientation = Quaternion(*default_orientation)

		#Normal path
		for goal in manipulator_path:
			#print("goal", goal)
			while not rospy.is_shutdown():
				try:
					plan = planner.plan_to_pose(goal, [])

					# print(plan)
					raw_input("Press <Enter> to move cubes: ")
					result = planner.execute_plan(plan)
					# print(result)
					if not result:
						raise Exception("Execution failed")
				except Exception as e:
					print(e)
				else:
					break

		# waypoints = manipulator_path
		
		# #Cartesian path
		# while not rospy.is_shutdown():
		# 	try:
		# 		plan = planner.cartesian_plan_to_pose(waypoints)
		# 		raw_input("Press <Enter> to push cube: ")
		# 		result = planner.execute_plan(plan)
		# 		if not result:
		# 			raise Exception("Execution failed")
		# 	except Exception as e:
		# 		print(e)
		# 	else:
		# 		break
