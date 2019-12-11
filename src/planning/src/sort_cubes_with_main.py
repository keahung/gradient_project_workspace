#!/usr/bin/env python
import sys
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
	from baxter_interface import Limb
else:
	from intera_interface import Limb

import rospy
import numpy as np
import traceback
from sorting_helper import *
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
	print("ros rotated", rotated_test_vec.vector)
	print("rotation", rot)


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

	print(base_vec_np)

	print("camera vec length", t)

	cube_pos = t * base_vec_np + trans
	return cube_pos

def get_camera_transform(tfBuffer):
	# print("getting camera transform")
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform('base', 'left_hand_camera', rospy.Time())
			return trans
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

def get_height():
	ar_marker_info = rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)
	return ar_marker_info.pose.pose.position.z



def process_cubes(cubes, tfBuffer, listener):
	transform = get_camera_transform(tfBuffer)
	# print("printing cubes info")
	# print(type(cubes))
	# print(cubes)
	processed_cubes = []
	for cube in cubes:
		y, x, hue = cube.x, cube.y, cube.R
		table_height = get_height()
		print(table_height)
		cube_pos = find_cube_coords((x, y), camera_model, transform, listener, table_height)
		new_cube = (cube_pos[0], cube_pos[1], hue)
		processed_cubes.append(new_cube)
	# print(processed_cubes)
	return processed_cubes

def test():
	cubes = [(0.6, -0.3, "red"), (0.5, -0.2, "blue"), (0.67, -0.25, "green")]
	cube_path = get_cube_path(cubes, color_piles)
	manipulator_path = get_manipulator_path(cube_path, default_coords)

	print(cube_path)
	print(manipulator_path)


if __name__ == '__main__':

	rospy.init_node('sort_cubes_node')


	#If projection matrix seems incorrect, get values from
	#"/right_hand_camera/camera_info"
	print("getting camera info")
	camera_info = rospy.wait_for_message("/cameras/right_hand_camera/camera_info", CameraInfo)
	#print(camera_info)
	camera_model = image_geometry.PinholeCameraModel()
	camera_model.fromCameraInfo(camera_info)
	print("center pixel of camera", camera_model.project3dToPixel((0, 0, 1)))

	table_height = surface_height

	tfBuffer = tf2_ros.Buffer()
	listener_a = tf2_ros.TransformListener(tfBuffer)
	listener = tf.TransformListener()

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

	planner = PathPlanner("right_arm")
	orien_const = OrientationConstraint()
	orien_const.link_name = "right_gripper";
	orien_const.header.frame_id = "base";
	orien_const.orientation.y = -1.0;
	orien_const.absolute_x_axis_tolerance = 0.1;
	orien_const.absolute_y_axis_tolerance = 0.1;
	orien_const.absolute_z_axis_tolerance = 0.1;
	orien_const.weight = 1.0;

	# PATH PLANNER FOR LEFT ARM
	planner_left = PathPlanner("left_arm")


	# table_size = np.array([0.6, 1.2, 0.03])
	# table_pose = PoseStamped()
	# table_pose.header.frame_id = "base"

	# #x, y, and z position
	# table_pose.pose.position.x = 0.5
	# if ROBOT == "baxter":
	# 	table_pose.pose.position.z = -0.28 # for baxter
	# else:
	# 	table_pose.pose.position.z = -0.33

	# #Orientation as a quaternion
	# table_pose.pose.orientation.w = 1.0
	# planner.add_box_obstacle(table_size, "table", table_pose)

	# cubes = [(0.48, -0.46, "red"), (0.486, -0.46, "blue"), (0.486, -0.46, "green")]

	# while not rospy.is_shutdown():
	# 	raw_input("press enter to relocate cube")

	# 	message = rospy.wait_for_message("/colors_and_position", ColorAndPositionPairs)
	# 	cubes = message.pairs
	# 	# cubes = [cubes[0]]
	# 	# cubes[0].x = 630
	# 	# cubes[0].y = 442
	# 	cubes = process_cubes(cubes, tfBuffer, listener)
	# 	first_cube = cubes[0]
	# 	cube_size = np.array([0.02, 0.02, 0.02])
	# 	cube_pose = PoseStamped()
	# 	cube_pose.header.frame_id = "base"
	# 	cube_pose.pose.position.x = first_cube[0]
	# 	cube_pose.pose.position.y = first_cube[1]
	# 	cube_pose.pose.position.z = -0.22 # for baxter
	# 	cube_pose.pose.orientation.w = 1.0
	# 	planner.remove_obstacle("cube 0")
	# 	planner.add_box_obstacle(cube_size, "cube 0", cube_pose)



	while not rospy.is_shutdown():


		#ctr = 0
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

		# Move left arm to vision pose.
		while not rospy.is_shutdown():
			try:
				plan = planner_left.plan_to_pose(vision_pose, [])
				# print(plan)
				raw_input("Press <Enter> to move the left arm to vision state: ")
				result = planner_left.execute_plan(plan)
				# print(result)
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
		# cubes = [cubes[0]]
		# cubes[0].x = 630
		# cubes[0].y = 442
		cubes = process_cubes(cubes, tfBuffer, listener)
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


		#print("found cubes", cubes)


		manipulator_height = get_height() + 0.04
		print("manipulator height", manipulator_height)
		# cubes[0] = (0.41, -0.4, 100)
		# print("first cube", cubes[0])
		# cubes = [cubes[0]]
		cube_path = get_cube_path_hue(cubes, table, manipulator_height)
		print("cube_path", cube_path)

		manipulator_path = get_manipulator_path(cube_path, default_coords)
		positions = [x.pose.position for x in manipulator_path]

		print("waypoints", positions)

		# raw_input("Manipulator path found. Press enter if camera out of the way.")
		raw_input("Manipulator path found. Press <Enter> to move camera arm out of the way.")

		# Move left arm out of the way
		while not rospy.is_shutdown():
			try:
				plan = planner_left.plan_to_pose(vision_pose, [])
				# print(plan)
				raw_input("Press <Enter> to move the left arm to passive vision state (out of the way): ")
				result = planner_left.execute_plan(plan)
				# print(result)
				if not result:
					raise Exception("Execution failed")
			except Exception as e:
				print(e)
			else:
				break

		# default_pose.pose.position.x += 0.1
		# manipulator_path[0] = default_pose
		# new_coords = default_coords + np.array([0.1, 0, 0])
		# manipulator_path[0].pose.position = Point(*new_coords)
		# manipulator_path[0].pose.orientation = Quaternion(*default_orientation)
		#Normal path
		# for goal in manipulator_path:
		# 	print("goal", goal)
		# 	while not rospy.is_shutdown():
		# 		try:
		# 			plan = planner.plan_to_pose(goal, [])

		# 			# print(plan)
		# 			raw_input("Press <Enter> to move cubes: ")
		# 			result = planner.execute_plan(plan)
		# 			# print(result)
		# 			if not result:
		# 				raise Exception("Execution failed")
		# 		except Exception as e:
		# 			print(e)
		# 		else:
		# 			break

		waypoints = [default_pose].extend(manipulator_path)
		
		#Cartesian path
		while not rospy.is_shutdown():
			try:
				plan = planner.cartesian_plan_to_pose(waypoints)
				raw_input("Press <Enter> to push cube: ")
				result = planner.execute_plan(plan)
				if not result:
					raise Exception("Execution failed")
			except Exception as e:
				print(e)
			else:
				break
