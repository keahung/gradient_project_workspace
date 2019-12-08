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
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Point
from color_gradient_vision.msg import ColorAndPositionPairs, ColorAndPosition 

from tf.transformations import quaternion_from_euler
import tf2_ros
from scipy.spatial.transform import Rotation as R

from path_planner import PathPlanner
from controller import Controller


#If projection matrix seems incorrect, get values from
#"/right_hand_camera/camera_info"

projection_matrix = np.array([[1007.739501953125, 0, 617.3479149530467, 0], 
	[0, 1011.606872558594, 407.8221878260792, 0], 
	[0, 0, 1, 0]])

#camera_coords should be an [x, y] pair
def find_cube_coords(camera_coords):
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			trans = fBuffer.lookup_transform('/camera_frame', '/base', rospy.Time()).transform
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	q = trans.rotation
	q = np.array([q.w, q.x, q.y. q.z])
	rot = R.from_quat(q)

	#rot = np.array(tf2_ros.Matrix3x3(trans.rotation))
	t = trans.translation
	trans = np.array([t.x, t.y, t.z])

	camera_point = np.array([camera_coords[0], camera_coords[1], 1])
	
	projection_matrix = projection_matrix[:, :3]
	spacial_vec = np.linalg.solve(projection_matrix, camera_point)
	#potential points will be of the form T + (Rx)t
	base_frame_vec = rot @ spacial_vec

	t = (table_height - trans[2]) / base_frame_vec[2]

	cube_pos = t * base_frame_vec + trans
	return cube_pos

def gen_update_cube_positions(cubes_container):
	def update_cube_positions(new_cubes):
		cubes_container[0] = new_cubes
	return update_cube_positions


rospy.init_node('sort_cubes_node')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
table_height = -0.26

cubes_container = []
update_cube_positions = gen_update_cube_positions(cubes_container)
rospy.Subscriber("colors_and_position", ColorAndPositionPairs, update_cube_positions)


planner = PathPlanner("right_arm")
orien_const = OrientationConstraint()
orien_const.link_name = "right_gripper";
orien_const.header.frame_id = "base";
orien_const.orientation.y = -1.0;
orien_const.absolute_x_axis_tolerance = 0.1;
orien_const.absolute_y_axis_tolerance = 0.1;
orien_const.absolute_z_axis_tolerance = 0.1;
orien_const.weight = 1.0;

table_size = np.array([0.6, 1.2, 0.03])
table_pose = PoseStamped()
table_pose.header.frame_id = "base"

#x, y, and z position
table_pose.pose.position.x = 0.5
if ROBOT == "baxter":
	table_pose.pose.position.z = -0.23 # for baxter
else:
	table_pose.pose.position.z = -0.33

#Orientation as a quaternion
table_pose.pose.orientation.w = 1.0
planner.add_box_obstacle(table_size, "table", table_pose)

cubes = [(0.48, -0.46, "red"), (0.486, -0.46, "blue"), (0.486, -0.46, "green")]


while not rospy.is_shutdown():
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

	#TODO 
	# 
	cubes = cubes_container[0]

	cube_path = get_cube_path(cubes, color_piles)
	manipulator_path = get_manipulator_path(cube_path, default_coords)


	# default_pose.pose.position.x += 0.1
	# manipulator_path[0] = default_pose
	# new_coords = default_coords + np.array([0.1, 0, 0])
	# manipulator_path[0].pose.position = Point(*new_coords)
	# manipulator_path[0].pose.orientation = Quaternion(*default_orientation)
	# #Normal path
	# for goal in manipulator_path:
	# 	print("goal", goal)
	# 	while not rospy.is_shutdown():
	# 		try:
	# 			plan = planner.plan_to_pose(goal, [])
	# 			# print(plan)
	# 			raw_input("Press <Enter> to move the right arm to goal pose 3: ")
	# 			result = planner.execute_plan(plan)
	# 			# print(result)
	# 			if not result:
	# 				raise Exception("Execution failed")
	# 		except Exception as e:
	# 			print(e)
	# 		else:
	# 			break

	waypoints = manipulator_path
	#Cartesian path
	while not rospy.is_shutdown():
		try:
			plan = planner.cartesian_plan_to_pose(waypoints)
			raw_input("Press <Enter> to move the right arm to goal pose 3: ")
			result = planner.execute_plan(plan)
			if not result:
				raise Exception("Execution failed")
		except Exception as e:
			print(e)
		else:
			break



def test():
	cubes = [(0.6, -0.3, "red"), (0.5, -0.2, "blue"), (0.67, -0.25, "green")]
	cube_path = get_cube_path(cubes, color_piles)
	manipulator_path = get_manipulator_path(cube_path, default_coords)

	print(cube_path)
	print(manipulator_path)
