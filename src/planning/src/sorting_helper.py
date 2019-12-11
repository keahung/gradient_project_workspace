import numpy as np
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Point

from tf.transformations import quaternion_from_euler



#cubes is a list of tuples (x, y, color)
#color_piles is a dictionary with entries color: pile coordinates
def pick_target_cube(cubes, color_piles):
	min_cube = None
	min_y = float("inf")
	for cube in cubes:
		if(cube[1] < min_y):
			pile_coords = color_piles[cube[2]]
			#Update min_cube if cube isn't already in target location
			if((cube[0]-pile_coords[0])**2 + (cube[1]-pile_coords[1])**2 > 0.1**2)      :
				min_cube = cube
				min_y = cube[1]
	return min_cube

def pick_target_position(cube, cubes, color_piles, surface_height):
	#For now, always send cubes to the center of the pile they should be in.
	return color_piles[cube[2]] + (surface_height,)


#Assume cube is a tuple, (x, y, hue).
def pick_target_position_hue(cube, table, surface_height):
	min_x = table[0] + 0.05
	max_x = table[1] - 0.05
	print(cube)
	hue = cube[2]
	x_pos = min_x + (hue/360)*(max_x - min_x)
	y_pos = 0.75*table[2] + 0.25*table[3]
	return np.array([x_pos, y_pos, surface_height])

def pick_target_cube_hue(cubes, table, surface_height):
	min_cube = None
	min_y = float("inf")
	for cube in cubes:
		if(cube[1] > min_y):
			pile_coords = pick_target_position_hue(cube, table, surface_height)
			#Update min_cube if cube isn't already in target location
			if((cube[0]-pile_coords[0])**2 + (cube[1]-pile_coords[1])**2 > 0.07**2):
				min_cube = cube
				min_y = cube[1]
	return min_cube


def bounding_rectangle(cubes):
	x_vals = [cube[0] for cube in cubes]
	y_vals = [cube[1] for cube in cubes]
	return min(x_vals), max(x_vals), min(y_vals), max(y_vals)

#Place color piles on lower half of table
def get_color_piles(table, colors):
	num_colors = len(colors)
	color_piles = {}
	min_x = table[0]
	max_x = table[1]
	y_level = 0.75*table[2] + 0.25*table[3]
	spacing = (max_x - min_x) / (len(colors) + 1)
	for i in range(len(colors)):
		color_piles[colors[i]] = (spacing*(i+1) + min_x, y_level)
	return color_piles

#Returns the list of positions the target cube should be in as it travels to target.
def get_cube_path(cubes, color_piles):
	cube = pick_target_cube(cubes, color_piles)
	cube_pos = np.array((cube[0], cube[1], surface_height))
	target_pos = np.array(pick_target_position(cube, cubes, color_piles, surface_height))
	print(cube_pos, target_pos)
	return [cube_pos, target_pos]

#Returns the list of positions the target cube should be in as it travels to target.
def get_cube_path_hue(cubes, table, surface_height):
	cube = pick_target_cube_hue(cubes, table, surface_height)
	cube_pos = np.array((cube[0], cube[1], surface_height))
	target_pos = np.array(pick_target_position_hue(cube, table, surface_height))
	print(cube_pos, target_pos)
	return [cube_pos, target_pos]


#Returns the list of poses the manipulator should go into as it pushes a cube along its path.
def get_manipulator_path(cube_path, start_pos):
	manipulator_path = []
	current_pos = start_pos
	for i in range(len(cube_path) - 1):
		cube_pos = cube_path[i]

		target_pos = cube_path[i+1]
		ready_pos = get_start_coordinates(cube_pos, target_pos)
		offset_pos = get_offset_point(cube_pos, ready_pos, current_pos)

		#current_pos -> offset_pos -> ready_pos -> target_pos
		offset_orien = get_push_orientation(offset_pos, ready_pos)
		offset_state = get_pose(offset_pos, offset_orien)

		ready_orien = get_push_orientation(ready_pos, target_pos)
		ready_state = get_pose(ready_pos, ready_orien)

		target_state = get_pose(target_pos, ready_orien)

		manipulator_path.extend([offset_state, ready_state, target_state])

		#------------------------------
		current_pos = target_pos

	return manipulator_path


def get_start_coordinates(cube_pos, target_pos):
	delta = target_pos - cube_pos
	delta = delta / np.linalg.norm(delta)
	ready_pos = cube_pos - 0.02*delta
	return ready_pos

def get_offset_point(cube_pos, ready_pos, current_pos):
	midpoint = (cube_pos + ready_pos) / 2
	delta = ready_pos - cube_pos
	rot_vec = 8*np.array([-delta[1], delta[0], 0])
	offset_pos_1 = midpoint + rot_vec
	offset_pos_2 = midpoint - rot_vec
	dist_1 = np.linalg.norm(current_pos - offset_pos_1)
	dist_2 = np.linalg.norm(current_pos - offset_pos_2)
	if(dist_1 < dist_2):
		return offset_pos_1
	else:
		return offset_pos_2

def get_push_orientation(cube_pos, target_pos):
	delta = target_pos - cube_pos
	theta_x = 3.1416
	theta_z = (float) (3.14/4 + np.arctan2(delta[1], delta[0]))
	#start_pose.pose.orientation.y = -1
	orientation = quaternion_from_euler(theta_x, 0, theta_z)
	return orientation

def get_pose(coords, quaternion):
	goal = PoseStamped()
	goal.header.frame_id = "base"

	goal.pose.position = Point(*coords)
	goal.pose.orientation = Quaternion(*quaternion)
	return goal

table = [0.45, 0.8, -0.45, -0.27]
colors = ["red", "blue", "green"]
color_piles = get_color_piles(table, colors)
#print("color piles", color_piles)

surface_height = -0.2
default_coords = np.array([0.6, -0.5, -0.15])
default_orientation = np.array([0.0, -1.0, 0.0, 0.0])
default_pose = get_pose(default_coords, default_orientation)

# Vision pose for the left arm
# TODO: determine these coordinates
vision_coords = np.array([])
vision_orientation = np.array([0.0, -1.0, 0.0, 0.0])
vision_pose = get_pose(vision_coords, vision_orientation)

# Passive pose for the left arm (moves it out of th way of the right arm)
# TODO: determine these coordinates
vision_coords_passive = np.array([])
vision_orientation_passive = np.array([0.0, -1.0, 0.0, 0.0])
vision_pose_passive = get_pose(vision_coords_passive, vision_orientation_passive)
