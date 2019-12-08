#!/usr/bin/env python
import sys
#sys.path.append("../")
#from utils.definitions import cube
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
    from baxter_interface import Limb
else:
    from intera_interface import Limb

import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

from path_planner import PathPlanner
from controller import Controller

def main(cube_pose, end_pose):
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")

    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 3.1415*2;
    # orien_const.weight = 1.0;

    start_pose = get_start(cube_pose, end_pose)
    print("cube_pose", cube_pose)
    print("start pose", start_pose)
    #print(cube_pose)
    # print(end_pose)

    # size = np.array([0.01, 0.01, 0.01])
    # planner.add_box_obstacle(size, "cube", cube_pose)


    table_size = np.array([0.4, 1.2, 0.1])
    table_pose = PoseStamped()
    table_pose.header.frame_id = "base"

    #x, y, and z position
    table_pose.pose.position.x = 0.5

    #Orientation as a quaternion
    table_pose.pose.orientation.w = 1.0
    planner.add_box_obstacle(table_size, "table", table_pose)

    while not rospy.is_shutdown():
        while not rospy.is_shutdown():
            try:
                plan = planner.plan_to_pose(start_pose, [])
                print(plan)

                raw_input("Press <Enter> to move the right arm behind cube: ")
                result = planner.execute_plan(plan)
                if not result:
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break
        raw_input("Press <Enter> to remove cube")
        # planner.remove_obstacle("cube")
        # orien_const.absolute_z_axis_tolerance = 3.1415/4;
        while not rospy.is_shutdown():
            try:
                plan = planner.plan_to_pose(end_pose, [])

                raw_input("Press <Enter> to move the cube to target: ")
                result = planner.execute_plan(plan)
                if not result:
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break
            

def get_start(cube_pose, end_pose):
    cube = cube_pose.pose
    end = end_pose.pose

    cube_vec = np.array([cube.position.x, cube.position.y, cube.position.z])
    end_vec = np.array([end.position.x, end.position.y, end.position.z])
    delta = end_vec - cube_vec
    delta = delta / np.linalg.norm(delta)
    start_vec = cube_vec - 0.01*delta

    start_pose = PoseStamped()
    start_pose.header.frame_id = "base"

    start_pose.pose.position.x = start_vec[0]
    start_pose.pose.position.y = start_vec[1]
    start_pose.pose.position.z = cube.position.z

    theta_x = 3.14159
    theta_z = np.arctan2(delta[1], delta[0])
    start_pose.pose.orientation.y = -1
    #start_pose.orientation = quaternion_from_euler(theta_x, 0, theta_z)

    # return start_pose
    # just for testing
    return cube_pose


def get_pose(x, y, z, a, b, c, d):
    goal = PoseStamped()
    goal.header.frame_id = "base"

    #x, y, and z position
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z

    #Orientation as a quaternion
    goal.pose.orientation.x = a
    goal.pose.orientation.y = b
    goal.pose.orientation.z = c
    goal.pose.orientation.w = d
    return goal


if __name__ == '__main__':
    rospy.init_node('moveit_node')
    cube_pose = get_pose(0.8, 0.05, 0, 0, -1, 0, 0)
    end_pose= get_pose(0.6, -0.3, 0.0, 0, -1, 0, 0)
    main(cube_pose, end_pose)
