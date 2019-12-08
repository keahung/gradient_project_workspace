#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
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

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import quaternion_from_euler

from path_planner import PathPlanner
from controller import Controller


def main(cube_pose, end_pose):

    planner = PathPlanner("right_arm")
    

    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 10000;
    orien_const.weight = 1.0;

    # allow only 1/2 pi rotation so as to not lose the cube during transit
    orien_path_const = OrientationConstraint()
    orien_path_const.link_name = "right_gripper";
    orien_path_const.header.frame_id = "base";
    orien_path_const.orientation.y = -1.0;
    orien_path_const.absolute_x_axis_tolerance = 0.1;
    orien_path_const.absolute_y_axis_tolerance = 0.1;
    orien_path_const.absolute_z_axis_tolerance = 1.57;
    orien_path_const.weight = 1.0;

    # quat = QuaternionStamped()
    # quat.header.frame_id = "base"
    # quat.quaternion.w = 1.0

    start_pose = get_start(cube_pose, end_pose)
    end_pose.pose.orientation = start_pose.pose.orientation

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

    # cube_size = np.array([0.05, 0.05, 0.15])

    default_state = get_pose(0.6, -0.5, -0.17, 0.0, -1.0, 0.0, 0.0)
    # default_state = get_pose(0.94, 0.2, 0.5, 0.0, -1.0, 0.0, 0.0)

    while not rospy.is_shutdown():
        while not rospy.is_shutdown():
                try:
                    plan = planner.plan_to_pose(default_state, [])
                    print(type(plan))
                    raw_input("Press <Enter> to move the right arm to default pose: ")
                    if not planner.execute_plan(plan):
                        raise Exception("(ours) Execution failed")
                except Exception as e:
                    print("caught!")
                    print e
                else:
                    break

        # planner.add_box_obstacle(cube_size, "cube", cube_pose)

        while not rospy.is_shutdown():
            try:
                plan = planner.plan_to_pose(start_pose, [orien_const])
                raw_input("Press <Enter> to move the right arm to cube: ")
                if not planner.execute_plan(plan):
                    raise Exception("(ours) Execution failed")
            except Exception as e:
                print("caught!")
                print e
            else:
                break

        orien_path_const.orientation = start_pose.pose.orientation
        # planner.remove_obstacle("cube")

        while not rospy.is_shutdown():
            try:
                plan = planner.plan_to_pose(end_pose, [orien_path_const])
                raw_input("Press <Enter> to move the right arm to end position: ")
                if not planner.execute_plan(plan):
                    raise Exception("(ours) Execution failed")
            except Exception as e:
                print("caught!")
                print e
            else:
                break


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

def get_start(cube_pose, end_pose):
    cube = cube_pose.pose
    end = end_pose.pose
    cube_vec = np.array([cube.position.x, cube.position.y, cube.position.z])
    end_vec = np.array([end.position.x, end.position.y, end.position.z])
    delta = end_vec - cube_vec
    delta = delta / np.linalg.norm(delta)
    start_vec = cube_vec - 0.2*delta

    start_pose = PoseStamped()
    start_pose.header.frame_id = "base"

    start_pose.pose.position.x = (float) (start_vec[0])
    start_pose.pose.position.y = (float) (start_vec[1])
    start_pose.pose.position.z = (float) (cube.position.z)

    theta_x = 3.1416
    theta_z = (float) (-3.14/4 - np.arctan2(delta[1], delta[0]))
    #start_pose.pose.orientation.y = -1
    start_pose.pose.orientation = Quaternion(*quaternion_from_euler(theta_x, 0, theta_z))
    return start_pose

# not in use, was for debugging purposes earlier
def truncate(val):
    return (float)('%.3f'%(val))
    

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    # baxter
    if ROBOT == "baxter":
        end_pose = get_pose(0.6, -0.7, -0.19, 0.0, -1.0, 0.0, 0.0)
        cube_pose = get_pose(0.6, -0.3, -0.19, 0.0, -1.0, 0.0, 0.0)
    else:
        # sawyer
        cube_pose = get_pose(0.6, -0.1, -0.19, 0.0, -1.0, 0.0, 0.0)
        end_pose = get_pose(0.9, -0.1, -0.19, 0.0, -1.0, 0.0, 0.0)
        # cube_pose = get_pose(0.6, -0.17, -0.19, 0.0, -1.0, 0.0, 0.0)
        # end_pose = get_pose(0.9, -0.04, -0.19, 0.0, -1.0, 0.0, 0.0)
    # end = get_pose(0.6, -0.1, 0.1, 0.0, -1.0, 0.0, 0.0)
    # goals = [start, middle, end]
    # main(cube_pose, end_pose)
    
    main(cube_pose, end_pose)

