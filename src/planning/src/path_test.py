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

from path_planner import PathPlanner
from controller import Controller

def main(goals):
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")

    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;


    # if ROBOT == "sawyer":
    #     Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    #     Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    #     Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    #     Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    # else:
    #     Kp = 0.45 * np.array([0.8, 2.5, 1.7, 2.2, 2.4, 3, 4])
    #     Kd = 0.015 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    #     Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    #     Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    while not rospy.is_shutdown():
        for goal in goals:
            while not rospy.is_shutdown():
                try:
                    plan = planner.plan_to_pose(goal, [orien_const])
                    # print(plan)

                    raw_input("Press <Enter> to move the right arm to goal pose 3: ")
                    result = planner.execute_plan(plan)
                    # print(result)
                    if not result:
                        raise Exception("Execution failed")
                except Exception as e:
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

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    start = get_pose(0.8, 0.05, 0, 0, -1, 0, 0)
    middle = get_pose(0.6, -0.3, 0.0, 0, -1, 0, 0)
    end = get_pose(0.6, -0.1, 0, 0, -1, 0, 0)
    goals = [start, middle, end]
    main(goals)




    # controller = Controller(Kp, Ki, Kd, Kw, Limb("right"))

    ##
    ## Add the obstacle to the planning scene here
    ##

    # size = np.array([0.4, 1.2, 0.1])
    # pose = PoseStamped()
    # pose.header.frame_id = "base"

    # #x, y, and z position
    # pose.pose.position.x = 0.5
    # pose.pose.position.y = 0
    # pose.pose.position.z = 0

    # #Orientation as a quaternion
    # pose.pose.orientation.x = 0.0
    # pose.pose.orientation.y = 0.0
    # pose.pose.orientation.z = 0.0
    # pose.pose.orientation.w = 1.0

    # planner.add_box_obstacle(size, "table", pose)

    # size = np.array([0.4, 0.1, 1])
    # pose = PoseStamped()
    # pose.header.frame_id = "base"

    # #x, y, and z position
    # pose.pose.position.x = 0.5
    # pose.pose.position.y = -0.5
    # pose.pose.position.z = 0

    # #Orientation as a quaternion
    # pose.pose.orientation.x = 0.0
    # pose.pose.orientation.y = 0.0
    # pose.pose.orientation.z = 0.0
    # pose.pose.orientation.w = 1.0

    # planner.add_box_obstacle(size, "wall", pose)


    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper"
    # orien_const.header.frame_id = "base"
    # orien_const.orientation.y = -1.0
    # orien_const.absolute_x_axis_tolerance = 0.1
    # orien_const.absolute_y_axis_tolerance = 0.1
    # orien_const.absolute_z_axis_tolerance = 0.1
    # orien_const.weight = 1.0