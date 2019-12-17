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

if __name__ == '__main__':

	rospy.init_node('camera_testing_node')

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
	print("center pixel of camera", camera_model.project3dToPixel((0, 0, 1)))
	print("upper pixel of camera", camera_model.project3dToPixel((0, 0.3, 1)))
	print("side pixel of camera", camera_model.project3dToPixel((0.3, 0, 1)))