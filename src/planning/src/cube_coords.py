import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Point
import tf2_ros
from scipy.spatial.transform import Rotation as R

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

table_height = -0.26

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






