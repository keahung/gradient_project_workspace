#!/usr/bin/env python  
# import roslib
# roslib.load_manifest('learning_tf')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.03),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "manipulator",
                         "r_gripper_r_finger_tip")
        rate.sleep()