#!/usr/bin/env python

import tf2_ros
import sys
import rospy


#Define the method which contains the main functionality of the node.
def controller(target_frame, source_frame):
  """
  Controls a turtlebot whose position is denoted by target_frame,
  to go to a position denoted by target_frame
  Inputs:
  - target_frame: the tf frame of the AR tag on your turtlebot
  - target_frame: the tf frame of the target AR tag
  """

  ################################### YOUR CODE HERE ##############

  #Create a publisher and a tf buffer, which is primed with a tf listener
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(2) # 10hz


  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
      print(trans)

      #################################### end your code ###############
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass
    # Use our rate object to sleep until it is time to publish again
    r.sleep()


if __name__ == '__main__':
  rospy.init_node('tf_echo_listener', anonymous=True)
  try:
	controller(sys.argv[1], sys.argv[2])
  except rospy.ROSInterruptException:
	pass
