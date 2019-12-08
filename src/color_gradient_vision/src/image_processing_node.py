#!/usr/bin/python
################################################################################
#
# Node to wrap the OccupancyGrid2d class.
#
################################################################################

from image_processor import ImageProcessor

import rospy
import sys

if __name__ == "__main__":
    rospy.init_node('image_processor', anonymous=True)

    IP = ImageProcessor()
    if not IP.Initialize():
        rospy.logerr("Failed to initialize the image processing node.")
        sys.exit(1)

    rospy.spin()
