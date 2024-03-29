#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import color_and_position

import numpy as np
import imutils
import matplotlib.pyplot as plt
import time
from scipy.spatial import distance as dist
from matplotlib.colors import hsv_to_rgb

#Import the String message type from the /msg directory of
#the std_msgs package.
from sensor_msgs.msg import Image
from color_gradient_vision.msg import ColorAndPositionPairs, ColorAndPosition

class ImageProcessor(object):

    def __init__(self):
          self._intialized = False
          self.bridge = CvBridge()

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/image_processor"

        # Load parameters.
        #if not self.LoadParameters():
        #    rospy.logerr("%s: Error loading parameters.", self._name)
        #    return False

        # Register callbacks.

        self._sub_topic = "/cameras/left_hand_camera/image" #TODO set this to the topic where the camera publishes to
        self._pub_topic = "colors_and_position" #TODO

        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        # Set up the map.
        #self._map = np.zeros((self._x_num, self._y_num))

        self._initialized = True
        return True

    def RegisterCallbacks(self):
        # Subscriber.
        self._sensor_sub = rospy.Subscriber(self._sub_topic,
                                            Image,
                                            self.SensorCallback,
                                            queue_size=1)

        # Publisher.
        #print(self._sensor_topic)
        #print(self._vis_topic)
        self._vis_pub = rospy.Publisher(self._pub_topic,
                                        ColorAndPositionPairs,
                                        queue_size=1)

        self._debug_pub = rospy.Publisher("processed_image",
                                        Image,
                                        queue_size=1)

        return True

    # Callback to process sensor measurements.
    def SensorCallback(self, msg):
        if not self._initialized:
            rospy.logerr("%s: Was not initialized.", self._name)
            return

        # TODO
        """
        image_processing on msg

        """
        try:
          cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
          print(e)

        (rows,cols,channels) = cv_image.shape

        color_position_list, image_to_show = get_centers(cv_image)

        #TODO 2nd parameters
        msgimg = self.bridge.cv2_to_imgmsg(image_to_show, "bgr8")

        #print("msgimg")
        #print(msgimg)

        self._debug_pub.publish(msgimg)




        #cv2.imshow("Image_window", cv_image)
        #cv2.waitKey(3)
        m = ColorAndPositionPairs()
        res = []
        for l in color_position_list:
          pos, color = l
          cp_msg = ColorAndPosition()
          cp_msg.x = pos[0]
          cp_msg.y = pos[1]
          cp_msg.R = color[0]
          cp_msg.G = 0
          cp_msg.B = 0
          res.append(cp_msg)
        m.pairs = res

        '''
        m = ColorAndPositionPairs()
        a = ColorAndPosition()
        b = ColorAndPosition()
        a.x = 0
        a.y = 0
        a.R = 10
        a.G = 10
        a.B = 10
        b.x = 1
        b.y = 0
        b.R = 10
        b.G = 10
        b.B = 10
        m.pairs = [a, b] #TODO what is an array
        '''

        # Visualize.
        print("publishing")
        self._vis_pub.publish(m)


# functional helper functions

# display an image in grayscale
def im_gray(img):
    plt.figure()
    plt.imshow(img, cmap= 'gray')

# display an image in color
def im(img):
    plt.figure()
    plt.imshow(img)

# downsample colored image enough times such that the area of the image is below 1000 sq pix
# rows * cols / x^2 < area
# x^2 > rows * cols / area
# x > sqrt(rows * cols / area)
def downsample(img, area):
    (rows, cols, _) = np.shape(img)
    x = int(np.ceil(np.sqrt(rows * cols / area)))
    print('x: ' + str(x))
    if rows // x <= 1 or cols // x <= 1:
        print('downsampled to the point where we have a 1d array')
        return
    result = img[::x, ::x, :]
    print('downsampled result shape: ' + str(np.shape(result)))
    print('downsampled result area: ' + str(np.shape(result)[0] * np.shape(result)[1]))
    return result

def find_average_color(img):
    colors = {}
    (rows, cols, _) = np.shape(img)
    for row in range(rows):
        for col in range(cols):
            k = tuple(img[row, col, :])
            if k in colors:
                colors[k] += 1
            else:
                colors[k] = 1
    # do the sorting
    N = 5
    sorted_colors = sorted((value, key) for (key, value) in colors.items())
    sorted_colors.reverse()
    top_N_colors = sorted_colors[0:5]
    return (sum(np.array(key).astype('uint32') for (value, key) in top_N_colors) / N).astype('uint8')

# kills off the top part
def robot_crop(img):
    #TODO fix robot crop
    return img[:, 50:-50, :], 50

def compute_color_rgb(image, c):
    # c original dim: (2, n)
    # c target dim: (n, 1, 2)
    c = np.array([[c[:,i]] for i in range(c.shape[1])])

    # construct a mask for the contour, then compute the
    # average L*a*b* value for the masked region
    mask = np.zeros(image.shape[:2], dtype="uint8")
    cv2.drawContours(mask, [c], -1, 255, -1)
    mask = cv2.erode(mask, None, iterations=2)
    mean = cv2.mean(image, mask=mask)[:3]

    # return the name of the color with the smallest distance
    return mean #todo


def compute_color_hue(image, c):
    # c original dim: (2, n)
    # c target dim: (n, 1, 2)
    c = np.array([[c[:,i][1], c[:,i][0]] for i in range(c.shape[1])])

    # construct a mask for the contour, then compute the
    # average L*a*b* value for the masked region
    mask = np.zeros(image.shape[:2], dtype="uint8")
    cv2.drawContours(mask, [c], -1, 255, -1)
    #im_gray(mask)
    mask = cv2.erode(mask, None, iterations=2)
    # im_gray(mask)
    mean_h = cv2.mean(image, mask=mask)[0]
    mean_s = cv2.mean(image, mask=mask)[1]
    mean_v = cv2.mean(image, mask=mask)[2]
    mean = np.array([mean_h, mean_s, mean_v])

    # return the name of the color with the smallest distance
    return mean #todo


def get_centers(image):
    original_shape = image.shape
    print("original_shape", original_shape)

    # load image as <rgb> and crop if necessary
    image = cv2.cvtColor(image.astype('uint8'), cv2.COLOR_BGR2RGB)
    image_original = image
    image, yoff = robot_crop(image)

    # downsample and find average <hsv> background color
    image_filtered = cv2.bilateralFilter(image, 15, 75, 75)
    image_hsv = cv2.cvtColor(image_filtered, cv2.COLOR_RGB2HSV)
    #image_hsv_ds = downsample(image_hsv, 10000)
    #image_hsv_ds_avg = find_average_color(image_hsv_ds)

    # color threshold on <hsv>, then gaussian filter
    #image_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    diff_s = image_hsv[:,:,1] - 0 * np.ones(np.shape(image_hsv)[0:2])
    diff_v = image_hsv[:,:,2]
    # mask = np.logical_or(diff_s > 20, diff_v < 20)
    mask = diff_s > 20
    mask_int = mask.astype('uint8')
    mask_int = (mask_int / np.max(mask_int) * 255).astype('uint8')
    mask_filtered_gauss = cv2.GaussianBlur(mask_int, (7, 7), 0)
    mask_filtered_gauss = cv2.threshold(mask_filtered_gauss, 254, 256, cv2.THRESH_BINARY)[1]
    # mask_filtered_gauss = mask_int

    # morphological opening/closing to remove extra noise
    #TODO figure out the resolution problem
    kernel = np.ones((5,5), np.uint8)
    closing = cv2.morphologyEx(mask_filtered_gauss, cv2.MORPH_CLOSE, kernel, iterations = 2)
    opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel, iterations = 2)
    thresh = opening

    # finding sure background area
    sure_bg = cv2.dilate(thresh,kernel,iterations=3)

    # finding sure foreground area
    dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 0)
    ret, sure_fg = cv2.threshold(dist_transform, 0.4*dist_transform.max(), 255, 0)

    # finding unknown region
    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg, sure_fg)

    # marker labelling
    ret, markers = cv2.connectedComponents(sure_fg)
    # add one to all labels so that sure background is not 0, but 1
    markers = markers+1
    # now, mark the region of unknown with zero
    markers[unknown==255] = 0

    # run watershed algorithm
    image_copy = np.copy(image)
    # image_copy_original = np.copy(image_original)
    markers = cv2.watershed(image_copy, markers)
    # markers_original =
    image_copy[markers == -1] = [255,0,0]


    # iterate through markers and find centers
    num_blocks = np.max(markers) - 1
    print('number of blocks detected: ' + str(num_blocks))
    centers = np.zeros((num_blocks, 2)).astype('uint16')
    #fig=plt.figure(figsize=(8, 8))
    index = 0
    res = []
    for marker in range(2, 2 + num_blocks):
        # average all the pixels
        cnt = np.array(np.where(markers == marker))
        color = compute_color_hue(image_hsv, cnt)
        #print("color: {}".format(color))
        color = [int(c) for c in color]
        index += 1
        #fig.add_subplot(8, 8, index)
        #color_image = np.array([[[color[0], color[1], color[2]]]]).astype('uint8')
        #print("hsv_original: {}".format(color_image))
        #color_image = cv2.cvtColor(color_image, cv2.COLOR_HSV2RGB)
        #plt.imshow(color_image)
        #print("rgb_transformed: {}".format(color_image))
        #color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2HSV)
        #print("hsv_transformed: {}".format(color_image))
        #color_image = hsv_to_rgb(color_image)

        coords = np.mean(np.array(np.where(markers == marker)), axis = 1).astype('uint16')
        #coords[1] = original_shape[1] - coords[1]

        centers[marker - 2, :] = coords

        res.append([np.array([coords[0], coords[1] + yoff]), color])
        cv2.circle(image_copy, (coords[1], coords[0]), 2, (0, 255, 0), -1)
        # cv2.circle(image_copy_original, (coords[1] + yoff, coords[0]), 2, (0, 255, 0), -1)
    #plt.show()
    print('centers: \n' + str(centers))
    #cv2.imwrite('output.jpg', image_copy)
    return res, image_copy

    # write outputs to another topic
