#!/usr/bin/env python

import rospy
import sys
import numpy as np
import tf
import tf2_ros

import cv2
from cv_bridge import CvBridge
import image_geometry

from geometry_msgs.msg import Point32
from std_msgs.msg import Int16

#from pylab import *

from sensor_msgs.msg import Image, CameraInfo, PointCloud


class WeedDetector:

  def __init__(self, robot_name):
    #get the name prefix
    self.robot_name = robot_name

    self.bridge = CvBridge()

    #used to transform pointcloud in map frame
    self.tflistener = tf.listener.TransformListener()

    #set our camera model
    self.camera_model = image_geometry.PinholeCameraModel()

    #initialise subscribers and publishers
    self.image_sub = rospy.Subscriber("{}/kinect2_camera/hd/image_color_rect".format(self.robot_name), Image, self.image_callback)
    self.crop_row_id = 0 #initialise crop row we are currently at
    self.crop_row_sub = rospy.Subscriber("/crop_row", Int16, self.cropRowCallback)

    self.weed_pointcloud_msg = PointCloud() #message to publish for weed detections
    self.weed_cloud_pub = rospy.Publisher("{}/weed_points".format(self.robot_name), PointCloud, queue_size=10)

    #get camera info once
    self.camera_info_data = rospy.wait_for_message('/{}/kinect2_camera/hd/camera_info'.format(self.robot_name), CameraInfo)
    self.camera_model.fromCameraInfo(self.camera_info_data)



  #get image data and detect weeds
  def image_callback(self, data):

    #in case something went wrong with setting the camera model we have to stop here and return
    if not self.camera_model:
        return

    #convert ROS image msg into an cv::Mat with defined encoding
    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    #show the img
    #self.show_img(cv_image)

    #convert img colors to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    #we are blurring to make the result more "smooth"
    #blurred_img = cv2.blur(hsv, (35, 35))
    blurred_img = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)

    #get upper and lower bound of weeds considering in which lane we are at
    lower_bound, upper_bound = self.weed_filterer()

    #check to find weeds based on the computed HSV bounds
    cv_image_copy = cv_image
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    res = cv2.bitwise_and(cv_image, cv_image, mask= mask)

    #find the contours in image
    image_with_detections, contours_filtered, rectList, rect_centers = self.find_contours(res, cv_image_copy)

    #show the img
    self.show_img(image_with_detections)

    self.publishWeedPointCloud(rect_centers)


  #get current crop number we are at
  def cropRowCallback(self, data):
    self.crop_row_id = data.data



  #return appripriate upper and lower bound for weed based on the lane in the crops that we are at
  def weed_filterer(self):

    #weeds for lettuce1 and lettuce2 can be detected with the same filter bounds
    if self.crop_row_id == 0 or self.crop_row_id == 1 or self.crop_row_id == 2 or self.crop_row_id == 3:
      lower_bound = np.array([30, 30, 0])
      upper_bound = np.array([100, 90, 85])
    #weeds for onion rows
    elif self.crop_row_id == 4 or self.crop_row_id == 5:
      lower_bound = np.array([40, 90, 0])
      upper_bound = np.array([100, 100, 200])
    else:
      lower_bound = np.array([30, 30, 0])
      upper_bound = np.array([100, 90, 85])      


    #lettuce1
    # lower_bound = np.array([30, 120, 0])
    # upper_bound = np.array([50, 180, 200])

    #onions. works very well for lettuce2 weeds and good for lettuce1 weeds
    # lower_bound = np.array([40, 40, 0])
    # upper_bound = np.array([100, 80, 200])

    #ground
    # lower_bound = np.array([0, 30, 30])
    # upper_bound = np.array([20, 140, 80])

    #everything that is not ground. anything that is plant (both lettuces, onions and weeds)
    # lower_bound = np.array([30, 0, 10])
    # upper_bound = np.array([90, 255, 150])


    return lower_bound, upper_bound



  #find the contours in image
  def find_contours(self, res, image):

    #convert image to grayscale and find contours in it
    #Tutorial used: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contours_begin/py_contours_begin.html#contours-getting-started
    imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 21, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #filter out the contrours that have a smaller area than our predefined threshold
    area_thresh = 350
    contours_filtered = []
    #do the filtering for each contour we found
    for contour in contours:
      cont_area = cv2.contourArea(contour)
      #if the current contour has a bigger area than the threshold, keep it
      if cont_area >= area_thresh:
        contours_filtered.append(contour)


    #draw the filtered contours we found
    cv2.drawContours(image, contours_filtered, -1, (255, 0, 0), -1)

    #get the rectangular boxes from the filtered contours we found
    image_with_detections, rectList, rect_centers = self.compute_rectangs(contours_filtered, image)


    return image_with_detections, contours_filtered, rectList, rect_centers



  #get the rectangular boxes from the filtered contours we found
  def compute_rectangs(self, contours_filtered, image):

    detected_rectangs = []
    rect_centers = []

    #from filtered contours detect rectangulars
    #Tutorial used: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_properties/py_contour_properties.html#contour-properties
    for contour in contours_filtered:
      x, y, w, h = cv2.boundingRect(contour)
      cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
      detected_rectangs.append([x, y, w, h])
      detected_rectangs.append([x, y, w, h])#neds to be done twice


    # print('Num of detected rectangles: {}'.format(len(detected_rectangs)))

    # #combine rectangles with similar sizes and similar locations
    rectList = cv2.groupRectangles(detected_rectangs, groupThreshold=1, eps=0.8)

    # print(rectList[0])
    # print(rectList[1])
    # print('Num of rects: {}'.format(len(rectList[0])))

    #compute and draw grouped rectangles
    for rectangle in rectList[0]:
      x, y, w, h = rectangle
      center_x = x + w/2
      center_y = y + h/2
      center_point = (center_x, center_y)
      rect_centers.append(center_point)
      #cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 3)
      cv2.circle(image, center_point, 12, (0, 0, 255), 3)


    #if groupRectangles, i should return rectList instead of detected_rectangs
    return image, detected_rectangs, rect_centers


  #publish pointcloud of weed positions
  def publishWeedPointCloud(self, rect_centers):
    
    #header part of message
    self.weed_pointcloud_msg.points = []
    self.weed_pointcloud_msg.header.frame_id = self.camera_model.tfFrame()
    self.weed_pointcloud_msg.header.stamp = rospy.Time()

    #rectify and project points in pointcloud based on camera model
    for w_center in rect_centers:
      #rectify and project in pixel coordinates
      #Tutorial used: http://docs.ros.org/en/kinetic/api/image_geometry/html/python/index.html
      rectified_point = self.camera_model.rectifyPoint(w_center)
      projected_point = self.camera_model.projectPixelTo3dRay(rectified_point)

      self.weed_pointcloud_msg.points.append(Point32(projected_point[0]*0.493, projected_point[1]*0.493, 0.493))


    #transform pointcloud to global map frame
    weed_pointcloud_in_map_msg = self.tflistener.transformPointCloud('map', self.weed_pointcloud_msg)

    #publish pointcloud
    self.weed_cloud_pub.publish(weed_pointcloud_in_map_msg)

    print('Num of points published: {}'.format(len(weed_pointcloud_in_map_msg.points)))


  #show image data
  def show_img(self, cv_image):

    resized_img = cv2.resize(cv_image, (640, 480)) 
    cv2.imshow("Image window", resized_img)
    cv2.waitKey(1)





def main(args):
  #initialize node
  rospy.init_node('weed_detector_node', anonymous=True)

  #get arguments and instantiate object of class
  robot_name = sys.argv[1].decode('string-escape')
  WeedDetector(robot_name)

  #start spinning
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
    
        
if __name__ == '__main__':
  main(sys.argv)
