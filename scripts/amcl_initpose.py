#!/usr/bin/env python

import rospy
import sys
import tf
import math
import numpy as np

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseWithCovarianceStamped


class AMCLInitPose:

  def __init__(self):

    #publish amcl initial pose
    self.amcl_initpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10, latch = "true")

    #set init pose and publish it
    init_pose = self.setAMCLInitPose()
    self.amcl_initpose_pub.publish(init_pose)



  #publish the amcl initial pose
  def setAMCLInitPose(self):
    init_pose = PoseWithCovarianceStamped()
    init_pose.header.stamp = rospy.Time()
    init_pose.header.frame_id = "map"

    #position
    init_pose.pose.pose.position.x = 5
    init_pose.pose.pose.position.y = 0
    init_pose.pose.pose.position.z = 0

    #quaternion
    init_pose.pose.pose.orientation.x = 0
    init_pose.pose.pose.orientation.y = 0
    init_pose.pose.pose.orientation.z = 0   
    init_pose.pose.pose.orientation.w = 1

    #covariance matrix
    init_pose.pose.covariance = np.zeros(36)

    return init_pose




def main(args):
  #initialize node
  rospy.init_node('amcl_initpose_node', anonymous=True)

  #create object of class
  AMCLInitPose()


  #start spinning
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass

        
if __name__ == '__main__':
  main(sys.argv)


