#!/usr/bin/env python

import rospy
import sys
import tf
import math
from collections import deque

from sensor_msgs.msg import PointCloud

from weeding_bot.srv import nozzle_move_to_spray, nozzle_move_to_sprayRequest



class Sprayer:

  def __init__(self, robot_name):
    #get the name prefix
    self.robot_name = robot_name

    self.unsprayed_queue = deque(maxlen=500)
    #self.unsprayed = []
    self.sprayed = []  #save all sprayed weeds to exclude them for any further spraying
    
    #initialise subscribers and publishers
    self.nozzle_spray_srv = rospy.ServiceProxy("{}/spray_service".format(self.robot_name), nozzle_move_to_spray)
    self.weed_cloud_sub = rospy.Subscriber("{}/weed_points".format(self.robot_name), PointCloud, self.sprayWeeds)

    #to lookup sprayer pose in map frame
    self.listener = tf.listener.TransformListener()


  #callback that sprays at all detected weeds
  def sprayWeeds(self, weed_pointcloud_msg):

    #get pose of srayer in map frame first
    spr_trans, failed_tf_lookup = self.getSprayerPoseInMap(weed_pointcloud_msg.header.stamp)


    #if we got the sprayer pose as we should we continue
    if(failed_tf_lookup!=1):
      #print("ready to spray")


      #we first filter the newly detected weeds
      #for all new detected weed places from the subscriber msg
      for weed in weed_pointcloud_msg.points:

        #we always start by saving every new weed we have found in a list
        #we will use these 
        if weed not in self.unsprayed_queue:
          self.unsprayed_queue.append(weed)
          #self.unsprayed.append(weed)


      print('Num of unsprayed: {}'.format(len(self.unsprayed_queue)))


      #for all unsprayed weeds spray the ones close to the sprayer's nozzle
      for weed in list(self.unsprayed_queue):

        #get difference in x and y axis etween sprayer and weed
        dx = abs(spr_trans[0] - weed.x)
        dy = spr_trans[1] - weed.y

        # print('current weed-point: x={}, y={}, z={}'.format(weed.x, weed.y, weed.z))
        # print('computed dy={}, dx={}'.format(dy, dx))

        #if the detected weed in inside the crop area (lane width-wise)
        if abs(dy) < 0.5:
          #if we are not far in terms of x axis - just so we don't miss
          if dx < 0.1:
            #if we haven't sprayed at this weed yet
            if weed not in self.sprayed:
              #add it to the stack of sprayed weeds
              self.sprayed.append(weed)

              #remove it from the unsprayed list
              self.unsprayed_queue.remove(weed)#remove 
              #self.unsprayed.remove(weed)

              #print('SPRAYING!!!')

              #call service to spray it
              req = nozzle_move_to_sprayRequest()
              req.y_axis_motion = dy
              #print('weed-point we spray: x={}, y={}, z={}'.format(weed.x, weed.y, weed.z))
              #print('spraying at dy={}, dx={}'.format(dy, dx))
              self.nozzle_spray_srv(req)



  #callback to get pose of sprayer in map frame
  #distance between sprayer and kinect2_link is 0.9m
  def getSprayerPoseInMap(self, msg_stamp):
    try:
      #we specify the time of the transform we want
      spr_trans, spr_rot = self.listener.lookupTransform('map', '{}/sprayer'.format(self.robot_name), msg_stamp)
      failed_tf_lookup = 0 #all good
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print("Can't find transform between /map and /sprayer links!")
      failed_tf_lookup = 1 #we failed to get the pose

    return spr_trans, failed_tf_lookup




def main(args):
  #initialize node
  rospy.init_node('sprayer_node', anonymous=True)

  #get arguments and instantiate object of class
  robot_name = sys.argv[1].decode('string-escape')
  Sprayer(robot_name)

  #start spinning
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
    
        
if __name__ == '__main__':
  main(sys.argv)
