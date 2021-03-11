#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from weeding_bot.srv import nozzle_move_to_spray, nozzle_move_to_sprayResponse
from uuid import uuid4

BOX_SDF="""
<?xml version='1.0'?>
<sdf version="1.4">
<model name="killbox">
  <pose>0 0 0 0 0 0</pose>
  <static>true</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia> <!-- inertias are tricky to compute -->
          <!-- http://gazebosim.org/tutorials?tut=inertia&cat=build_robot -->
          <ixx>0.00083</ixx>       <!-- for a box: ixx = 0.083 * mass * (y*y + z*z) -->
          <ixy>0.0</ixy>         <!-- for a box: ixy = 0 -->
          <ixz>0.0</ixz>         <!-- for a box: ixz = 0 -->
          <iyy>0.00083</iyy>       <!-- for a box: iyy = 0.083 * mass * (x*x + z*z) -->
          <iyz>0.0</iyz>         <!-- for a box: iyz = 0 -->
          <izz>0.0000083</izz>       <!-- for a box: izz = 0.083 * mass * (x*x + y*y) -->
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>.05 .05 .005</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>.05 .05 .005</size>
          </box>
        </geometry>
        <material>
            <ambient>1 0 0 1</ambient>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class Sprayer:

  def __init__(self):
    self.sdf = BOX_SDF
    rospy.Service('/thorvald_001/spray_service', nozzle_move_to_spray, self.spray)
    self.spawner = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

  def spray(self, r):
    request = SpawnModelRequest()
    request.model_name = 'killbox_%s' % uuid4()
    request.model_xml = self.sdf
    request.reference_frame = 'thorvald_001/base_link'
    request.initial_pose.position.z = 0.005
    request.initial_pose.position.x = -0.45
    request.initial_pose.orientation.w = 1.0

    #this is the motion in the y axs that the nozzle has to move to spray the weeds
    #this is based on the center of the robot, or to be more precise, the base_link frame
    request.initial_pose.position.y = r.y_axis_motion

    #do the "spraying" - spawn simple object to the place of spraying
    self.spawner(request)

    #we get and set the result of the service
    result = nozzle_move_to_sprayResponse()
    result.spray_done = True

    return result


if __name__ == "__main__":
  rospy.init_node('nozzle_move_to_spray_server')
  Sprayer()
  rospy.spin()
