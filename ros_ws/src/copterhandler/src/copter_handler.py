#!/usr/bin/env python
from copy import copy
from time import sleep

import sys
import traceback
import rospy
import math
import tf
import numpy as np
from visualization_msgs.msg import Marker
from AbstractVirtualCapability import VirtualCapabilityServer

from VirtualCopter import VirtualCopter


class CopterHandler:
    def __init__(self):
        self.position = np.array([0, 0, 2.])
        self.rotation = [0, 0, 0, 1]
        self.scale = .2

        self.max_vel = .01
        self.acc = 0.0000001
        self.pub = rospy.Publisher("/robot", Marker, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.name = "copter"

    def get_position(self):
        return list(self.position)

    def get_tf_name(self):
        return self.name

    def set_name(self, name):
        self.name = name

    def set_pos(self, goal: list):

        rospy.logwarn(f"Flying to Position: {goal}")

        self.br.sendTransform((goal[0], goal[1], goal[2]),
                              tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "goal", "world")


        while True:
            goal = np.array(goal)
            vector = goal - self.position

            if np.linalg.norm(vector) < 0.1:
                self.position = goal
                self.publish_visual()
                break

            current_vel = self.max_vel * vector / np.linalg.norm(vector)
            self.position += current_vel


            self.publish_visual()
            sleep((abs(current_vel[0]) + abs(current_vel[1]) + abs(current_vel[2])))



    def publish_visual(self):
        #rospy.logwarn(f"Publishing {self.position}")
        marker = Marker()
        marker.id = int(rospy.get_param('~semantix_port'))
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"place_copter"
        marker.lifetime = rospy.Duration(0)
        # marker.color.r = .1
        # marker.color.g = .15
        # marker.color.b = .3
        marker.mesh_use_embedded_materials = True
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.x = self.rotation[0]
        marker.pose.orientation.y = self.rotation[1]
        marker.pose.orientation.z = self.rotation[2]
        marker.pose.orientation.w = self.rotation[3]
        # Scale down
        marker.scale.x = self.scale
        marker.scale.y = self.scale
        marker.scale.z = self.scale
        marker.color.a = 1
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = r"package://copterhandler/meshes/copter.dae"

        self.pub.publish(marker)

        # TF
        self.br.sendTransform(self.position,
                              tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), self.name, "world")


if __name__ == '__main__':
    rospy.init_node('rosnode')
    rate = rospy.Rate(10)

    rospy.logwarn("Starting")

    server = VirtualCapabilityServer(int(rospy.get_param('~semantix_port')))

    copter = VirtualCopter(server)
    robot = CopterHandler()

    rospy.logwarn("Setting functionality")

    copter.funtionality["set_pos"] = robot.set_pos
    copter.funtionality["get_pos"] = robot.get_position

    copter.funtionality["set_name"] = robot.set_name
    copter.funtionality["get_name"] = robot.get_tf_name

    rospy.logwarn("Starting VirtualCopter Semantix")
    copter.start()

    rospy.logwarn("Publishing")
    robot.publish_visual()

    while not rospy.is_shutdown():
        robot.br.sendTransform(robot.position,
                              tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), robot.name, "world")
        rate.sleep()
