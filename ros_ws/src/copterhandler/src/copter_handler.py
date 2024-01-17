#!/usr/bin/env python
from copy import copy, deepcopy

import numpy as np
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler
from AbstractVirtualCapability import VirtualCapabilityServer
from time import sleep
from visualization_msgs.msg import Marker

from VirtualCopter import VirtualCopter


class CopterHandler:
    def __init__(self):

        self.position = np.array([0, 0, 2.])
        self.rotation = [0, 0, 0, 1]
        self.scale = 1.0

        self.tf_position = None
        self.max_vel = 1.5
        self.acc = 0.001
        self.pub = rospy.Publisher("/robot", Marker, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.name = f"copter@{int(rospy.get_param('~semantix_port'))}"
        self.goal = self.position
        self.flying = False

    def set_rot(self, rot):
        self.rotation = rot # quaternion_from_euler(rot[0], rot[1], rot[2])
        return self.rotation

    def rotate(self, axis, deg):
        axis = np.array(axis)
        theta = np.deg2rad(deg)
        self.rotation = list(quaternion_about_axis(theta, axis))
        return self.rotation

    def place_block(self, goal: list):
        vel = self.acc
        self.tf_position = deepcopy(self.position)
        self.tf_position[2] += .1
        self.tf_position[1] += .2

        while True:
            goal = np.array(goal)
            vector = goal - self.tf_position

            if np.linalg.norm(vector) < self.max_vel:
                self.tf_position = goal
                self.publish_visual()
                return self.position.tolist()

            current_vel = vel * vector / np.linalg.norm(vector)
            self.tf_position += current_vel

            self.publish_visual()
            # sleep((abs(current_vel[0]) + abs(current_vel[1]) + abs(current_vel[2]))*.01)
            vel += self.acc
            vel = min(vel, self.max_vel)

    def remove_tf(self):
        self.tf_position = None

    def get_position(self):
        return list(self.position)

    def get_tf_name(self):
        return self.name

    def set_name(self, name):
        self.name = name

    def set_pos(self, goal: list):
        self.goal = goal
        rospy.logwarn(f"Flying to Position: {goal}")
        self.br.sendTransform((goal[0], goal[1], goal[2]),
                              tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "goal", "world")
        # already flying...
        if self.flying:
            while self.flying:
                sleep(.001)
            return self.position.tolist()
        else:
            self.flying = True
            vel = self.acc
            while True:
                goal = np.array(self.goal)
                vector = goal - self.position

                if np.linalg.norm(vector) < self.max_vel:
                    self.position = goal
                    self.publish_visual()
                    self.flying = False
                    return self.position.tolist()

                current_vel = vel * vector / np.linalg.norm(vector)
                self.position += current_vel
                self.publish_visual()
                # TODO wait appropriate
                sleep(np.sum(np.abs(current_vel)) * .01)
                # sleep((abs(current_vel[0]) + abs(current_vel[1]) + abs(current_vel[2])))
                vel += self.acc
                vel = min(vel, self.max_vel)

    def publish_visual(self):
        # rospy.logwarn(f"Publishing {self.position}")
        marker = Marker()
        marker.id = int(rospy.get_param('~semantix_port'))
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = self.name
        marker.lifetime = rospy.Duration(0)
        # marker.color.r = .1
        # marker.color.g = .15
        # marker.color.b = .3
        marker.mesh_use_embedded_materials = True
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.w = self.rotation[0]
        marker.pose.orientation.x = self.rotation[1]
        marker.pose.orientation.y = self.rotation[2]
        marker.pose.orientation.z = self.rotation[3]
        # Scale down
        marker.scale.x = self.scale
        marker.scale.y = self.scale
        marker.scale.z = self.scale
        marker.color.a = 1
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = r"package://copterhandler/meshes/copter.dae"

        self.pub.publish(marker)

        pos = None
        if self.tf_position is not None:
            pos = self.tf_position
        # TF
        # This is bc ROS takes quaternion as: x,y,z,w
        rot = [self.rotation[1], self.rotation[2], self.rotation[3], self.rotation[0]]
        self.br.sendTransform(pos if pos is not None else self.position,
                              rot, rospy.Time.now(), self.name, "world")


if __name__ == '__main__':
    xmlrpc_port = int(rospy.get_param('~xmlrpc_port'))
    tcpros_port = int(rospy.get_param('~tcpros_port'))
    rospy.init_node('rosnode', xmlrpc_port=xmlrpc_port, tcpros_port=tcpros_port)
    rate = rospy.Rate(30)

    rospy.logwarn("Starting")

    server = VirtualCapabilityServer(int(rospy.get_param('~semantix_port')))

    copter = VirtualCopter(server)
    robot = CopterHandler()

    rospy.logwarn("Setting functionality")

    copter.functionality["set_pos"] = robot.set_pos
    copter.functionality["get_pos"] = lambda: list(robot.position)

    copter.functionality["set_name"] = robot.set_name
    copter.functionality["get_name"] = lambda: robot.name

    copter.functionality["get_rot"] = lambda: robot.rotation
    copter.functionality["set_rot"] = robot.set_rot
    copter.functionality["rotate"] = robot.rotate
    copter.functionality["place_block"] = robot.place_block
    copter.functionality["remove_tf"] = robot.remove_tf

    rospy.logwarn("Starting VirtualCopter Semantix")
    copter.start()

    rospy.logwarn("Publishing")
    robot.publish_visual()

    while not rospy.is_shutdown():
        robot.publish_visual()
        rate.sleep()
