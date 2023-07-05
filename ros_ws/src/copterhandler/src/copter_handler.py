#!/usr/bin/env python

import rospy
import math
from visualization_msgs.msg import Marker
from AbstractVirtualCapability import VirtualCapabilityServer

from VirtualCopter import VirtualCopter


class CopterHandler:
    def __init__(self):
        self.position = [0, 0, 2.]
        self.rotation = [0, 0, 0, 1]
        self.scale = .2

        self.max_vel = .5
        self.acc = 0.004
        self.pub = rospy.Publisher("/robot", Marker, queue_size=1)

    def set_pos(self, p: list):
        pos = self.position
        dist = math.sqrt((p[0] - pos[0]) ** 2 + (p[1] - pos[1]) ** 2 + (p[2] - pos[2]) ** 2)

        dist_x = math.sqrt((p[0] - pos[0]) ** 2)
        dist_y = math.sqrt((p[1] - pos[1]) ** 2)
        dist_z = math.sqrt((p[2] - pos[2]) ** 2)

        current_vel = [0,0,0]
        while dist > 0.1:
            if dist_x > 0:
                current_vel[0] += self.acc
                current_vel[0] = self.max_vel if current_vel[0] > self.max_vel else current_vel[0]
            elif dist_x < 0:
                current_vel[0] -= self.acc
                current_vel[0] = self.max_vel if -current_vel[0] > self.max_vel else current_vel[0]
            else:
                current_vel[0] = 0

            if dist_y > 0:
                current_vel[1] += self.acc
                current_vel[1] = self.max_vel if current_vel[1] > self.max_vel else current_vel[1]
            elif dist_y < 0:
                current_vel[1] -= self.acc
                current_vel[1] = self.max_vel if -current_vel[1] > self.max_vel else current_vel[1]
            else:
                current_vel[1] = 0

            if dist_z > 0:
                current_vel[2] += self.acc
                current_vel[2] = self.max_vel if current_vel[2] > self.max_vel else current_vel[2]
            elif dist_z < 0:
                current_vel[2] -= self.acc
                current_vel[2] = self.max_vel if -current_vel[2] > self.max_vel else current_vel[2]
            else:
                current_vel[0] = 0

            dist_x = math.sqrt((p[0] - pos[0]) ** 2)
            dist_y = math.sqrt((p[1] - pos[1]) ** 2)
            dist_z = math.sqrt((p[2] - pos[2]) ** 2)
            dist = math.sqrt((p[0] - pos[0]) ** 2 + (p[1] - pos[1]) ** 2 + (p[2] - pos[2]) ** 2)
            self.publish_visual()
        self.position = pos

    def publish_visual(self):
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


if __name__ == '__main__':
    rospy.init_node('rosnode')
    rate = rospy.Rate(10)

    server = VirtualCapabilityServer(int(rospy.get_param('~semantix_port')))

    copter = VirtualCopter(server)
    robot = CopterHandler()


    copter.funtionality["set_pos"] = robot.set_pos
    copter.funtionality["get_pos"] = lambda: robot.position

    while not rospy.is_shutdown():
        robot.publish_visual()
        rate.sleep()
