#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from AbstractVirtualCapability import VirtualCapabilityServer

from VirtualCopter import VirtualCopter


class CopterHandler:
    def __init__(self):
        self.position = [0, 0, 2.]
        self.rotation = [0, 0, 0, 1]
        self.scale = .2
        self.pub = rospy.Publisher("/robot", Marker, queue_size=1)

    def set_pos(self, pos: list):
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


    copter.funtionality["set_pos"] = None
    copter.funtionality["get_pos"] = None

    while not rospy.is_shutdown():
        robot.publish_visual()
        rate.sleep()