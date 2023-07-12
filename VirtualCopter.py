#!/usr/bin/env python
import signal
import sys
import time
from copy import copy
from time import sleep

from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint


class VirtualCopter(AbstractVirtualCapability):
    def __init__(self, server):
        super().__init__(server)
        self.position = [0., 0., 0.]
        self.funtionality = {"get_pos": None, "set_pos": None, "get_name": None, "set_name": None, "get_rot": None, "set_rot": None, "rotate":None}
        self.max_vel = 0.25
        self.acc = 0.002

    def FlyToPosition(self, params: dict):
        formatPrint(self, f"Set Position {params}")
        if self.funtionality["set_pos"] is not None:
            self.position = self.funtionality["set_pos"](params["Position3D"])
        return {"Position3D": self.position}

    def SetPosition(self, params: dict):
        formatPrint(self, f"Get Position {params}")
        if self.funtionality["set_pos"] is not None:
            self.position = self.funtionality["set_pos"](params["Position3D"])
        return {"Position3D": self.position}

    def GetPosition(self, params: dict):
        formatPrint(self, f"Get Position {params}")
        if self.funtionality["set_pos"] is not None:
            self.position = self.funtionality["get_pos"](params["Position3D"])
        return {"Position3D": self.position}

    def Settf_name(self, params: dict):
        tf_name = params["SimpleStringParameter"]
        if self.funtionality["set_name"] is not None:
            self.position = self.funtionality["set_name"](tf_name)
        return {"SimpleStringParameter": tf_name}

    def Gettf_name(self, params: dict):
        tf_name = "NO_ROS_CONNECTION"
        if self.funtionality["get_name"] is not None:
            tf_name = self.position = self.funtionality["get_name"]()
        return {"SimpleStringParameter": tf_name}

    def SetRotation(self, params: dict):
        quat = params["Quaternion"]
        if self.funtionality["set_rot"] is not None:
             self.funtionality["set_rot"](quat)
        return {"Quaternion": quat}

    def GetRotation(self, params: dict):
        quat = [0,0,0,0]
        if self.funtionality["get_name"] is not None:
            quat = self.funtionality["get_rot"]()
        return {"Quaternion": quat}

    def RotateAroundAxis(self, params: dict):
        axis = params["Axis"]
        degree = params["SimpleDoubleParameter"]
        if self.funtionality["get_name"] is not None:
            quat = self.funtionality["rotate"](axis, degree)
        return {"Quaternion": quat}


    def loop(self):
        pass


if __name__ == '__main__':
    # Needed for properly closing when process is being stopped with SIGTERM signal
    def handler(signum, frame):
        print("[Main] Received SIGTERM signal")
        listener.kill()
        quit(1)


    try:
        port = None
        if len(sys.argv[1:]) > 0:
            port = int(sys.argv[1])
        server = VirtualCapabilityServer(port)
        listener = VirtualCopter(server)
        listener.start()
        signal.signal(signal.SIGTERM, handler)
        listener.join()
    # Needed for properly closing, when program is being stopped wit a Keyboard Interrupt
    except KeyboardInterrupt:
        print("[Main] Received KeyboardInterrupt")
        server.kill()
        listener.kill()
