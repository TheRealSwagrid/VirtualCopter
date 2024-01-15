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
        self.current_block_id = -1
        self.position = [0., 0., 0.]
        self.functionality = {"get_pos": None, "set_pos": None, "get_name": None, "set_name": None, "get_rot": None,
                              "set_rot": None, "rotate": None, "place_block": None, "remove_tf": None}
        self.direction = [1., 1., 1.]
        self.arming_status = False
        self.lock = False

    def TransferBlock(self, params: dict):
        block_id = params["SimpleIntegerParameter"]
        if self.current_block_id != -1 and block_id != -1:
            raise ValueError(f"Still got the Block {self.current_block_id} while waiting for block {block_id}")
        if block_id == -1:
            self.current_block_id = -1
            return params
        self.current_block_id = block_id
        self.invoke_sync("attach_block", {"SimpleIntegerParameter": self.current_block_id,
                                          "SimpleStringParameter": self.functionality["get_name"]()})
        return params

    def PlaceBlock(self, params: dict):
        pos = params["Position3D"]
        if self.current_block_id is not None:
            if self.functionality["place_block"] is not None:
                self.functionality["place_block"](pos)
            # Wait until the block has been set with the accurate position (BlockHandler is slow)
            sleep(.25)
            self.invoke_sync("detach_block", {"SimpleIntegerParameter": self.current_block_id})
            if self.functionality["remove_tf"] is not None:
                self.functionality["remove_tf"]()
            self.current_block_id = -1
        else:
            raise Exception("No Block found")
        return {}

    def FlyToPosition(self, params: dict):
        formatPrint(self, f"Set Position {params}")
        previousLock = copy(self.lock)
        if self.functionality["set_pos"] is not None:
            self.position = self.functionality["set_pos"](params["Position3D"])
        self.lock = previousLock
        return {"Position3D": self.position}

    def SetPosition(self, params: dict):
        formatPrint(self, f"Set Position {params}")
        previousLock = copy(self.lock)
        self.lock = True
        if self.functionality["set_pos"] is not None:
            self.position = self.functionality["set_pos"](params["Position3D"])
        self.lock = previousLock
        return {"Position3D": self.position}

    def GetPosition(self, params: dict):
        formatPrint(self, f"Get Position {params}")
        if self.functionality["set_pos"] is not None:
            self.position = self.functionality["get_pos"]()
        return {"Position3D": self.position}

    def Settf_name(self, params: dict):
        tf_name = params["SimpleStringParameter"]
        if self.functionality["set_name"] is not None:
            self.position = self.functionality["set_name"](tf_name)
        return {"SimpleStringParameter": tf_name}

    def Gettf_name(self, params: dict):
        tf_name = "NO_ROS_CONNECTION"
        if self.functionality["get_name"] is not None:
            tf_name = self.position = self.functionality["get_name"]()
        return {"SimpleStringParameter": tf_name}

    def SetRotation(self, params: dict):
        quat = params["Quaternion"]
        if self.functionality["set_rot"] is not None:
            self.functionality["set_rot"](quat)
        return {"Quaternion": quat}

    def GetRotation(self, params: dict):
        quat = [0, 0, 0, 0]
        if self.functionality["get_rot"] is not None:
            quat = self.functionality["get_rot"]()
        return {"Quaternion": quat}

    def RotateAroundAxis(self, params: dict):
        axis = params["Axis"]
        if axis == 'z':
            axis = [0, 0, 1]
        elif axis == 'y':
            axis = [0, 1, 0]
        elif axis == 'x':
            axis = [1, 0, 0]
        degree = params["SimpleDoubleParameter"]
        if self.functionality["get_name"] is not None:
            quat = self.functionality["rotate"](axis, degree)
        formatPrint(self, f"New Quaternion {quat}")
        return {"Quaternion": quat}

    def GetDirection(self, params: dict):
        return {"Vector3": self.direction}

    def SetDirection(self, params: dict):
        new_direction = params["Vector3"]
        self.direction = new_direction
        return self.GetDirection(params)

    def SetArmingStatus(self, params: dict):
        formatPrint(self, f"Set Arming Status to {params}")
        self.arming_status = params["SimpleBooleanParameter"]
        return self.GetArmingStatus(params)

    def GetArmingStatus(self, params: dict):
        return {"SimpleBooleanParameter": self.arming_status}

    def GetLock(self, params: dict):
        return {"bool": self.lock}

    def SetLock(self, params: dict):
        self.lock = params["bool"]
        return self.GetLock({})

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
