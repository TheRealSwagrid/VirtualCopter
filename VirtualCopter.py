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
        self.funtionality = {"get_pos": None, "set_pos": None}
        self.max_vel = 0.25
        self.acc = 0.002

    def FlyToPosition(self, params: dict):
        formatPrint(self, f"Get Position {params}")
        if self.funtionality["get_pos"] is not None:
            self.position = self.funtionality["set_pos"](params["Position3D"])
        return {"Position3D": self.position}

    def SetPosition(self, params: dict):
        formatPrint(self, f"Get Position {params}")
        if self.funtionality["get_pos"] is not None:
            self.position = self.funtionality["set_pos"](params["Position3D"])
        return {"Position3D": self.position}

    def GetPosition(self, params: dict):
        formatPrint(self, f"Get Position {params}")
        return {"Position3D": self.position}

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
        listener = PlacerRobot(server)
        listener.start()
        signal.signal(signal.SIGTERM, handler)
        listener.join()
    # Needed for properly closing, when program is being stopped wit a Keyboard Interrupt
    except KeyboardInterrupt:
        print("[Main] Received KeyboardInterrupt")
        server.kill()
        listener.kill()
