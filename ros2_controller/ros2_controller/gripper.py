import time

from .onrobot import RG

from util import config, exceptions

class Gripper:
    def __init__(self):
        self.gripper = RG(
            config.GRTIPPER_NAME, 
            config.TOOLCHARGER_IP, 
            config.TOOLCHARGER_PORT
        )

        # init gripper
        self.close_grip()

    def close_grip(self):
        self.gripper.close_gripper()
        time.sleep(1.25)

    def open_grip(self):
        self.gripper.open_gripper()
        time.sleep(1.25)
        
    def is_close(self):
        # return self.get_status()[1] == 1
        return self.gripper.get_fingertip_offset() <= 5
    
    def is_open(self):
        return self.get_status()[1] == 0

    def get_status(self):
        return self.gripper.get_status()

    def __del__(self):
        self.gripper.close_connection()