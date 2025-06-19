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
        return self.get_status()[1] == 1
    
    def is_open(self):
        return self.get_status()[1] == 0
    
    def is_hold(self):
        width = self.get_width()
        if width < 10:
            return config.CLOSE
        if width < 30:
            return config.HOLDING
        return config.OPEN

    def get_status(self):
        return self.gripper.get_status()
    
    def get_width(self):
        return self.gripper.get_width()

    def __del__(self):
        self.gripper.close_connection()