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

    def close_grip(self):
        self.gripper.close_gripper()
        time.sleep(2.0)
        # status = self.get_status()
        # print(f"{__name__}: {status}")
        # if status[1] == 0:
        #     raise exceptions.ROS2_CONTROLLER_ERROR(301)

    def open_grip(self):
        self.gripper.open_gripper()
        time.sleep(2.0)
        # status = self.get_status()
        # print(f"{__name__}: {status}")
        
    def is_close(self):
        return self.get_status()[1] == 1
    
    def is_open(self):
        return self.get_status()[1] == 0

    def get_status(self):
        return self.gripper.get_status()

    def __del__(self):
        self.gripper.close_connection()