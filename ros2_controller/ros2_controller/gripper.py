from .onrobot import RG

from util import config

class Gripper:
    def __init__(self):
        self.gripper = RG(
            config.GRTIPPER_NAME, 
            config.TOOLCHARGER_IP, 
            config.TOOLCHARGER_PORT
        )

    def open_grip(self):
        self.gripper.open_gripper()

    def close_grip(self):
        self.gripper.close_gripper()

    def get_status(self):
        return self.gripper.get_status()

    def __del__(self):
        self.gripper.close_connection()