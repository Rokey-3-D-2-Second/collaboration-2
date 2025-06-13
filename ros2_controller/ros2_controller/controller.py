import rclpy
from rclpy.node import Node

from mover import Mover
from forcer import Forcer
from gripper import Gripper

from util import config

import DR_init
DR_init.__dsr__id = config.ROBOT_ID
DR_init.__dsr__model = config.ROBOT_MODEL

class Controller:
    def set_dependencies(
        self, 
        check_motion,
    ):
        self.check_motion = check_motion

    def __init__(self, node: Node):
        # super().__init__('drawer_and_eraser', namespace=ROBOT_ID)

        self.node = node

        self.mover = Mover()
        self.forcer = Forcer()
        self.gripper = Gripper()

    

    def __del__(self):
        self.node.destroy_node()

def init_modules(        
        movel, 
        check_motion,
        
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,

        get_current_posx,

        DR_AXIS_Z,

        DR_FC_MOD_REL
    ):

    Controller.set_dependencies(
        check_motion,
    )
    Mover.set_dependencies(
        movel, 

        get_current_posx,
    )
    Forcer.set_dependencies(
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,

        DR_AXIS_Z,

        DR_FC_MOD_REL
    )

def main():
    rclpy.init()
    node = rclpy.create_node('drawer_and_eraser', namespace=config.ROBOT_ID)
    DR_init.__dsr__node = node
    
    # import dsr api
    from DSR_ROBOT2 import (
        get_tcp, get_tool,
        set_tcp, set_tool, 
        set_ref_coord,

        movel, amovesx,
        check_motion,
        
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,

        get_current_posx,

        DR_AXIS_Z,

        DR_FC_MOD_REL
    )

    # init modules
    init_modules(  
        movel, 
        check_motion,
        
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,

        get_current_posx,

        DR_AXIS_Z,

        DR_FC_MOD_REL
    )

    # set tcp & tool
    set_tcp(config.ROBOT_TCP)
    set_tool(config.ROBOT_TOOL)

    # check tcp & tool
    tcp, tool = get_tcp(), get_tool()
    print(f'tcp: {tcp}, tool: {tool}')
    if tcp != config.ROBOT_TCP or tool != config.ROBOT_TOOL:
        node.destroy_node()
        rclpy.shutdown()
        return

    controller = Controller(node)

    del controller
    rclpy.shutdown()
