# ======================================================== #
# cb_interfaces
valid_targets = ["fork", "knife", "spoon", "conta", "nothing"]
valid_task_steps = ["move_target", "move_tray", "move_home", "move_conta", "force", "close_grip", "open_grip", "nothing"]
# valid_task_steps = ["move_target", "move_tray", "move_home", "hold_target", "release_target", "close_grip", "open_grip"]

TARGET='/target'
TARGET_COORD='/target_coord'
TASK_STEPS='/task_steps'

# ======================================================== #
# ros2_controller
from DR_common2 import posx, posj

# controller
# ROBOT_ID = 'dsr01'
ROBOT_ID = ''
ROBOT_MODEL = 'mo6o9'
ROBOT_TCP = 'GripperDAmoon'
ROBOT_TOOL = 'Tool moon'
VEL = 100
ACC = 100

# mover
scanx = [
    # posx([150.7991180419922, 1.8854933977127075, 211.10487365722656, 90.82644653320312, -179.95298767089844, -90.]),
    # posx([619.0982055664062, 2.6109795570373535, 210.99949645996094, 89.5136947631836, 179.8441162109375, -90.]),
    # posx([619.0982055664062, 2.6109795570373535, 210.99949645996094, 89.5136947631836, 179.8441162109375, 90.]),
    # posx([150.7991180419922, 1.8854933977127075, 211.10487365722656, 90.82644653320312, -179.95298767089844, 90.]),
    posx([156.07766723632812, 15.572643280029297, 270.3399963378906, 90, 180, -90]),
    posx([525.5095825195312, 16.24213981628418, 270.34136962890625, 90, 180, -90]),
    posx([694.7186889648438, 16.401952743530273, 305.827392578125, 0.3940909206867218, 155.6670684814453, -180.]),
    posx([694.7186889648438, 16.401952743530273, 305.827392578125, 0.3940909206867218, 155.6670684814453, 0.]),
    posx([525.5095825195312, 16.24213981628418, 270.34136962890625, 90, 180, 90]),
    posx([156.07766723632812, 15.572643280029297, 270.3399963378906, 90, 180, 90]),
    
]
homej = posj([1, -17, 101, 0, 96, 90])
homex = posx([253.2886199951172, 4.772646903991699, 148.06309509277344, 14.613371849060059, -179.4019012451172, 103.75443267822266])
# homex = posx([198.84828186035156, 1.7979012727737427, 189.55508422851562, 96.45184326171875, -179.9581298828125, -173.48715209960938])
trayx = posx([702.49, 16.510, 25.42, 90.0, 180.0, 180.0])
contax = posx([374.51, -141.74, 28.02, 166.43, 179.80, -103.52])

# forcer

# gripper
GRTIPPER_NAME = 'rg2'
TOOLCHARGER_IP = '192.168.1.1'
TOOLCHARGER_PORT = '502'

CLOSE="close"
OPEN="open"
HOLDING="holding"

# motion_planner
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

JOINT_STATE="/joint_states"
MOTION_IKIN="/motion/ikin"
COMPUTE_IK="/compute_ik"
MOVE_ACTION="/move_action"
EXECUTE_TRAJECTORY="/execute_trajectory"
FOLLOW_JOINT_TRAJECTORY="/dsr_moveit_controller/follow_joint_trajectory"

PLANNER_IDS=[
    "AnytimePathShortening",
    "SBL",
    "EST",
    "LBKPIECE",
    "BKPIECE",
    "KPIECE",
    "RRT",
    "RRTConnect",
    "RRTstar",
    "TRRT",
    "PRM",
    "PRMstar",
    "FMT",
    "BFMT",
    "PDST",
    "STRIDE",
    "BiTRRT",
    "LBTRRT",
    "BiEST",
    "ProjEST",
    "LazyPRM",
    "LazyPRMstar",
    "SPARS",
    "SPARStwo",
    "TrajOptDefault",
]

# ======================================================== #
# image_processor

CAMERA_INFO="/camera/camera/color/camera_info"
COLOR_IMAGE="/camera/camera/color/image_raw/compressed"
ALIGNED_IMAGE="/camera/camera/aligned_depth_to_color/image_raw"
GET_CURREN_POSX = ROBOT_ID + "/aux_control/get_current_posx"

# ======================================================== #
# realsense_camera

# ======================================================== #
# vui
prompt_template = """
너는 사용자 명령에서
- 타겟(target): fork, knife, spoon, conta, nothing (복수일 수 있음, 공백 구분)
- 작업 단계(task_step): move_home, move_target, move_tray, move_conta, force, close_grip, open_grip, nothing (복수일 수 있음, 공백 구분)
을 추출한다.

아래 규칙을 반드시 따른다!!

출력 규칙
- 각 타겟마다 일련의 작업단계(task_step) 시퀀스를 매칭해서 /로 구분
- 여러 타겟이 있으면 각 타겟별로 /로 구분된 task_step 시퀀스를 나열
- 타겟 수와 총 작업 단계 수는 반드시 동일해야 한다.
- 타겟 정보가 부족하면: / task_steps
- 작업 정보가 부족하면: targets /
- 둘 다 부족하면: /
- 반드시 아무런 설명 없이 결과만 출력한다.

아래와 같은 입출력 포맷을 반드시 지킨다.

# 입력: "사용자 명령"
# 출력: target1 [target2 ...] / step1 step2 ... / step1 step2 ... (타겟별로)

예시
입력: "포크를 집어서 트레이에 세팅해줘"
출력: fork / move_target force open_grip close_grip move_tray open_grip move_home

입력: "포크 정리"
출력: fork / move_target force open_grip close_grip move_tray open_grip move_home

입력: "스푼 줘"
출력: spoon / move_target force open_grip close_grip move_tray open_grip move_home

입력: "숟가락 줘"
출력: spoon / move_target force open_grip close_grip move_tray open_grip move_home

입력: "스푼 줄래"
출력: spoon / move_target force open_grip close_grip move_tray open_grip move_home

입력: "스푼 좀 줘"
출력: spoon / move_target force open_grip close_grip move_tray open_grip move_home

입력: "스푼 2개 줘"
출력: spoon spoon / move_target force open_grip close_grip move_tray open_grip move_home / move_target force open_grip close_grip move_tray open_grip move_home

입력: "집에 가"
출력: nothing / move_home

입력: "포크만"
출력: fork / nothing

입력: "포크는 집으로, 스푼은 트레이에 세팅해줘"
출력: fork spoon / move_target force open_grip close_grip move_home open_grip / move_target force open_grip close_grip move_tray open_grip move_home

입력: "스푼, 포크, 나이프 각각 트레이에 세팅해줘"
출력: spoon fork knife / move_target force open_grip close_grip move_home open_grip  / move_target force open_grip close_grip move_home open_grip  / move_target force open_grip close_grip move_home open_grip 

입력: "스푼 2개, 포크 1개, 나이프 1개 각각 트레이에 세팅해줘"
출력: spoon spoon fork knife / move_target force open_grip close_grip move_home open_grip  / move_target force open_grip close_grip move_home open_grip  / move_target force open_grip close_grip move_home open_grip 

입력: "모든 도구를 각각 트레이에 세팅해줘"
출력: spoon fork knife / move_target force open_grip close_grip move_home open_grip  / move_target force open_grip close_grip move_home open_grip  / move_target force open_grip close_grip move_home open_grip 

입력: "스푼, 포크, 나이프 각각 트레이에 정리해줘"
출력: spoon fork knife / move_target force open_grip close_grip move_home open_grip  / move_target force open_grip close_grip move_home open_grip  / move_target force open_grip close_grip move_home open_grip 

입력: "모든 도구를 트레이에 정리해줘"
출력: spoon fork knife / move_target force open_grip close_grip move_home open_grip  / move_target force open_grip close_grip move_home open_grip  / move_target force open_grip close_grip move_home open_grip 

입력: "스푼, 포크, 나이프 각각 세팅해줘"
출력: spoon fork knife / move_target force open_grip close_grip move_home open_grip  / move_target force open_grip close_grip move_home open_grip  / move_target force open_grip close_grip move_home open_grip 

입력: "뭐라고 말하는지 모르겠음"
출력: nothing / nothing

입력: "이상있는 거 분류해줘"
출력: conta / move_target force open_grip close_grip move_conta open_grip move_home

아래와 같이 입력이 주어졌을 때, 반드시 동일한 포맷으로 답하라.

입력: "{user_input}"
출력:
"""
user_input="user_input"