# ======================================================== #
# cb_interfaces
valid_targets = ["fork", "knife", "spoon"]
valid_task_steps = ["move_target", "move_tray", "move_home", "force", "close_grip", "open_grip"]
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
    # posx([585.9119873046875, 7.041282653808594, 479.5486755371094, 96.65674591064453, 179.85296630859375, -173.46255493164062]),
    # posx([143.47349548339844, 7.150089263916016, 479.7123718261719, 97.31204223632812, -179.94760131835938, -172.81277465820312]),
    posx([619.0982055664062, 2.6109795570373535, 210.99949645996094, 89.5136947631836, 179.8441162109375, 179.3989715576172]),
    posx([150.7991180419922, 1.8854933977127075, 211.10487365722656, 90.82644653320312, -179.95298767089844, -179.30828857421875,]),
]
homej = posj([0, -16, 95, 0, 101, 90])
homex = posx([251.9128875732422, 7.273128509521484, 479.5822448730469, 78.13613891601562, -179.99046325683594, 167.9998321533203])
trayx = posx([652.49, 16.510, 40.42, 90.0, 180.0, 180.0])

# forcer

# gripper
GRTIPPER_NAME = 'rg2'
TOOLCHARGER_IP = '192.168.1.1'
TOOLCHARGER_PORT = '502'

# moviter
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

MOTION_IKIN="/motion/ikin"
FOLLOW_JOINT_TRAJECTORY="/dsr_moveit_controller/follow_joint_trajectory"

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
- 타겟(target): fork, knife, spoon (복수일 수 있음, 공백 구분)
- 작업 단계(task_step): move_target, move_tray, move_home, force, close_grip, open_grip (복수일 수 있음, 공백 구분)
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

# 아래와 같은 입출력 포맷을 반드시 지킨다.

# 입력: "사용자 명령"
# 출력: target1 [target2 ...] / step1 step2 ... / step1 step2 ... (타겟별로)

예시
입력: "포크를 집어서 트레이에 세팅해줘"
출력: fork / move_target force open_grip close_grip move_tray open_grip move_home

입력: "스푼 줘"
출력: spoon / move_target force open_grip close_grip move_tray open_grip move_home

입력: "집에 가"
출력: / move_home

입력: "포크만"
출력: fork /

입력: "포크는 집으로, 스푼은 트레이에 세팅해줘"
출력: fork spoon / move_target force open_grip close_grip move_home open_grip / move_target force open_grip close_grip move_tray open_grip move_home

입력: "뭐라고 말하는지 모르겠음"
출력: /

아래와 같이 입력이 주어졌을 때, 반드시 동일한 포맷으로 답하라.

입력: "{user_input}"
출력:
"""
user_input="user_input"