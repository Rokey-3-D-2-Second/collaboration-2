# ======================================================== #
# cb_interfaces
valid_targets = ["fork", "knife", "spoon"]
valid_task_steps = ["move_target", "move_tray", "move_home", "force", "close_grip", "open_grip"]

TARGET='/target'
TARGET_COORD='/target_coord'
TASK_STEPS='/task_steps'

# ======================================================== #
# image_processor

# ======================================================== #
# realsense_camera

# ======================================================== #
# ros2_controller

# controller
ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'mo6o9'
ROBOT_TCP = 'GripperDAmoon'
ROBOT_TOOL = 'Tool moon'
VEL = 100
ACC = 100

# mover

# forcer

# gripper
GRTIPPER_NAME = 'rg2'
TOOLCHARGER_IP = '192.168.1.1'
TOOLCHARGER_PORT = '502'

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