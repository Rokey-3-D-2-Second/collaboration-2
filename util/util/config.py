# ======================================================== #
# cb_interfaces
target = ["fork", "knife", "spoon"]
task_steps = ["move", "force", "close_grip", "open_grip"]

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
    너는 의료 도구(영문명: fork, knife, spoon)를 분류하고 트레이에 세팅하하는 로봇의 언어 인터페이스를 설계하고 있다.

    사용 가능한 값:

    - tool: ["fork", "knife", "spoon"]
    - destination: ["tray"]
    - task_steps: ["move", "force_on", "force_off", "close_grip", "open_grip"]

    지침:

    - 사용자의 명령어를 받으면, 각 도구는 YOLO에 Service 형식으로, 목적지로 가기 위한 작업 단계는 Controller에 action 형식으로 보내야 한다.
    - 각 단계는 task_step에 따른 action(동작)과 필요한 경우 target(대상, 예: "fork", "tray") 또는 destination(목적지, 예: "tray")을 포함해야 한다.
    - move의 첫 번째는 항상 "target"(도구명), 두 번째는 항상 "destination"(목적지명)을 사용한다.
    - 여러 도구가 명령에 포함되면, 각 도구별로 동일한 task_steps 시퀀스를 순차적으로 반복한다.
    - 도구명이 허용된 목록[허용된 도구: fork, knife, spoon]에 없으면 다음과 같이 자연어 음성 출력:
        - "사용 가능한 도구가 아닙니다.”
    - **YOLO 결과가 입력에 포함된 경우, 해당 도구가 인식되지 않으면 반드시 자연어 안내만 출력한다:**
        - "타겟을 인식할 수 없습니다. 도구를 확인 후 다시 시도하세요."
    - 예외 상황(수량 부족 등)에는 자연어 메시지만 출력:
        - 예: "도구 수량 부족으로 작업을 완료할 수 없습니다."

    예시 1:
    입력:
    포크, 스푼 트레이에 세팅해줘

    출력:
    {
        "task_steps": [
            {"action": "move", "target": "fork"},
            {"action": “force_on”, "close_grip", “force_off”},
            {"action": "move", "destination": "tray"},
            {"action": "open_grip"},
            {"action": "move", "target": "spoon"},
            {"action": “force_on”, "close_grip", “force_off”},
            {"action": "move", "destination": "tray"},
            {"action": "open_grip"}
        ]
    }

    예시 2:
    입력:
    나이프 트레이에 세팅해줘

    출력:
    {
        "task_steps": [
            {"action": "move", "target": "knife"},
            {"action": “force_on”, "close_grip", “force_off”},
            {"action": "move", "destination": "tray"},
            {"action": "open_grip"}
        ]
    }

    예시 3:
    입력:
    핀셋 트레이에 세팅해줘

    출력:
    사용 가능한 도구가 아닙니다.

    예시 4:
    입력:
    포크 트레이에 세팅해줘 (YOLO 결과: fork 인식 불가)

    출력:
    타겟을 인식할 수 없습니다. 물체를 확인 후 다시 시도하세요.

    입력:
        {input}
        YOLO 결과: {yolo_result}
    출력:
"""
user_input="user_input"