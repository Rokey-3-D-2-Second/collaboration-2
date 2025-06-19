import traceback

# ======================================================== #
# image_processor
class IMAGE_PROCESSOR_ERROR(Exception):
    """
    ERROR_MESSAGES = {
        100: "이미지/깊이/내부파라미터 미수신",
        101: "타겟을 찾을 수 없습니다.",
        103: "유효하지 않은 깊이 값",
        104: "현재 좌표를 알 수 없습니다.",
        105: "이미지 처리 중 알 수 없는 오류가 발생했습니다.",
        106: "Exit"
    }
    """

    ERROR_MESSAGES = {
        100: "이미지/깊이/내부파라미터 미수신",
        101: "타겟을 찾을 수 없습니다.",
        103: "유효하지 않은 깊이 값",
        104: "이미지 처리 중 알 수 없는 오류가 발생했습니다.",
        105: "이미지 처리 중 알 수 없는 오류가 발생했습니다.",
        106: "Exit"
    }

    def __init__(self, code: int):
        super().__init__(code)
        tb = traceback.extract_stack()[-2]
        self.file = tb.filename
        self.line = tb.lineno
        self.code = code
        self.message = self.ERROR_MESSAGES.get(code, "정의되지 않은 이미지 처리 에러입니다.")

    def __str__(self):
        return f"[{self.file}:{self.line}] (code: {self.code}) {self.message}"

# ======================================================== #
# realsense_camera
class REALSENSE_CAMERA_ERROR(Exception):
    ERROR_MESSAGES = {
        200: "카메라 연결 실패.",
        201: "카메라 데이터 수신 오류.",
        202: "카메라 권한 오류.",
    }

    def __init__(self, code: int):
        super().__init__(code)
        tb = traceback.extract_stack()[-2]
        self.file = tb.filename
        self.line = tb.lineno
        self.code = code
        self.message = self.ERROR_MESSAGES.get(code, "정의되지 않은 카메라 에러입니다.")

    def __str__(self):
        return f"[{self.file}:{self.line}] (code: {self.code}) {self.message}"

# ======================================================== #
# ros2_controller
class ROS2_CONTROLLER_ERROR(Exception):
    """
    ERROR_MESSAGES = {
        300: "Unkown Step",
        301: "No detection on Gripper",
        302: "No Target",
    }
    """

    ERROR_MESSAGES = {
        300: "Unkown Step",
        301: "No detection on Gripper",
        302: "No Target",
    }

    def __init__(self, code: int):
        super().__init__(code)
        tb = traceback.extract_stack()[-2]
        self.file = tb.filename
        self.line = tb.lineno
        self.code = code
        self.message = self.ERROR_MESSAGES.get(code, "정의되지 않은 ROS2 컨트롤러 에러입니다.")

    def __str__(self):
        return f"[{self.file}:{self.line}] (code: {self.code}) {self.message}"

# ======================================================== #
# vui
class VUI_ERROR(Exception):
    """
    ERROR_MESSAGES = {
        400: "음성 인식 실패.",
        401: "명령어 해석 실패.",
        402: "키워드 추출 실패.",
        403: "타겟과 작업 단계의 개수가 일치하지 않습니다.",
        404: "유효하지 않은 타겟이 포함되어 있습니다.",
        405: "유효하지 않은 작업 단계가 포함되어 있습니다.",
        406: "Exit"
    }
    """

    ERROR_MESSAGES = {
        400: "음성 인식 실패.",
        401: "키워드 추출 실패.",
        402: "명령어 해석 실패.",
        403: "타겟과 작업 단계의 개수가 일치하지 않습니다.",
        404: "유효하지 않은 타겟이 포함되어 있습니다.",
        405: "유효하지 않은 작업 단계가 포함되어 있습니다.",
        406: "Exit",
        407: "Target에 Nothing이 포함되어 있습니다.",
        408: "Task Step에 Nothing이 포함되어 있습니다.",
    }

    def __init__(self, code: int):
        super().__init__(code)
        tb = traceback.extract_stack()[-2]
        self.file = tb.filename
        self.line = tb.lineno
        self.code = code
        self.message = self.ERROR_MESSAGES.get(code, "정의되지 않은 VUI 에러입니다.")

    def __str__(self):
        return f"[{self.file}:{self.line}] (code: {self.code}) {self.message}"