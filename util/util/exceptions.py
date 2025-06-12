import traceback

# ======================================================== #
# image_processor
class IMAGE_PROCESSOR_ERROR(Exception):
    ERROR_MESSAGES = {
        100: "이미지 파일을 찾을 수 없습니다.",
        101: "이미지 포맷이 잘못되었습니다.",
        102: "이미지 처리 중 알 수 없는 오류가 발생했습니다.",
    }

    def __init__(self, code):
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

    def __init__(self, code):
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
    ERROR_MESSAGES = {
        300: "ROS2 컨트롤러 초기화 실패.",
        301: "명령 전송 실패.",
        302: "컨트롤러 응답 없음.",
    }

    def __init__(self, code):
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
    }
    """

    ERROR_MESSAGES = {
        400: "음성 인식 실패.",
        401: "키워드 추출 실패.",
        402: "명령어 해석 실패.",
    }

    def __init__(self, code):
        super().__init__(code)
        tb = traceback.extract_stack()[-2]
        self.file = tb.filename
        self.line = tb.lineno
        self.code = code
        self.message = self.ERROR_MESSAGES.get(code, "정의되지 않은 VUI 에러입니다.")

    def __str__(self):
        return f"[{self.file}:{self.line}] (code: {self.code}) {self.message}"