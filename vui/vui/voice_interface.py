# voice_interface.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cb_interfaces.action import TaskSteps
from cb_interfaces.srv import Target

import os
import sys

from .stt_module import STTModule
from .langchain_module import LangChainModule
from .tts_module import TTSModule

from util import config, exceptions

sys.stderr = open(os.devnull, 'w')

class VoiceInterface(Node):
    def __init__(self):
        super().__init__('voice_interface')

        self.stt = STTModule()
        self.langchain = LangChainModule()
        self.tts = TTSModule()

        self.valid_targets = config.valid_targets
        self.valid_task_steps = config.valid_task_steps
        
        self.target_service_client = self.create_client(Target, config.TARGET)
        self.task_steps_action_client = ActionClient(self, TaskSteps, config.TASK_STEPS)

        self.target_count = 0
        self.action_count = 0

        self.start_once = self.create_timer(0.1, self.run)

    # STT
    def listener(self):
        self.get_logger().info("🎤 5초간 음성을 입력해주세요...")
        user_text = self.stt.listen()
        self.get_logger().info(f'user_text: {user_text}')
        return user_text

    # LangChain
    def extractor(self, user_text):
        targets, task_steps_per_target = self.langchain.extract(user_text)
        self.get_logger().info(f'targets: {targets}, task_steps_per_target: {task_steps_per_target}')
        return targets, task_steps_per_target

    # TTS
    def speaker(self, respoonse):
        self.tts.speak(respoonse)
        self.get_logger().info(f'🗣️: {respoonse}')

    # Validate
    def is_same_count(self, targets, task_steps_per_target):
        if len(targets) != len(task_steps_per_target) or len(targets) == 0:
            # self.speaker(exceptions.VUI_ERROR.ERROR_MESSAGES[403])
            raise exceptions.VUI_ERROR(403)
        self.get_logger().info('타겟과 작업 단계의 개수가 일치')

    def is_valid_targets(self, targets):
        if not all(t in self.valid_targets for t in targets):
            # self.speaker(exceptions.VUI_ERROR.ERROR_MESSAGES[404])
            raise exceptions.VUI_ERROR(404)
        self.get_logger().info('모든 타겟이 유효')

    def is_valid_task_steps(self, task_steps_per_target):
        for steps in task_steps_per_target:
            if not all(s in self.valid_task_steps for s in steps):
                # self.speaker(exceptions.VUI_ERROR.ERROR_MESSAGES[404])
                raise exceptions.VUI_ERROR(405)
        self.get_logger().info('모든 작업 단계가 유효')

    # Service to Image Processor
    def call_target_services(self, targets):
        self.targets = targets
        self.target_service_results = []
        self.target_count = len(targets)

        for target in targets:
            request = Target.Request()
            request.target_name = target
            future = self.target_service_client.call_async(request)
            future.add_done_callback(lambda f, t=target: self.target_service_callback(f, t))

    def target_service_callback(self, future, target):
        # 서비스 응답 결과 확인 및 로깅
        response = future.result()
        if response is None:
            msg = "Target 서비스 호출 실패"
            self.get_logger().error(msg)
            self.exit(msg)
            return
        self.get_logger().info("Target 서비스 호출 성공")
        
        self.target_service_results.append(response.found)
        self.target_count -= 1

        # 모든 서비스 응답이 끝났을 때만 액션 실행
        if self.target_count > 0:
            msg = f"남은 서비스 응답 존재: {self.target_count}"
            self.get_logger().info(msg)
            self.exit(msg)
            return
        self.get_logger().info("모든 서비스 응답 종료")

        # 모든 타겟을 탐지했을 때만 액션 실행
        if not all(self.target_service_results):
            msg = f"탐지 실패 타겟 존재: {target}"
            self.get_logger().error(msg)
            self.exit(msg)
            return
        self.get_logger().info('모든 타겟 탐지 성공')

        for idx, target in enumerate(self.targets):
            steps = self.steps_per_target[idx] if idx < len(self.steps_per_target) else []
            self.send_task_steps_action(steps)
        self.get_logger().info('모든 타겟에 대한 액션 호출')

    # Action to Controller
    def send_task_steps_action(self, task_steps):
        # 액션 서버 연결 확인
        if not self.task_steps_action_client.wait_for_server(timeout_sec=5.0):
            msg = "TaskSteps 액션 서버를 찾을 수 없습니다."
            self.get_logger().error(msg)
            self.exit(msg)
            return

        # 액션 goal 생성 및 전송
        goal_msg = TaskSteps.Goal()
        goal_msg.steps = task_steps
        self.get_logger().info(f"TaskSteps 액션 goal 전송: {task_steps}")

        send_goal_future = self.task_steps_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        feedback_text = f"[VUI] 피드백: {feedback.current_step} - {feedback.step_desc}"
        self.get_logger().info(feedback_text)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            msg = "Goal rejected"
            self.get_logger().error(msg)
            self.exit(msg)
            return
        self.get_logger().info("Goal accepted, 결과 대기 중...")
        
        self.action_count += 1
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.action_count -= 1
        
        result = future.result().result
        if not result.success:
            msg = f"Goal {result.success}"
            self.get_logger().error(msg)
            self.exit(msg)
            return
        result_text = f"최종결과는 success={result.success}, message={result.message} 입니다."
        self.get_logger().info(result_text)

        if self.action_count == 0:
            self.exit(result_text)

    # RUN
    def run(self):
        self.start_once.destroy()

        try:
            # STT
            user_text = self.listener()
            # user_text = "칼 가져와"
            # user_text = "숟가락 가져와"
            # user_text = "포크 가져와"

            # LangChain
            targets, task_steps_per_target = self.langchain.extract(user_text)
        
            # validate targets, task_steps_per_targets
            self.is_same_count(targets, task_steps_per_target)
            self.is_valid_targets(targets)
            self.is_valid_task_steps(task_steps_per_target)

            # Service to image_processor
            self.steps_per_target = task_steps_per_target
            self.call_target_services(targets)

            # Action to controller는 서비스 응답 후 콜백에서 실행
            
        except exceptions.VUI_ERROR as e:
            msg = str(e)
            self.get_logger().error(msg)
            self.exit(msg)
        
    def exit(self, msg):
        # raise exceptions.VUI_ERROR(406)
        self.speaker(msg)
        self.start_once = self.create_timer(0.1, self.run)

def main():
    rclpy.init()
    vi = VoiceInterface()

    try:
        rclpy.spin(vi)
    except KeyboardInterrupt:
        pass
    except exceptions.VUI_ERROR as e:
        print(str(e))
    finally:
        vi.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()