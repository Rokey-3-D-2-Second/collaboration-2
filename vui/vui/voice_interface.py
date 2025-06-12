# voice_interface.py

from dotenv import load_dotenv
import os

from .stt_module import STTModule
from .langchain_module import LangChainModule
from .tts_module import TTSModule

from util import exceptions

import warnings
import sys

# 모든 경고 무시
# warnings.filterwarnings("ignore")

# stderr (경고 메시지 출력 대상) -> 무시
sys.stderr = open(os.devnull, 'w')

class VoiceInterface:
    def __init__(self):
        load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), "../resource/.env"))
        self.api_key = os.getenv("OPENAI_API_KEY")
        # print("✅ DEBUG: API 키 확인 =", self.api_key)  # ← 이 줄 추가
        self.stt = STTModule()
        self.langchain = LangChainModule()
        self.tts = TTSModule()

    def run(self):
        print("[{__name__}] 🎤 음성 명령을 기다리는 중입니다... (로키야)")
        
        # 1. STT - 음성 입력 → 텍스트 변환
        user_text = self.stt.listen()

        # 2. LangChain - 명령어 의미 분석
        try:
            targets, task_steps = self.langchain.extract(user_text)
        except exceptions.VUI_ERROR as e:
            print(e)
            targets, task_steps = [], []

        # 3. 가상 전달 - (YOLO 및 Controller로 전달되는지 가정)
        print(f"[{__name__}] 🛠️ YOLO 모듈로 보낼 도구들: {targets}")
        print(f"[{__name__}] 📡 Controller 모듈로 보낼 목적지: {task_steps}")

        # 4. 응답 텍스트 구성
        if targets and task_steps:
            response_text = f"{' '.join(targets)}를 {' '.join(task_steps)} 순서로 처리하겠습니다."
        elif targets:
            response_text = f"{' '.join(targets)}를 어떻게 처리할까요?"
        elif task_steps:
            response_text = f"어떤 도구를 {', '.join(task_steps)} 순서로 처리할까요?"
        else:
            response_text = "죄송합니다. 이해하지 못했어요."

        print(f"[{__name__}] 🗣️ 최종 응답: {response_text}")

        # 5. TTS - 음성 출력
        self.tts.speak(response_text)


def main():
    vi = VoiceInterface()
    vi.run()

if __name__ == "__main__":
    main()