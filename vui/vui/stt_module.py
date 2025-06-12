# stt_module.py

import sounddevice as sd
import scipy.io.wavfile as wav
import tempfile
import openai
from dotenv import load_dotenv
import os

from util import exceptions

class STTModule:
    def __init__(self, duration=5, samplerate=16000):
        # 환경변수 로드
        load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), "../resource/.env"))
        self.api_key = os.getenv("OPENAI_API_KEY")

        self.duration = duration
        self.samplerate = samplerate

    def listen(self):
        """마이크로부터 음성을 입력받고 Whisper API로 텍스트 반환"""
        print("[{__name__}] 🎤 5초간 음성을 입력해주세요...")
        audio = sd.rec(int(self.duration * self.samplerate), samplerate=self.samplerate, channels=1, dtype='int16')
        sd.wait()

        print("📡 Whisper 서버로 전송 중...")
        try:
            text = self._send_to_whisper(audio)
        except Exception:
            raise exceptions.VUI_ERROR(400)
        
        print(f"📝 변환 결과: {text}")
        return text

    def _send_to_whisper(self, audio):
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            wav.write(temp_wav.name, self.samplerate, audio)
            with open(temp_wav.name, "rb") as f:
                transcript = openai.Audio.transcribe(
                    model="whisper-1",
                    file=f,
                    api_key=self.api_key
                )
        return transcript["text"]


#############################
# ✅ 독립 실행 테스트용
#############################
if __name__ == "__main__":
    stt = STTModule()
    result = stt.listen()
    print("[{__name__}] 🎧 최종 STT 출력:", result)
