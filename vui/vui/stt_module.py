import sounddevice as sd
import scipy.io.wavfile as wav
import tempfile
import openai
from dotenv import load_dotenv
import os

class STTModule:
    def __init__(self, duration=5.0, samplerate=16000):
        load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), "../resource/.env"))
        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise RuntimeError("OPENAI_API_KEY is not set")

        openai.api_key = self.api_key

        self.duration = duration
        self.samplerate = samplerate

    def listen(self):
        print("🎤 5초간 음성을 입력해주세요...")
        audio = sd.rec(int(self.duration * self.samplerate), samplerate=self.samplerate, channels=1, dtype='int16')
        sd.wait()

        print("📡 Whisper 서버로 전송 중...")
        try:
            text = self._send_to_whisper(audio)
        except openai.error.OpenAIError as e:
            print(f"OpenAI API Error: {e}")
            raise
        except Exception as e:
            print(f"Unknown error: {e}")
            raise

        print(f"📝 변환 결과: {text}")
        return text

    def _send_to_whisper(self, audio):
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            wav.write(temp_wav.name, self.samplerate, audio)
            with open(temp_wav.name, "rb") as f:
                transcript = openai.Audio.transcribe(
                    model="whisper-1",
                    file=f
                )
        return transcript["text"]


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
    print("🎧 최종 STT 출력:", result)
