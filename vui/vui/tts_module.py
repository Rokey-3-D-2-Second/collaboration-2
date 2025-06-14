from gtts import gTTS
from playsound import playsound
import tempfile

from util import exceptions

class TTSModule:
    def __init__(self):
        pass

    def speak(self, text: str):
        print(f"🗣️ gTTS 변환 중: {text}")
        try:
            tts = gTTS(text=text, lang='ko')  # 한국어
        except Exception:
            raise exceptions.VUI_ERROR(402)
        
        with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as tmp:
            tts.save(tmp.name)
            playsound(tmp.name)

#############################
# ✅ 독립 실행 테스트용
#############################
if __name__ == "__main__":
    tts = TTSModule()
    tts.speak("안녕하세요. 로키입니다. 무엇을 도와드릴까요?")
