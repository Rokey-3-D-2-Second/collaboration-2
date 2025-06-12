from gtts import gTTS
from playsound import playsound
import tempfile

class TTS:
    def __init__(self):
        pass

    def speak(self, text: str):
        print(f"🗣️ gTTS 변환 중: {text}")
        tts = gTTS(text=text, lang='ko')  # 한국어
        with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as tmp:
            tts.save(tmp.name)
            playsound(tmp.name)


if __name__ == "__main__":
    tts = TTS()
    tts.speak("안녕하세요. 로키입니다. 무엇을 도와드릴까요?")
