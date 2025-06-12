# langchain_module.py

import os
from dotenv import load_dotenv
from langchain_community.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain

from util import config, exceptions

class LangChainModule:
    def __init__(self):
        load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), "../resource/.env"))
        self.api_key = os.getenv("OPENAI_API_KEY")

        self.llm = ChatOpenAI(
            model="gpt-4o",
            temperature=0.5,
            openai_api_key=self.api_key
        )

        prompt_template = config.prompt_template

        self.chain = LLMChain(
            llm=self.llm,
            prompt=PromptTemplate(input_variables=[config.user_input], template=prompt_template)
        )

    def extract(self, user_input: str):
        """텍스트 명령어에서 도구/목적지 추출"""
        response = self.chain.invoke({config.user_input: user_input})
        raw_result = response["text"]
        print(f"[{__name__}] 🧠 LLM 응답: {raw_result}")

        # 결과 파싱
        try:
            targets_raw, task_steps_raw = raw_result.strip().split("/")
            targets = targets_raw.strip().split()
            task_steps = task_steps_raw.strip().split()
        except Exception:
            raise exceptions.VUI_ERROR(401)

        print(f"[{__name__}] 🔧 추출된 도구: {targets}")
        print(f"[{__name__}] 📍 추출된 목적지: {task_steps}")
        return targets, task_steps


#############################
# ✅ 독립 실행 테스트용
#############################
if __name__ == "__main__":
    lc = LangChainModule()
    test_sentence = input("🗣️ 테스트 문장을 입력하세요: ")
    lc.extract(test_sentence)
