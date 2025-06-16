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
        """텍스트 명령어에서 타겟별 task_step 시퀀스 추출"""
        response = self.chain.invoke({config.user_input: user_input})
        raw_result = response["text"]
        # print(f"🧠 LLM 응답: {raw_result}")

        # print(f"결과를 파싱합니다.")
        try:
            parts = [p.strip() for p in raw_result.strip().split("/")]
            targets = parts[0].split() if parts[0] else []
            steps_per_target = [p.strip().split() for p in parts[1:]] if len(parts) > 1 else []
        except Exception:
            raise exceptions.VUI_ERROR(401)

        # print(f"🔧 추출된 도구: {targets}")
        # print(f"📍 타겟별 목적지: {steps_per_target}")
        return targets, steps_per_target


#############################
# ✅ 독립 실행 테스트용
#############################
if __name__ == "__main__":
    lc = LangChainModule()
    test_sentence = input("🗣️ 테스트 문장을 입력하세요: ")
    lc.extract(test_sentence)
