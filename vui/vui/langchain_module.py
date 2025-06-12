# langchain_module.py

import os
from dotenv import load_dotenv
from langchain_community.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain


class LangChainModule:
    def __init__(self):
        load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), "../resource/.env"))
        self.api_key = os.getenv("OPENAI_API_KEY")

        self.llm = ChatOpenAI(
            model="gpt-4o",
            temperature=0.5,
            openai_api_key=self.api_key
        )

        prompt_template = """
        당신은 사용자의 문장에서 특정 도구와 목적지를 추출해야 합니다.

        <목표>
        - 문장에서 다음 리스트에 포함된 도구를 최대한 정확히 추출하세요.
        - 문장에 등장하는 도구의 목적지(어디로 옮기라고 했는지)도 함께 추출하세요.

        <도구 리스트>
        - hammer, screwdriver, wrench, pos1, pos2, pos3

        <출력 형식>
        - 다음 형식을 반드시 따르세요: [도구1 도구2 ... / pos1 pos2 ...]
        - 도구와 위치는 각각 공백으로 구분
        - 도구가 없으면 앞쪽은 공백 없이 비우고, 목적지가 없으면 '/' 뒤는 공백 없이 비웁니다.
        - 도구와 목적지의 순서는 등장 순서를 따릅니다.
        <특수 규칙>
        - 명확한 도구 명칭이 없지만 문맥상 유추 가능한 경우(예: "못 박는 것" → hammer)는 리스트 내 항목으로 최대한 추론해 반환하세요.
        - 다수의 도구와 목적지가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.
        <예시>
        - 입력: "hammer를 pos1에 가져다 놔"  
        출력: hammer / pos1

        - 입력: "왼쪽에 있는 해머와 wrench를 pos1에 넣어줘"  
        출력: hammer wrench / pos1

        - 입력: "왼쪽에 있는 hammer를줘"  
        출력: hammer /

        - 입력: "왼쪽에 있는 못 박을 수 있는것을 줘"  
        출력: hammer /

        - 입력: "hammer는 pos2에 두고 screwdriver는 pos1에 둬"  
        출력: hammer screwdriver / pos2 pos1

        <사용자 입력>
        "{user_input}"
        """

        self.chain = LLMChain(
            llm=self.llm,
            prompt=PromptTemplate(input_variables=["user_input"], template=prompt_template)
        )

    def extract(self, user_input: str):
        """텍스트 명령어에서 도구/목적지 추출"""
        response = self.chain.invoke({"user_input": user_input})
        raw_result = response["text"]
        print(f"🧠 LLM 응답: {raw_result}")

        # 결과 파싱
        try:
            objects_raw, targets_raw = raw_result.strip().split("/")
            objects = objects_raw.strip().split()
            targets = targets_raw.strip().split()
        except ValueError:
            print("⚠️ 잘못된 출력 형식입니다. '/' 포함 여부를 확인하세요.")
            return [], []

        print(f"🔧 추출된 도구: {objects}")
        print(f"📍 추출된 목적지: {targets}")
        return objects, targets


#############################
# ✅ 독립 실행 테스트용
#############################
if __name__ == "__main__":
    lc = LangChainModule()
    test_sentence = input("🗣️ 테스트 문장을 입력하세요: ")
    lc.extract(test_sentence)
