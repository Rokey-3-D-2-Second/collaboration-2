# MEDICREW: 중앙공급실 자동화 로봇 서비스
AI(Computer Vision)기반 협동 로봇 작업 어시스턴트 구현 프로젝트

## 프로젝트 개요

- **목표**: 중앙공급실의 수작업 중심 운영으로 인한 인력부담, 위생, 신속대응 한계를 스마트 병원 전환으로 극복
- **주요 기능**: 음성 명령 및 대화형 인터페이스, AI 비전 기반 실물 인식 및 분류, 매니퓰레이터 기반 자동 정리 및 이송
- **사용 장비**: Doosan m0609, RG2 gripper
- **개발 환경**: Ubuntu 22.04, ROS2 Humble
- **주요 기술 스택**: ROS2, MoveIt2(Motion Planning), Octomap
- **기간**: 2025.06.09 ~ 2025.06.20

## 시연 영상

- 영상 링크: [Demo Video](https://youtu.be/V7rXOWxyarg)

<div align="center">

[Demo Video](https://github.com/user-attachments/assets/67010a89-4dae-406c-8c7c-979b4136967a)

<!--
![스크린샷](https://github.com/user-attachments/assets/30322db1-5d32-4324-8b34-3e8028cd11e9)
-->

</div>

## 시스템 아키텍처 및 다이어그램

<div align="center">

![협동-2 시스템 아키텍처](https://github.com/user-attachments/assets/54c450b5-9f4c-405e-8fef-ba0b9490e580)

![협동-2 플로우차트](https://github.com/user-attachments/assets/8f062112-6a72-4422-bca3-e5440636dfb6)

</div>

## 상세 설명

### 문제정의

- 의료현장에서 반복적 업무, 인력 부족, 감염·오염 등 문제에 따른 환자와 의료진의 불편.
- 의료기구의 오염 여부, 물품 배치 등 수작업의 비효율과 인적 오류.
- 로봇 활용 확대를 위한 자연어(음성) 명령 기반의 사용자 친화적 제어 필요.

### 해결방안

- 음성 명령 → LLM 프롬프트 기반 타겟·작업 단계 추출 → 로봇 동작 시퀀스 생성 및 실행 → 음성 피드백의 엔드투엔드 자동화.
- RGB-D 카메라+YOLO+Color Detecting으로 물체 인식 및 오염 여부 자동 판단.
- 고해상도 Octomap 및 장애물 인식, 충돌 민감도/작업 영역 제한 등 로봇 안전성 강화.
- MongoDB 연동 자동 로그 기록, 최근 작업 분석 후 HTML/PDF 리포트 자동화.

### 주요기능

- **음성 명령 해석 및 제어:** LangChain 프롬프트로 타겟·작업 추출 및 예외 입력 처리, 작업 실행 후 음성 피드백.
- **3D 좌표계 변환:** YOLO 객체 검출 → 3D 좌표(카메라→Base) 변환 후 로봇으로 전송.
- **로봇 제어 자동화:** IK 계산, 경로계획(Path Plan), Trajectory 실행.
- **기구 오염 여부 자동 판정:** YOLO+BBox 내 색상 분석, 임계치 기반 자동 오염 감지.
- **안전 제어:** 충돌 민감도, 작업 영역 제한, 외력 감지 기반 긴급정지.
- **고해상도 Octomap 기반 장애물 회피:** 1cm 단위 장애물 감지, MoveIt 연동 경로 계획.
- **작업 로그/리포트 자동화:** MongoDB 로그 저장, DataFrame 시각화, HTML/PDF 리포트 제공.
- **실환경 검증 및 시연:** 실제 병원 시나리오 기반 종합 시연, 사용자 피드백 반영 가능성 검토

## 프로젝트 기여자

- 문승연: opm0508@naver.com
- 이호준: hojun7889@gmail.com
- 지예은: jyebubu@gmail.com
- 홍지원: jw1122.h@gmail.com

## 교육과정 및 참고자료

### 교육과정

<div align="center">

| 주차 | 기간 | 구분 | 강의실 |
| --- | --- | --- | --- |
| <5주차> | 2025.06.09(월) ~ 2025.06.13(금) | 협동-2 | * 별관 : C-2호 |
| <6주차> | 2025.06.16(월) ~ 2025.06.20(금) | 협동-2 | * 별관 : C-2호 |

| 차시 | 학습내용 | 세부 내용 |
| --- | --- | --- |
| 1 | RGB Camera 로봇 통신환경 구축 – 1 | Depth Camera 패키지 설치, Camera Topic, PointCloud with RQt and RViz2, Gripper제어와 Modbus |
| 2 | RGB Camera 로봇 통신환경 구축 – 2  | Depth Camera Calibration(이론 및 개념, Checker Board & Eye-in-Hand Calibration, 3차원 좌표 추정) |
| 3 | Vision AI 모델 기반 로봇 서비스 구축 – 1  | 딥러닝 기반 인공지능 모델 기초(YOLO, Ultralytics Settings, Datasets, Model Training, Inference) |
| 4 | Vision AI 모델 기반 로봇 서비스 구축 – 2 | Doosan ROS2 & DRL 기반 서비스 구축, Object Detection, Pick and Place |
| 5 | Vision AI 모델 기반 로봇 서비스 구축 – 3 | Object Detection → Text 입력 기반 Pick and Place 구 |
| 6 | 음성/언어 모델 기반 로봇 서비스 구축 – 1 | Wakeup Word 모델 및 Voice Recognition API 사용법, STT(Speech To Text), Keywork Extraction |
| 7 | 음성/언어 모델 기반 로봇 서비스 구축 – 2 | LLM(Large Language Model) 소개 및 사용, LangChain 기반 Node 생성 및 ROS2 통신 |
| 8 | AI기반 협동 로봇 작업 어시스턴트 구현 – 1 | 단위 모듈 통합 및 AI기반 협동 로봇 완성 |
| 9 | AI기반 협동 로봇 작업 어시스턴트 구현 – 2 | 시나리오, 시스템 설계, 팀 프로젝트 개발 실습 및 적용 |
| 10 | AI기반 협동 로봇 작업 어시스턴트 구현 – 3 | 팀 프로젝트 시연, 발표 및 피드백, 팀 프로젝트 산출물 정리 |

</div>

### 참고자료

- https://manual.doosanrobotics.com/ko/programming-manual/3.3.0/publish/
- https://moveit.picknik.ai/main/doc/concepts/motion_planning.html
- https://octomap.github.io/
- https://github.com/OctoMap/octomap_mapping/tree/ros2
