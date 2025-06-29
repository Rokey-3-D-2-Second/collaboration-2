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

![스크린샷](https://github.com/user-attachments/assets/30322db1-5d32-4324-8b34-3e8028cd11e9)

</div>

## 시스템 아키텍처 및 다이어그램

<div align="center">

![협동-2 시스템 아키텍처](https://github.com/user-attachments/assets/54c450b5-9f4c-405e-8fef-ba0b9490e580)

![협동-2 플로우차트](https://github.com/user-attachments/assets/8f062112-6a72-4422-bca3-e5440636dfb6)

</div>

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
