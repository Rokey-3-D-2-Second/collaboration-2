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

- 교통사고로 인한 사회적 비용이 지속적으로 증가하고 있음(2022년 국가예산의 4.3%).
- 일반 운전자 촬영 사진의 법적·보험적 활용 한계.
- 고속도로 2차 사고의 치사율이 일반 사고보다 7배 이상 높음.
- 신속하지 못한 현장 대응, 비전문가에 의한 부정확한 정보 기록이 2차 피해와 사회적 부담을 증가시킴.

### 해결방안

- TurtleBot 기반 자율주행 로봇 시스템과 서버를 연동.
- 사고 감지, 현장 자동 촬영, 차량 통제, 데이터 실시간 전송을 자동화.
- AI 기반(객체인식, 위치 추정) 사고 감지와 SLAM·Navigation을 통한 정확한 자율 순찰.
- 촬영된 데이터를 바탕으로 자동 보고서(HTML) 생성까지 전 과정 자동화.

### 주요기능

- **순찰 및 사고 감지:** 순찰 경로 내 자율주행, YOLO 기반 객체 인식으로 사고 실시간 감지.
- **사고 현장 자동 촬영:** 사고 감지 시 촬영봇이 현장 이동 후 사진·3D 포인트클라우드 획득.
- **좌표 변환 및 데이터 전송:** 카메라 좌표를 글로벌 좌표로 변환하여 서버에 보고.
- **차량 통제:** 사고 감지 시 경고음 및 안내화면, 통제 종료 후 순찰 재개.
- **자동 리포트 생성:** 사고 기록, 촬영 데이터, 위치 정보로 HTML 보고서 자동화.
- **통합 서버 및 모듈화:** 노드별 통신 구조 설계, 각 단계의 상태 관리(FSM), 실시간 명령 처리 및 테스트 자동화.
- **시스템 신뢰성 강화:** ROS2 QOS, 네트워크 신뢰성 확보, 순찰-감지-촬영-복귀까지 전 과정 완전 자동화

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
