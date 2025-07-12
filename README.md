# 💊 PharmacyBot: Voice-Driven Medicine Recommendation & Robotic Dispensing System

> “AI가 듣고, 이해하고, 직접 약을 꺼내주는 약국 자동화 솔루션”

---

## 📌 Overview

PharmacyBot은 심야, 공휴일 등 **약국 이용이 어려운 시간대에도 의약품을 편리하게 제공**하기 위한 **AI 기반 협동 로봇 시스템**  
음성으로 증상을 말하면 AI가 적절한 약을 추천하고, 로봇팔이 해당 약을 집어 사용자에게 제공

---

## 🧠 Motivation

- **서울시 약국 5,317개 중 심야 운영 약국은 단 38개 (0.7%)**
- 고령화 및 1인 약국 증가 → 약사 부재 시 보완 시스템 필요
- 기존 키오스크 기반 무인 시스템은 디지털 약자에겐 진입 장벽이 높음

---

## 🧬 Hardware, Tools, Environment

- Doosan m0609, RG2 gripper
- ROS 2 Humble
- Python 3.8+
- Ubuntu 22.04

---

## 🧪 Key Technologies

- **Whisper API** for STT
- **LangChain + Vector DB** for context-aware recommendation
- **YOLO v11m** for object detection
- **DRS SDK + ROS2** for robot arm control
- **PyQt GUI** for real-time interaction

---

## 🛠 System Architecture

<img width="1094" height="311" alt="제목 없는 다이어그램-페이지-3 drawio" src="https://github.com/user-attachments/assets/a1364533-56f2-4aef-a0c9-7560709fb5fa" />

---

## 🧩 Core Modules

| Module | Description |
|--------|-------------|
| `voice_input.py` | Wake word 감지, 음성 STT, 증상 파일 저장 |
| `symptom_matcher.py` | 증상 → 약 추천 (LLM + VectorDB 활용) |
| `detector.py` | YOLO v11m 기반 객체 탐지 및 좌표 추출 |
| `robot_arm.py` | 로봇팔 제어 및 약 집기 동작 |
| `pharmacy_manager.py` | 전체 프로세스 제어 및 예외 처리 로직 |
| `gui.py` | 사용자와의 채팅 인터페이스 및 결과 시각화 |

---

## 🌟 Project Highlights

- LLM 기반 약 추천 시스템
- 음성 인터페이스로 디지털 약자 접근성 향상
- YOLO + 로봇팔로 약 자동 제공
- 모듈화된 ROS2 구조 및 GUI 기반 상호작용

---

## 🚨 Exception Handling

| 상황 | 대응 전략 |
|------|------------|
| 음성인식 3회 실패 | STT 시스템 재시작 유도 |
| 약물 탐지 실패 | 탐지 재시도 후 Home 복귀 |
| 로봇팔 그립 실패 | 탐지 루프로 다시 진입 |

---

## 🧪 Installation & Run

```bash
# Clone this repository
git clone https://github.com/Rokey-D-3/pharmacy_bot.git
cd pharmacy_bot

# Launch the system
ros2 launch pharmacy_system.launch.py
```

---

## 📈 Future Work

- 웹 기반 인터페이스 확장
- 약 데이터 DB 연동 및 재고 관리
- 다양한 이미지/라벨 학습 데이터 추가
- 더 복잡한 증상 분석 모델 도입

---

## ✅ Team & Roles

| Name | Role | Email |
|------|------|-------|
| 김승주 | 팀장, 모션 플래닝, 약 데이터 라벨링 | lunaticju@gmail.com |
| 최초인 | 전체 시스템 설계, 음성처리, 서비스/토픽 | choin22222@naver.com |
| 박정하 | 시나리오 통합 테스트, 예외 처리 | park013031@gmail.com |
| 함국성 | LLM + VectorDB, GUI 구현 | gukseong723@gmail.com |
