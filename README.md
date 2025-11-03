# 🦾 DF_Doosan2 – Smart Refueler for Doosan E0509
> “Doosan E0509 + RH-P12-RN(A) + RealSense” 기반 ROS 2 자율 주유 로봇 패키지

---

## 📘 개요
이 프로젝트는 **Doosan E0509 협동로봇**, **RH-P12-RN(A) 그리퍼**, **Intel RealSense D435**를 이용해  
차량을 자동 인식하고 주유구를 열어 연료를 주입하는 **Smart Fuel Robot System**입니다.

Flutter 앱 → FastAPI 서버 → ROS 2 → Doosan 로봇팔까지 연동하여  
결제 → 인식 → 주유 → 복귀까지 완전 자동으로 수행됩니다.

---

## 🧰 하드웨어 및 소프트웨어 스택

| 항목 | 구성 |
|------|------|
| **로봇 팔** | Doosan E0509 |
| **그리퍼** | RH-P12-RN(A) (Modbus RTU 제어) |
| **카메라** | Intel RealSense D435 |
| **운영체제** | Ubuntu 22.04 LTS |
| **ROS 버전** | ROS 2 Humble |
| **언어 / 라이브러리** | Python 3.10 (rclpy, OpenCV, PyRealSense2, Ultralytics YOLOv8) |
| **통합 서버** | FastAPI + Uvicorn (Flutter 앱과 연동) |

---

## 📁 프로젝트 경로

본 프로젝트는 다음 경로에 추가되어야 합니다 👇  
/home/df/xyz_ws/src/doosan-robot2/dsr_example2/dsr_example/


폴더 구조 예시:
dsr_example/
├── fuel_listener_node.py
├── motion_controller.py
├── vision_target_node.py
├── webcam_manager_ros.py
├── realsense_manager_ros.py
├── gripper_drl_controller.py
├── test_task_manager.py
└── smart_refueler_bringup.launch.py


---

## ⚙️ 설치 및 빌드

### 1️⃣ Doosan 공식 패키지 설치
```bash
cd ~/xyz_ws/src
git clone -b humble https://github.com/DoosanRobotics/doosan-robot2.git
cd ~/xyz_ws
rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
colcon build
. install/setup.bash
```

2️⃣ Smart Refueler 코드 추가
```bash
# 본 프로젝트를 dsr_example 폴더 안으로 복사
cp -r ~/Downloads/df_doosan2/* /home/df/xyz_ws/src/doosan-robot2/dsr_example2/dsr_example/

# 빌드
cd ~/xyz_ws
colcon build --packages-select dsr_example
. install/setup.bash
```

🚀 실행 순서
🔹 Step 1 – E0509 로봇 Bringup (실제 하드웨어 모드)
```
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real host:=110.120.1.39 port:=12345 model:=e0509
```

⚙️ mode:=real 옵션으로 실행해야 실제 로봇과 통신하며,
host, port, model 값은 환경에 맞게 수정합니다.

🔹 Step 2 – FastAPI 서버 실행 (Flutter 결제 연동)

Flutter 앱에서 결제 완료 신호를 ROS로 전달하기 위한 FastAPI 서버를 실행합니다.
```
cd ~/xyz_ws/server
uvicorn server:app --host 0.0.0.0 --port 12345 --reload
```
🔹 Step 3 – Smart Refueler 통합 Launch 실행

모든 ROS 노드(비전, 제어, 통신)를 한 번에 실행합니다.
```
ros2 launch dsr_example smart_refueler_bringup.launch.py
```

실행되는 주요 노드:

fuel_listener_node → Flutter 서버에서 주유 명령 수신

vision_target_node → YOLOv8 탐지 및 3D 좌표 변환

webcam_manager_ros, realsense_manager_ros → 카메라 스트림 퍼블리시

motion_controller → Doosan 로봇 제어 및 그리퍼 동작

🧠 시스템 플로우
```
[ Flutter App ]
   ↓ (POST /start_fuel)
[ FastAPI Server (/server) ]
   ↓  → /fuel_task/start 토픽
[ fuel_listener_node ]
   ↓
[ motion_controller ] ← [ vision_target_node ]
   ↓
[ Doosan E0509 + RH-P12-RN(A) Gripper ]
```

✅ 포트폴리오 핵심 포인트

실제 E0509 로봇 하드웨어 기반 자율 주유 로봇 구현

Flutter 앱 ↔ FastAPI ↔ ROS 2 ↔ Doosan 로봇 제어 풀스택 연동

YOLOv8 기반 실시간 차량/주유구 탐지 및 좌표 변환

ROS 2 노드 및 토픽 간 명확한 데이터 플로우 설계

Isaac Sim / Isaac Lab 시뮬레이션 확장 가능 (향후 계획)

🧾 참고 및 라이선스

기반 리포지토리: Doosan Robotics / doosan-robot2

작성자: dongfan
