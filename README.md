
# Smart Refueler ROS 2 Package
두산 협동로봇과 YOLO 기반 비전을 결합해 차량의 주유구를 자동으로 탐지·접근·주유하는 ROS 2 패키지입니다. Flutter 앱 → FastAPI → ROS 2 노드로 이어지는 풀 파이프라인을 구축해 포트폴리오용 실사용 시나리오를 구현했습니다.

## 프로젝트 한눈에 보기
- **모션 시퀀스 자동화**: 결제 완료와 차량 정지 여부를 확인한 뒤 e0509 협동로봇이 노즐을 잡고 차량 주유구까지 움직이는 전체 FSM을 `motion_controller`가 담당합니다.
- **듀얼 카메라 비전**: USB 웹캠으로 차량 도착을 감지하고, Intel RealSense depth 기반 YOLO 모델로 주유구 위치를 3D 포인트로 산출합니다.
- **DRL 기반 그리퍼 제어**: Modbus 명령을 DRL 스크립트로 로딩해 Doosan 플랜지 시리얼을 통해 전동 그리퍼를 초기화·제어합니다.
- **API 연동**: FastAPI 서버가 `/start_fuel` 토픽을 발행하면 ROS 2 노드가 주문 정보와 연동된 주유 작업을 수행합니다.
- **현장형 안전장치**: 워크스페이스 제한, Z 최소 높이, 힘 안전 감지 등 실제 협동로봇 적용을 고려한 소프트 리밋을 적용했습니다.

## 아키텍처
```
Flutter App ──▶ FastAPI 서버 ──▶ /start_fuel
                                   │
                                   ▼
                        FuelCommandListener (/fuel_task/start)
                                   │
       ┌──────────────┬────────────┴─────────────┐
       ▼              ▼                          ▼
WebcamManager   RealSenseManager         MotionController
  │                  │                      │         │
  │                  └── YOLO (VisionTargetNode) ────┘
  │                           │
  ▼                           ▼
/car_detected        /fuel/object_3d & /fuel/yolo_detections
```

## 제공 노드
| 노드 | 역할 | 주요 인터페이스 |
| --- | --- | --- |
| `motion_controller` | 결제/차량 감지 FSM, 로봇 경로 계획, 그리퍼 제어 | Sub: `/fuel_task/start`, `/car_detected`, `/fuel/yolo_detections`, `/fuel/object_3d`, `/stop_motion`<br>Pub: `/fuel_status`, `/target_direction` |
| `vision_target_node` | 듀얼 YOLO 추론, 2D → 3D 변환, 차량 감지 이벤트 | Sub: `/fuel/webcam_color`, `/fuel/realsense_color`, `/fuel/realsense_depth`<br>Pub: `/fuel/yolo_detections`, `/fuel/object_3d`, `/fuel/image_result`, `/car_detected` |
| `realsense_manager_ros` | RealSense 컬러/깊이 스트림 정렬 및 퍼블리시 | Pub: `/fuel/realsense_color`, `/fuel/realsense_depth` |
| `webcam_manager_ros` | USB 웹캠 프레임 취득 및 퍼블리시 | Pub: `/fuel/webcam_color` |
| `fuel_command_listener` | FastAPI → ROS 브리지, 주문 데이터 전달 | Sub: `/start_fuel` → Pub: `/fuel_task/start` |
| `example_*` | 토픽 테스트 및 로봇 단독 예제 | `example_move`, `example_gripper` 등 |

## 요구 사항
### 하드웨어
- Doosan 협동로봇 e0509 (또는 동일한 DOF/그리퍼 구성을 갖춘 모델)
- Modbus 제어 전동 그리퍼 (Doosan 플랜지 시리얼 사용)
- Intel RealSense (D435 시리즈 권장) 및 USB 웹캠
- NVIDIA GPU 권장 (YOLO 추론 가속)

### 소프트웨어
- Ubuntu 22.04 + ROS 2 Humble
- Doosan ROS 2 드라이버 패키지 (`dsr_common2`, `dsr_bringup2`)
- Python 3.10, `ultralytics`, `opencv-python`, `pyrealsense2`, `cv_bridge`
- RealSense SDK 및 udev 설정

필수 환경 변수:

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
>>>>>>> 8ecc3ea510061cd8c5caf9b9216a6d90d6552d0b
```bash
cd ~/xyz_ws/src
git clone -b humble https://github.com/DoosanRobotics/doosan-robot2.git
cd ~/xyz_ws
rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
colcon build
. install/setup.bash
```


## 설치 및 빌드
```bash
# 워크스페이스 구성
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repository> dsr_example

# 의존 패키지 설치 (예시)
sudo apt install ros-humble-cv-bridge ros-humble-launch ros-humble-launch-ros
pip install ultralytics opencv-python pyrealsense2

# 빌드
cd ~/ros2_ws
colcon build --packages-select dsr_example
source install/setup.bash
```

## 실행 가이드
1. **Doosan 로봇 드라이버 기동**
   ```bash
   ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
     mode:=virtual host:=127.0.0.1 port:=12345 model:=e0509
   ```
2. **스마트 주유 파이프라인 실행**
   ```bash
   ros2 launch dsr_example smart_refueler_bringup.launch.py
   ```
   위 런치 파일은 웹캠, RealSense, YOLO 비전, 모션 컨트롤러, FastAPI 브리지를 순차적으로 실행합니다.
3. **주유 명령 발행 (예시)**
   ```bash
   ros2 topic pub --once /start_fuel std_msgs/String \
     '{"order_id":"ORDER-001","fuelType":"diesel","amount":50000}'
   ```
   - `FuelCommandListener`가 해당 메시지를 `/fuel_task/start`로 전달하고, `motion_controller`가 주유 시퀀스를 시작합니다.

## 개발 기여 포인트
- **통합 모션 컨트롤 노드**: 기존 연속 스크립트를 ROS 2 서비스/토픽 기반 FSM으로 리팩터링하고, 안전 속도·워크스페이스 제한·힘 감지를 적용했습니다.
- **YOLO 파이프라인 재설계**: 웹캠과 RealSense에 서로 다른 가중치를 적용해 차량-주유구를 분리 감지하고, depth 맵을 기반으로 TCP에서 사용 가능한 3D 목표 좌표를 산출했습니다.
- **그리퍼 DRL 스크립트화**: Modbus 초기화·이동·피드백 코드를 DRL 태스크로 캡슐화하여 ROS 노드에서 동기식으로 호출 가능하도록 구현했습니다.
- **API-ROS 브리지**: FastAPI 백엔드와 ROS 사이의 `String JSON` 프로토콜을 정의하고, 결제/차량 감지 조건이 충족될 때만 모션을 개시하도록 보호 로직을 추가했습니다.

## 데모 영상
- [Motion sequence demo](https://github.com/user-attachments/assets/19cf76fe-81c6-442c-90ae-ef4b49af17c0)
- [Single robot example](https://github.com/user-attachments/assets/c9b19447-0f7e-42b2-824a-a744db24079e)

## 프로젝트 구조
```
dsr_example/
├── fuel_listener_node.py      # FastAPI → ROS 브리지
├── motion_controller.py       # FSM + 로봇/그리퍼 제어
├── vision_target_node.py      # 듀얼 YOLO + 3D 포인트 산출
├── realsense_manager_ros.py   # RealSense 스트림 관리
├── webcam_manager_ros.py      # USB 웹캠 스트림 관리
├── weights/                   # YOLO 가중치 (.pt)
└── simple/                    # 로봇 동작 예제 스크립트
```

## 향후 개선 아이디어
- TF 기반 좌표 변환 및 핸드-아이 캘리브레이션 자동화
- 강인한 힘 제어를 위한 실시간 임피던스/어드미턴스 제어기 연동
- 주유 단계 모니터링 대시보드 및 알람 시스템
- YOLO 라벨 세분화 및 다중 차량 대응 로직

## 라이선스
이 프로젝트는 BSD License를 따릅니다. 상세한 조건은 `LICENSE` 파일 또는 Doosan ROS 2 패키지의 라이선스를 참고하세요.

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
>>>>>>> 8ecc3ea510061cd8c5caf9b9216a6d90d6552d0b
