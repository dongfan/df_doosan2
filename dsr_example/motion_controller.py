#!/usr/bin/env python3
"""
MotionController (combined)
- FuelTaskManager 기능(결제 신호/FSM/차량 감지 연동)과 MotionController(로봇/그리퍼/안전이동)를 하나의 노드로 통합
- 외부(Flutter→FastAPI)에서는 기존처럼 /fuel_task/start 토픽으로 JSON을 퍼블리시하면 됩니다.

주요 토픽/서비스
- sub  : /fuel_task/start (String JSON: {orderId, fuelType, amount})
- sub  : /car_detected (String "detected")
- sub  : /fuel/yolo_detections (String JSON array)  # YOLO 결과(웹캠/리얼센스) 통합 입력
- sub  : /fuel/object_3d (PointStamped, camera frame)  # 리얼센스 기반 3D 타깃 포인트
- sub  : /stop_motion (Bool)
- pub  : /fuel_status (String: idle/progress/done/error)
- pub  : /target_direction (Float32)  # (선택) 노즐↔주유구 방향 보조
- srv  : /motion_controller/orient_negative_y (Trigger)

설정 포인트
- CAMERA_OFFSET_TCP_Z_M = +0.05  # 카메라가 TCP보다 5 cm 위
- ORIENT_PRESET_POSJ : 툴을 -Y(바닥 방향)으로 보는 자세 프리셋

주의
- 실제 환경에 맞게 워크스페이스/최소Z/속도·가속도 상한 등을 조정하세요.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger

import numpy as np
import json
import time
# import threading
from collections import deque

import DR_init
from dsr_example.gripper_drl_controller import GripperController

# ─────────────────────────────────────────────────────────────
# Doosan 기본 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ─────────────────────────────────────────────────────────────
# 안전/동작 파라미터
TARGET_LABEL = "green_car"      # YOLO 허용 라벨(예: 자동차)
CAP_LABEL = "Cap"               # YOLO 허용 라벨(예: 자동차)
NOZZLE_LABEL = "Nozzle"         # YOLO 허용 라벨(예: 자동차)
LABEL_TIMEOUT_SEC = 3.0          # 허용 라벨 감지 유지 시간
V_MAX = 60                       # 이동 속도 상한 (Doosan 단위)
A_MAX = 60                       # 가속도 상한
PRE_UP_MM = 120.0                # 접근 전 위로 확보할 높이
STANDOFF_MM = 120.0              # 목표 지점 위에서 멈출 여유
MIN_Z_MM = 400.0                 # 절대 최소 Z (충돌 방지)
WS_XY_MM = 800.0                 # XY 워크스페이스 절대 한계(±)

CAMERA_OFFSET_TCP_Z_M = 0.05     # 카메라가 TCP보다 +5 cm (위)
ORIENT_PRESET_POSJ = (20, 35, 105, 105, -90, 50)  # 바닥(-Y) 방향 프리셋
ORIENT_POSJ_POS_XL = (27, -6, 100, -90, 26, -180) # 바닥(+X) 방향 프리셋 : 휘발유
ORIENT_POSJ_POS_XR = (-30, -10, 100, 85, 35, 0) # 바닥(+X) 방향 프리셋 : 경유

# ─────────────────────────────────────────────────────────────
class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        self.get_logger().info("🤖 MotionController (combined) starting...")
        self.coord_buffer = deque(maxlen=10)  # 최근 10개 좌표 유지
        self.last_valid_coord = None           # 최근 안정 좌표

        # FSM/주문 상태
        self.current_state = "IDLE"  # IDLE → PROGRESS → DONE
        self.order_id = None
        self.fuel_type = None
        self.amount = 0.0

        # 감지 상태
        self.last_label_ts = 0.0
        self.allowed_label = CAP_LABEL
        self.last_car_detected_event = False

        # 이동 상태
        self.is_busy = False
        self.force_triggered = False

        self.last_base_coords = None
        self.last_detected_point = None

        # 그리퍼 초기화
        self._init_gripper_and_home()

        # 구독/퍼블리셔
        self.sub_start = self.create_subscription(String, '/fuel_task/start', self.on_task_start, 10)
        self.sub_car_detected = self.create_subscription(String, '/car_detected', self.on_car_detected, 10)
        self.sub_yolo = self.create_subscription(String, '/fuel/yolo_detections', self.on_detections, 10)
        self.sub_obj3d = self.create_subscription(PointStamped, '/fuel/object_3d', self.object_callback, 10)
        self.sub_stop = self.create_subscription(Bool, '/stop_motion', self.on_stop_signal, 10)
        self.sub_webcam = self.create_subscription(String, '/fuel/webcam_detections', self.on_webcam_detections, 10)
        self.sub_realsense = self.create_subscription(String, '/fuel/realsense_detections', self.on_realsense_detections, 10)

        self.pub_status = self.create_publisher(String, '/fuel_status', 10)
        self.pub_target_dir = self.create_publisher(Float32, '/target_direction', 10)

        # 힘/토크(있으면 사용)
        try:
            from dsr_msgs2.msg import ForceTorque
            self.sub_force = self.create_subscription(ForceTorque, f'/{ROBOT_ID}/force_torque_raw', self.on_force, 10)
        except Exception:
            self.get_logger().warn("⚠️ Force topic type not available; skip force protection.")

        self.get_logger().info("✅ Subscriptions ready: /fuel_task/start, /car_detected, /fuel/yolo_detections, /fuel/object_3d, /stop_motion")

        # 기본 Hand–Eye 행렬 설정 (fuel_cap 모드 기본)
        self.mode = "fuel_cap"
        self.T_tcp2cam = self._make_tcp2cam_matrix(self.mode)

        self.timer = self.create_timer(0.5, self.control_loop)

    # ─────────────────────────────────────────────────────────
    # 초기화 및 유틸
    def _init_gripper_and_home(self):
        try:
            from DSR_ROBOT2 import wait, movej
            self.gripper = GripperController(node=self, namespace=ROBOT_ID)
            if not self.gripper.initialize():
                raise RuntimeError("Gripper initialization failed")

            self.get_logger().info("그리퍼 초기 위치 오픈")
            self.gripper.move(0)
            wait(1.5)

            self.get_logger().info("홈 자세 이동")
            movej([0, 0, 90, 0, 90, 0], 60, 60)
            wait(1.5)

            self.gripper.move(0)
            wait(1.5)
        except Exception as e:
            self.get_logger().error(f"❌ Gripper/Init error: {e}")
            raise

    # ─────────────────────────────────────────────────────────
    # 각 카메라별 감지
    def on_webcam_detections(self, msg: String):
        """웹캠 YOLO 결과 — 탐색 중단 없이 로그만 출력"""
        try:
            dets = json.loads(msg.data)
            labels = [d.get('cls') for d in dets if 'cls' in d]
            if TARGET_LABEL in labels:
                self.get_logger().info(f"👁️ [Webcam YOLO] {TARGET_LABEL} 감지됨 — 탐색 유지")
        except Exception as e:
            self.get_logger().warn(f"웹캠 YOLO 파싱 오류: {e}")

    def on_realsense_detections(self, msg: String):
        """리얼센스 YOLO 결과 — 탐색 중단 트리거 (이동은 control_loop에서 수행)"""
        try:
            dets = json.loads(msg.data)
            labels = [d.get('cls') for d in dets if 'cls' in d]

            if CAP_LABEL in labels:
                # 1️⃣ 탐색 중이면 멈춤
                if getattr(self, "searching", False):
                    self.get_logger().info(f"✅ [Realsense YOLO] {CAP_LABEL} 감지됨 — 탐색 중단")
                    self.searching = False
                    self.detected_cap_once = True
                    self.last_label_ts = time.time()

                # 2️⃣ 좌표 갱신은 object_callback에서 처리하므로 여기서는 안 건드림
                else:
                    if getattr(self, "detected_cap_once", False):
                        return

        except Exception as e:
            self.get_logger().warn(f"리얼센스 YOLO 파싱 오류: {e}")

    # 결제/시작 신호 & 차량 감지 FSM
    def on_task_start(self, msg: String):
        """Flutter/서버에서 결제 완료 후 주유 시작 신호(JSON)를 받는다."""
        try:
            payload = json.loads(msg.data)
            self.order_id = payload.get("order_id", "UNKNOWN")
            self.payment_confirmed = True
            self.get_logger().info(f"💳 결제 완료 수신 (order_id={self.order_id})")
        except Exception as e:
            self.get_logger().error(f"❌ 결제 메시지 파싱 실패: {e}")
            return

    def on_car_detected(self, msg: String):
        if not msg.data:
            return  # 차량이 사라졌으면 무시

        # 상태 저장
        self.detected_car = True
        self.get_logger().info("🚗 차량 감지됨")

        # 조건 확인
        if getattr(self, "payment_confirmed", False) and self.current_state != "IN_PROGRESS":
            self.get_logger().info("✅ 차량 감지 + 결제 완료 → 주유 시퀀스 시작")
            self.start_fueling_sequence()
        else:
            if not getattr(self, "payment_confirmed", False):
                self.get_logger().info("💤 결제 대기 중 (아직 결제 완료 신호 없음)")
            elif self.current_state == "IN_PROGRESS":
                self.get_logger().info("⚙️ 이미 주유 시퀀스 진행 중")

    def on_detections(self, msg: String):
        """YOLO 결과 JSON에서 허용 라벨 감지 시 타임스탬프 갱신"""
        try:
            dets = json.loads(msg.data)
            labels = [d.get('cls') for d in dets if 'cls' in d]
            if self.allowed_label in labels:
                self.last_label_ts = time.time()
        except Exception as e:
            self.get_logger().warn(f"parse det error: {e}")

    def start_fueling_sequence(self):
        if self.current_state == "IN_PROGRESS":
            return
        
        self.current_state = "IN_PROGRESS"
        self.get_logger().info("🚀 주유 시퀀스 시작: orient_negative_y() → 탐색 시작")

        # 1️⃣ 툴 -Y(바닥) 방향 회전
        self.orient_negative_y()
        self.set_handeye_mode("fuel_cap")
        self.search_for_object()

        # 2️⃣ 유종별 +X 방향 전환
        self.orient_positive_x()

        # 3️⃣ 객체 탐색 시작
        self.search_for_object()

    def search_for_object(self):
        """객체가 인식될 때까지 상하좌우로 10cm씩 탐색 이동하는 함수"""
        from DSR_ROBOT2 import movel, wait, DR_MV_MOD_REL
        from DR_common2 import posx as dr_posx
        import time

        step_mm = 50  # 5 cm
        directions = [
            (0, 0, step_mm, 0, 0, 0),   # 위로 이동
            (0, 0, -step_mm, 0, 0, 0),  # 아래로 이동
            (step_mm, 0, 0, 0, 0, 0),   # 오른쪽으로 이동
            (-step_mm, 0, 0, 0, 0, 0)   # 왼쪽으로 이동
        ]

        # 2️⃣ 3초 동안 객체 감지 확인 루프
        check_duration = 3.0
        check_start = time.time()
        self.searching = True
        start_time = time.time()
        timeout_sec = 60.0  # 탐색 제한시간 (초)
        self.get_logger().info(f"🔎 객체 탐색 시작 (최대 {timeout_sec:.0f}초 제한)")
        
        while rclpy.ok():
            for move_dir in directions:
                if not self.searching:
                    # self.get_logger().info("🛑 탐색 중단 (object_callback에서 종료)")
                    break

                # 1️⃣ 이동
                try:
                    movel(dr_posx(*move_dir), v=20, a=20, mod=DR_MV_MOD_REL)
                    wait(0.5)
                except Exception as e:
                    self.get_logger().warn(f"⚠️ 탐색 이동 실패: {e}")

                # 2️⃣ 3초간 감지 확인
                check_duration = 3.0
                check_start = time.time()
                while time.time() - check_start < check_duration:
                    rclpy.spin_once(self, timeout_sec=0.2)
                    age = time.time() - self.last_label_ts
                    if age <= LABEL_TIMEOUT_SEC:
                        self.get_logger().info(f"✅ 감지됨(age={age:.2f}s) → 탐색 종료")
                        self.searching = False
                        return

                # 2️⃣ 시간 제한 체크
                elapsed = time.time() - start_time
                if elapsed > timeout_sec:
                    self.get_logger().warn("⏰ 탐색 제한시간 초과 → 탐색 중단")
                    self.searching = False
                    return
                
        self.searching = False
        self.get_logger().info("🔁 탐색 루프 종료")

    # ───────────────────────── 좌표 변환 Hand-Eye ────────────────────────────────
    def _make_tcp2cam_matrix(self, mode: str):
        """작업 단계(mode)에 따라 Hand–Eye 행렬을 설정"""
        T = np.eye(4)
        if mode == "fuel_cap":
            T[:3, :3] = np.array([[1,0,0],[0,0,-1],[0,1,0]])
            T[:3, 3] = [0, 0, CAMERA_OFFSET_TCP_Z_M]
        elif mode == "nozzle":
            T[:3, :3] = np.array([[0,0,1],[0,1,0],[-1,0,0]])
            T[:3, 3] = [0, 0, CAMERA_OFFSET_TCP_Z_M]
        else:
            T[:3, :3] = np.eye(3)
            T[:3, 3] = [0, 0, CAMERA_OFFSET_TCP_Z_M]
        return T

    def set_handeye_mode(self, mode: str):
        """주유 모드 변경 (fuel_cap / nozzle / idle)"""
        if mode not in ["fuel_cap", "nozzle", "idle"]:
            self.get_logger().warn(f"⚠️ Unknown hand-eye mode: {mode}")
            return
        self.mode = mode
        self.T_tcp2cam = self._make_tcp2cam_matrix(mode)
        self.get_logger().info(f"🔁 Hand–Eye 모드 변경: {mode}")

    def pose_to_matrix(self, pose):
        if isinstance(pose, (list, tuple)) and isinstance(pose[0], (list, tuple)):
            pose = pose[0]
        if len(pose) < 6:
            raise ValueError(f"Invalid pose length: {len(pose)} (need ≥6)")
        
        x, y, z, rx, ry, rz = pose
        rx, ry, rz = np.deg2rad([rx, ry, rz])
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(rx), -np.sin(rx)],
                       [0, np.sin(rx), np.cos(rx)]])
        Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                       [0, 1, 0],
                       [-np.sin(ry), 0, np.cos(ry)]])
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                       [np.sin(rz), np.cos(rz), 0],
                       [0, 0, 1]])
        R = Rz @ Ry @ Rx
        T = np.eye(4)
        T[:3,:3] = R
        T[:3,3] = [x/1000.0, y/1000.0, z/1000.0]
        return T
    
    # ──────────────────── 좌표 noise 제거용 ─────────────────────────
    def smooth_coordinates(self, Xb, Yb, Zb):
        """최근 좌표 평균을 통한 이동평균 필터"""
        self.coord_buffer.append((Xb, Yb, Zb))
        if len(self.coord_buffer) < 3:
            return Xb, Yb, Zb  # 초기엔 필터 적용 X
        avg = np.mean(self.coord_buffer, axis=0)
        return avg[0], avg[1], avg[2]
    
    def filter_jump(self, Xb, Yb, Zb, threshold=0.05):
        """좌표 점프 방지: 이전 좌표 대비 급격한 변화 제거"""
        if self.last_valid_coord is None:
            self.last_valid_coord = (Xb, Yb, Zb)
            return Xb, Yb, Zb

        Xp, Yp, Zp = self.last_valid_coord
        if (abs(Xb - Xp) > threshold or
            abs(Yb - Yp) > threshold or
            abs(Zb - Zp) > threshold):
            self.get_logger().warn("⚠️ 좌표 점프 감지 → 이전 좌표 유지")
            return Xp, Yp, Zp

        self.last_valid_coord = (Xb, Yb, Zb)
        return Xb, Yb, Zb
    
    # ────────────────────────────────────────────────────────────────
    # 3D 타깃 좌표 수신 → Base 변환 → 안전 이동
    def object_callback(self, msg: PointStamped):
        """YOLO 3D 포인트 수신 → Base 좌표 변환 및 저장"""
        from DSR_ROBOT2 import get_current_posx

        self.last_detected_point = msg
        try:
            Xc, Yc, Zc = msg.point.x, msg.point.y, msg.point.z
            pose = get_current_posx()[0][:6]
            self.get_logger().info(f"DEBUG pose_raw={pose}")
            T_base2tcp = self.pose_to_matrix(pose)
            T_base2cam = T_base2tcp @ self.T_tcp2cam
            cam_point = np.array([[Xc], [Yc], [Zc], [1]])
            base_point = T_base2cam @ cam_point
            Xb, Yb, Zb = base_point[:3, 0]
            Xb, Yb, Zb = self.smooth_coordinates(Xb, Yb, Zb)
            Xb, Yb, Zb = self.filter_jump(Xb, Yb, Zb)
            self.last_base_coords = (Xb, Yb, Zb)
            self.get_logger().info(f"📍 감지 좌표(Base): X={Xb:.3f} Y={Yb:.3f} Z={Zb:.3f}")
        
            # 감지 성공 → 탐색 중단 및 이동 준비
            if getattr(self, "searching", False):
                self.get_logger().info("🛑 감지됨 → 탐색 종료 후 이동 준비")
                self.searching = False
                self.ready_to_move = True

        except Exception as e:
            self.get_logger().warn(f"⚠️ 좌표 변환 실패, 재시도 예정: {e}")
            self.ready_to_move = False
            self.error_retrying = True
            # 3초 후 탐색 재개
            self.create_timer(3.0, self.restart_search)

    def restart_search(self):
        """좌표 변환 실패 시 탐색 재개"""
        if not getattr(self, "searching", False):
            self.get_logger().info("🔄 재탐색 재시작")
            self.searching = True
            self.search_for_object()

    def control_loop(self):
        if not self.last_base_coords or self.is_busy or self.searching:
            return

        Xb, Yb, Zb = self.last_base_coords
        self.is_busy = True

        try:
            from DSR_ROBOT2 import movel, wait, get_current_posx, DR_MV_MOD_ABS, DR_MV_MOD_REL
            from DR_common2 import posx
            pose = get_current_posx()[0][:6]
            # 공통 접근 동작
            hold_distance_mm = 0

            # ✅ 단계별 동작 분리
            if self.mode == "fuel_cap":
                # 주유구 쪽으로 접근
                hold_distance_mm = 170
                target = posx(Xb*1000, Yb*1000 + hold_distance_mm, Zb*1000, pose[3], pose[4], pose[5])
                self.get_logger().info(f"DEBUG pose={target}")
                movel(target, v=30, a=30, mod=DR_MV_MOD_ABS)
                wait(2)

                movel(posx(0, 0, 0, 0, 45, 0), v=50, a=50, mod=DR_MV_MOD_REL)
                wait(1.5)
                self.rotate_grip(2, True)

            elif self.mode == "nozzle":
                # 주유건 쪽으로 접근
                hold_distance_mm = 80
                target = posx(Xb*1000 - hold_distance_mm, Yb*1000, Zb*1000, pose[3], pose[4], pose[5])
                self.get_logger().info(f"DEBUG pose={target}")
                movel(target, v=30, a=30, mod=DR_MV_MOD_ABS)
                wait(2)
                self.gripper.move(600)

        except Exception as e:
            self.get_logger().error(f"❌ 이동 실패: {e}")
        finally:
            self.is_busy = False

    def approach_target(self, Xb, Yb, Zb, hold_distance_mm):
        from DSR_ROBOT2 import movel, wait, get_current_posx, DR_MV_MOD_ABS
        from DR_common2 import posx
        pose = get_current_posx()[0][:6]
        
        # ① 현재 포즈 확인
        curr_x, curr_y, curr_z = pose[0:3]

        # ② 기본 목표 설정
        target_x, target_y, target_z = Xb*1000, Yb*1000, Zb*1000

        # ③ 모드별 축 기준으로 접근 거리 조정
        if self.mode == "fuel_cap":
            # -Y 방향으로 hold_distance_mm 만큼 떨어지기
            target_y = curr_y + hold_distance_mm
            self.get_logger().info(f"🧭 -Y축 기준 접근 (fuel_cap) : hold={hold_distance_mm}mm")

        elif self.mode == "nozzle":
            # +X 방향으로 hold_distance_mm 만큼 떨어지기
            target_x = curr_x - hold_distance_mm
            self.get_logger().info(f"🧭 +X축 기준 접근 (nozzle) : hold={hold_distance_mm}mm")

        else:
            # 기본적으로 Z축 접근 유지
            depth_diff = (Zb * 1000) - curr_z
            target_z = curr_z + np.sign(depth_diff) * max(abs(depth_diff) - hold_distance_mm, 0)
            self.get_logger().info(f"🧭 Z축 기준 접근 (기본) : hold={hold_distance_mm}mm")

        # ④ 이동 실행
        target = posx(target_x, target_y, target_z, pose[3], pose[4], pose[5])
        movel(target, v=30, a=30, mod=DR_MV_MOD_ABS)
        wait(1.5)
    # ─────────────────────────────────────────────────────────
    # 주유 완료
    def finish_fueling(self):
        """주유 완료 처리 (그리퍼 동작 + REST 전송 + 상태 갱신)"""
        self.current_state = "DONE"

        # ② REST 서버로 주유 완료 신호 전송
        try:
            import requests
            payload = {"order_id": getattr(self, "order_id", "UNKNOWN"), "status": "done"}
            url = "http://localhost:8000/fuel/complete"  # 🔧 필요시 서버 IP 변경
            response = requests.post(url, json=payload, timeout=3)
            if response.status_code == 200:
                self.get_logger().info(f"🌐 REST 전송 성공: {response.text}")
            else:
                self.get_logger().warn(f"⚠️ REST 응답 코드: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"❌ REST 전송 실패: {e}")

        # ③ 로그 및 상태 출력
        self.get_logger().info("🏁 주유 프로세스 완료 (FSM: DONE)")

    # ─────────────────────────────────────────────────────────
    # 힘/충돌 보호
    def on_force(self, msg):
        if self.force_triggered:
            return
        Fx, Fy, Fz = msg.fx, msg.fy, msg.fz
        total = (Fx**2 + Fy**2 + Fz**2) ** 0.5
        if total > 15.0:
            self.force_triggered = True
            self.get_logger().warn(f"⚠️ Collision detected! F={total:.1f}N → stop & retreat")
            self.hard_stop_and_release()
            self.force_triggered = False

    def hard_stop_and_release(self):
        try:
            from DSR_ROBOT2 import move_stop, movel, DR_MV_MOD_REL, DR_TOOL
            from DR_common2 import posx as dr_posx
            move_stop()
            rel = dr_posx(0, 0, 10, 0, 0, 0)
            movel(rel, v=20, a=20, mod=DR_MV_MOD_REL, ref=DR_TOOL)
            self.get_logger().info("🛑 Stopped & retreated (tool Z+10mm)")
        except Exception as e:
            self.get_logger().warn(f"Stop/retreat failed: {e}")

    # 주유구를 오픈하기 위해 그리퍼를 회전시키는 함수
    def rotate_grip(self, cnt: int, b_open: bool = True):
        from DSR_ROBOT2 import (amovel, DR_MV_MOD_REL,
            movel, movej, wait)
        from DR_common2 import posx, posj
        count = 0
        open_angle = -120 if b_open else 120

        if b_open:
            while count < cnt :
                self.gripper.move(580)
                wait(1.5)
                
                movej(posj(0, 0, 0, 0, 0, open_angle), v=120, a=120, mod=DR_MV_MOD_REL)
                wait(1.0)
                count = count + 1

                if count < cnt:
                    self.gripper.move(440)
                    wait(1.5)
                    movej(posj(0, 0, 0, 0, 0, -open_angle), v=120, a=120, mod=DR_MV_MOD_REL)
                    wait(1.0)

    # ─────────────────────────────────────────────────────────
    def orient_positive_x(self):
        from DSR_ROBOT2 import movej, wait, DR_MV_MOD_ABS
        from DR_common2 import posj
        # 유종 상태 확인
        fuel_type = getattr(self, "fuel_type", "").lower()

        # 휘발유 → XL / 경유 → XR
        if "gas" in fuel_type or "휘발" in fuel_type:
            target_pose = posj(*ORIENT_POSJ_POS_XL)
            label = "휘발유(+XL)"
        elif "diesel" in fuel_type or "경유" in fuel_type:
            target_pose = posj(*ORIENT_POSJ_POS_XR)
            label = "경유(+XR)"
        else:
            # 기본은 XL로 설정
            target_pose = posj(*ORIENT_POSJ_POS_XL)
            label = "기본(+XL, 유종 미지정)"

        self.get_logger().info(f"🧭 툴을 +X 방향으로 회전 중… ({label})")
        try:
            movej(target_pose, v=50, a=50, mod=DR_MV_MOD_ABS)
            wait(2)
            self.get_logger().info(f"✅ 툴 방향 전환 완료 ({label})")
        except Exception as e:
            self.get_logger().error(f"❌ +X 방향 전환 실패 ({label}): {e}")

    def orient_negative_y(self):
        from DSR_ROBOT2 import movej, wait, DR_MV_MOD_ABS
        from DR_common2 import posj
        self.get_logger().info("🧭 툴을 -Y(바닥) 방향으로 회전 중…")
        target_pose = posj(*ORIENT_PRESET_POSJ)
        movej(target_pose, v=50, a=50, mod=DR_MV_MOD_ABS)
        wait(2)
        self.get_logger().info("✅ 툴 방향 전환 완료 (-Y)")

    # ─────────────────────────────────────────────────────────
    # 비상 정지
    def on_stop_signal(self, msg: Bool):
        if msg.data:
            self.hard_stop_and_release()

    # ─────────────────────────────────────────────────────────
    # 정리
    def terminate_gripper(self):
        try:
            if hasattr(self, 'gripper') and self.gripper:
                self.gripper.shutdown()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)

    # DSR 초기 노드 선언(권장 순서): 별도 노드 등록
    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    node = MotionController()
    node.get_logger().info("✅ Hand–Eye 멀티모드 버전 실행 중. set_handeye_mode('fuel_cap'|'nozzle'|'idle')로 모드 전환 가능.")
    node.orient_negative_y()      # -Y 방향 전환
    node.set_handeye_mode("fuel_cap")
    node.search_for_object()      # 바로 탐색 시작
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 MotionController stopped.")
        node.terminate_gripper()
    finally:
        node.terminate_gripper()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
