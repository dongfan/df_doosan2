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
LABEL_TIMEOUT_SEC = 1.0          # 허용 라벨 감지 유지 시간
V_MAX = 60                       # 이동 속도 상한 (Doosan 단위)
A_MAX = 60                       # 가속도 상한
PRE_UP_MM = 120.0                # 접근 전 위로 확보할 높이
STANDOFF_MM = 120.0              # 목표 지점 위에서 멈출 여유
MIN_Z_MM = 400.0                 # 절대 최소 Z (충돌 방지)
WS_XY_MM = 800.0                 # XY 워크스페이스 절대 한계(±)

CAMERA_OFFSET_TCP_Z_M = 0.05     # 카메라가 TCP보다 +5 cm (위)
ORIENT_PRESET_POSJ = (20, 35, 105, 105, -90, 50)  # 바닥(-Y) 방향 프리셋

# ─────────────────────────────────────────────────────────────
class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        self.get_logger().info("🤖 MotionController (combined) starting...")

        # FSM/주문 상태
        self.current_state = "IDLE"  # IDLE → PROGRESS → DONE
        self.order_id = None
        self.fuel_type = None
        self.amount = 0.0

        # 감지 상태
        self.last_label_ts = 0.0
        self.allowed_label = TARGET_LABEL
        self.last_car_detected_event = False

        # 이동 상태
        self.is_busy = False
        self.force_triggered = False

        # 그리퍼 초기화
        self._init_gripper_and_home()

        # 서비스(툴 방향 전환)
        self.srv_orient_y = self.create_service(
            Trigger,
            '/motion_controller/orient_negative_y',
            self.handle_orient_negative_y
        )
        
        # 구독/퍼블리셔
        self.sub_start = self.create_subscription(String, '/fuel_task/start', self.on_task_start, 10)
        self.sub_car_detected = self.create_subscription(String, '/car_detected', self.on_car_detected, 10)
        self.sub_yolo = self.create_subscription(String, '/fuel/yolo_detections', self.on_detections, 10)
        self.sub_obj3d = self.create_subscription(PointStamped, '/fuel/object_3d', self.object_callback, 10)
        self.sub_stop = self.create_subscription(Bool, '/stop_motion', self.on_stop_signal, 10)

        self.pub_status = self.create_publisher(String, '/fuel_status', 10)
        self.pub_target_dir = self.create_publisher(Float32, '/target_direction', 10)

        # 힘/토크(있으면 사용)
        try:
            from dsr_msgs2.msg import ForceTorque
            self.sub_force = self.create_subscription(ForceTorque, f'/{ROBOT_ID}/force_torque_raw', self.on_force, 10)
        except Exception:
            self.get_logger().warn("⚠️ Force topic type not available; skip force protection.")

        self.get_logger().info("✅ Subscriptions ready: /fuel_task/start, /car_detected, /fuel/yolo_detections, /fuel/object_3d, /stop_motion")

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
        except Exception as e:
            self.get_logger().error(f"❌ Gripper/Init error: {e}")
            raise

    def pose_to_matrix(self, pose):
        # pose가 중첩 리스트일 경우 자동 펼치기
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

    # ─────────────────────────────────────────────────────────
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
            self.get_logger().warn("⚙️ 이미 진행 중, 중복 실행 방지")
            return

        self.current_state = "IN_PROGRESS"
        self.get_logger().info("🚀 주유 시퀀스 시작: orient_negative_y() → search_for_object()")
        try:
            self.orient_negative_y()
        except Exception as e:
            self.get_logger().error(f"❌ 시퀀스 시작 실패: {e}")
            self.current_state = "ERROR"

        self.payment_confirmed = False
        self.get_logger().info("💳 결제 상태 초기화 (다음 주유 대기)")

    def search_for_object(self):
        self.searching = True
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

        self.searching = True
        for move_dir in directions:
            if not self.searching:
                self.get_logger().info("🛑 탐색 중단 (object_callback에서 종료)")
                break

            # 1️⃣ 한 방향으로 이동
            try:
                movel(dr_posx(*move_dir), v=20, a=20, mod=DR_MV_MOD_REL)
                wait(0.5)
            except Exception as e:
                self.get_logger().warn(f"⚠️ 탐색 이동 실패: {e}")

            # 2️⃣ 잠시 spin으로 콜백 기회 주기
            rclpy.spin_once(self, timeout_sec=0.2)

            # 3️⃣ YOLO 감지 확인
            age = time.time() - self.last_label_ts
            if age <= LABEL_TIMEOUT_SEC:
                self.get_logger().info(f"✅ 감지됨(age={age:.2f}s) → 탐색 종료")
                self.searching = False
                break

        self.searching = False
        self.get_logger().info("🔁 탐색 루프 종료")

    # ─────────────────────────────────────────────────────────
    # 3D 타깃 좌표 수신 → Base 변환 → 안전 이동
    def object_callback(self, msg: PointStamped):
        self.get_logger().info("📍 object_callback 탐색 중 ")
        if self.current_state != "PROGRESS":
            return
        
        if self.is_busy:
            self.get_logger().warn("⚠️ Busy, ignoring new target.")
            return

        try:
            from DSR_ROBOT2 import (get_current_posx, movel, wait, DR_MV_MOD_ABS, DR_MV_MOD_REL)
            from DR_common2 import posx

            Xc, Yc, Zc = msg.point.x, msg.point.y, msg.point.z
            pose = get_current_posx()

            if not pose or not isinstance(pose, (list, tuple)) or len(pose[0]) < 6:
                self.get_logger().error(f"❌ Invalid pose from get_current_posx(): {pose}")
                return

            x, y, z, rx, ry, rz = pose[0][0:6]
            target_pos = [x, y, z, rx, ry, rz]
            
            T_base2tcp = self.pose_to_matrix(target_pos)
            # -Yc : Y축이 반대로 설치
            cam_point = np.array([[-Xc], [-Yc], [Zc], [1]]) 
            base_point = T_base2tcp @ cam_point
            Xb, Yb, Zb = base_point[:3, 0]

            # 이동 명령 (mm 단위)
            target = posx(Xb*1000, Yb*1000, Zb*1000 + 140, rx, ry, rz)
            self.is_busy = True
            self.get_logger().info(
                f"🎯 Move Target (Base): X={target[0]:.3f} Y={target[1]:.3f} Z={target[2]:.3f} "
                f"RX={target[3]:.2f} RY={target[4]:.2f} RZ={target[5]:.2f}"
            )
            # target = posx(400, 0, 300, rx, ry, rz)
            movel(posx(target), v=30, a=30, mod=DR_MV_MOD_ABS)
            wait(2)
            self.get_logger().info("✅ Move completed.")

            self.gripper.move(0)
            wait(1.5)
            # 2️⃣ 순응 제어 활성화
            # self.check_crash()

        except Exception as e:
            self.get_logger().error(f"❌ Move failed: {e}")
        finally:
            self.is_busy = False

    # ─────────────────────────────────────────────────────────

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

    # ─────────────────────────────────────────────────────────
    # 방향 전환 (서비스/직접 호출)
    def handle_orient_negative_y(self, request, response):
        try:
            self.orient_negative_y()
            response.success = True
            response.message = "Tool oriented to -Y successfully"
        except Exception as e:
            response.success = False
            response.message = f"orient_negative_y failed: {e}"
        return response

    def orient_negative_y(self):
        from DSR_ROBOT2 import movej, wait, DR_MV_MOD_ABS
        from DR_common2 import posj
        self.get_logger().info("🧭 툴을 -Y(바닥) 방향으로 회전 중…")
        target_pose = posj(*ORIENT_PRESET_POSJ)
        movej(target_pose, v=50, a=50, mod=DR_MV_MOD_ABS)
        wait(2)
        self.get_logger().info("✅ 툴 방향 전환 완료 (-Y)")

        # 방향 전환 후 객체 탐색 수행
        try:
            self.get_logger().info("🔍 방향 전환 완료 → 객체 탐색 시작")
            self.search_for_object()
            # threading.Thread(target=self.search_for_object, daemon=True).start()
        except Exception as e:
            self.get_logger().warn(f"⚠️ 객체 탐색 중 오류 발생: {e}")

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
