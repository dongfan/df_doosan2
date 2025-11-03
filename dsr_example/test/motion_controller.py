#!/usr/bin/env python3
# motion_controller.py — Final Safe Version
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Bool
import numpy as np
import json
import time

from std_srvs.srv import Trigger

import DR_init
from dsr_example.gripper_drl_controller import GripperController

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# -------------------------------------------------------------------
# 사용자 설정(필요 시 조정)
TARGET_LABEL = "green_car"   # 허용 라벨 (향후 변경 가능)
LABEL_TIMEOUT_SEC = 1.0       # 라벨 감지 후 유효 시간
V_MAX = 60                    # 이동 속도 상한 (Doosan 단위)
A_MAX = 60                    # 가속도 상한

# 접근/안전 파라미터 (mm)
PRE_UP_MM = 120.0             # 접근 전 위로 확보할 높이
STANDOFF_MM = 120.0           # 목표 지점 위에서 멈출 여유
MIN_Z_MM = 400.0              # 절대 최소 Z (바닥/테이블 충돌 방지)
WS_XY_MM = 800.0              # XY 워크스페이스 안전 한계(±)
# -------------------------------------------------------------------

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        self.get_logger().info("🤖 MotionController (final safe version) starting...")

        # --- Gripper 초기화 ---
        self.gripper = None
        try:
            from DSR_ROBOT2 import wait, movej
            self.gripper = GripperController(node=self, namespace=ROBOT_ID)

            if not self.gripper.initialize():
                self.get_logger().error("Gripper initialization failed. Exiting.")
                raise Exception("Gripper initialization failed")
            
            self.get_logger().info("그리퍼를 활성화합니다...")
            self.gripper_is_open = True
            self.gripper.move(0)
            wait(2)
            movej([0, 0, 90, 0, 90, 0], 100, 100)
            wait(2)
            
        except Exception as e:
            self.get_logger().error(f"An error occurred during gripper setup: {e}")
            rclpy.shutdown()

        self.srv_orient_y = self.create_service(
            Trigger,
            '/motion_controller/orient_negative_y',
            self.handle_orient_negative_y
        )
        
        # 상태
        self.is_busy = False
        self.last_label_ts = 0.0
        self.allowed_label = TARGET_LABEL
        self.latest_target_base = None  # (Xb, Yb, Zb) in meters

        # 구독: 라벨 감지(JSON), 객체 좌표(base), 정지/방향 명령(옵션)
        self.sub_det = self.create_subscription(String, '/fuel/yolo_detections',
                                                self.on_detections, 10)
        self.sub_stop = self.create_subscription(Bool, '/stop_motion',
                                                 self.on_stop_signal, 10)
        self.sub_dir = self.create_subscription(String, '/target_direction',
                                                self.on_direction, 10)  # (옵션) 사용 안하면 무시됨

        # 객체 좌표 구독
        self.sub_obj = self.create_subscription(PointStamped, '/fuel/object_3d', self.object_callback, 10)

        # 힘/토크(있으면 사용, 없으면 경고만)
        self.force_triggered = False
        try:
            from dsr_msgs2.msg import ForceTorque
            self.sub_force = self.create_subscription(ForceTorque, '/dsr01/force_torque_raw',
                                                      self.on_force, 10)
        except Exception:
            self.get_logger().warn("⚠️ Force topic type not available; skip force protection.")

        self.get_logger().info("✅ Subscriptions ready: /fuel/object_base, /fuel/yolo_detections, /stop_motion")

    # ─────────────────────────────────────────────────────────────
    # 콜백들
    def on_detections(self, msg: String):
        """YOLO 결과 JSON에서 허용 라벨 감지 시 타임스탬프 갱신"""
        try:
            dets = json.loads(msg.data)
            labels = [d.get('cls') for d in dets if 'cls' in d]
            if self.allowed_label in labels:
                self.last_label_ts = time.time()
        except Exception as e:
            self.get_logger().warn(f"parse det error: {e}")

    def on_stop_signal(self, msg: Bool):
        if msg.data:
            self.hard_stop_and_release()

    def on_direction(self, msg: String):
        # (옵션) 필요 시 방향 프리셋 처리 가능. 현재는 로그만.
        self.get_logger().info(f"ℹ️ target_direction: {msg.data}")

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

    def object_callback(self, msg):
        skip_classes = ["green_car", "orange_car", "yellow_car"]    

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

    def on_force(self, msg):
        """힘센서 임계 초과 시 즉시 정지 & 후퇴"""
        if self.force_triggered:
            return
        Fx, Fy, Fz = msg.fx, msg.fy, msg.fz
        total = (Fx**2 + Fy**2 + Fz**2) ** 0.5
        # 경험적으로 15~20N부터 회피 추천
        if total > 15.0:
            self.force_triggered = True
            self.get_logger().warn(f"⚠️ Collision detected! F={total:.1f}N → stop & retreat")
            self.hard_stop_and_release()
            self.force_triggered = False

    def handle_orient_negative_y(self, request, response):
        self.orient_negative_y()
        response.success = True
        response.message = "Tool oriented to -Y successfully"
        return response
    # ─────────────────────────────────────────────────────────────
    def orient_negative_y(self):
        """
        로봇 TCP(툴)가 -Y 방향을 바라보도록 회전
        (즉, 카메라가 바닥 방향을 향하게 함)
        """
        from DSR_ROBOT2 import movej, wait, get_current_posx, DR_MV_MOD_ABS
        from DR_common2 import posj

        try:
            # pose = get_current_posx()
            # if not pose or len(pose[0]) < 6:
            #     self.get_logger().error("❌ 현재 자세를 불러오지 못했습니다.")
            #     return

            # x, y, z, rx, ry, rz = pose[0][:6]

            # Y축 반대 방향으로 툴을 회전 (바닥을 향하게)
            target_pose = posj(20, 35, 105, 105, -90, 50)

            self.get_logger().info("🧭 툴을 -Y 방향(바닥 방향)으로 회전 중...")
            movej(target_pose, v=50, a=50, mod=DR_MV_MOD_ABS)
            wait(2)
            self.get_logger().info("✅ 툴 방향 전환 완료 (-Y 방향)")
        except Exception as e:
            self.get_logger().error(f"❌ orient_negative_y() 실패: {e}")

    def drop_car_cap(self):
        from DSR_ROBOT2 import (task_compliance_ctrl, set_desired_force, get_tool_force,
            release_force, release_compliance_ctrl, get_current_posx, wait, DR_MV_MOD_REL,
            DR_MV_MOD_ABS, get_current_posj, movel)
        from DR_common2 import posx

        movel(posx(0, 30, 30, 0, 0, 0), v=100, a=100, mod=DR_MV_MOD_REL)
        wait(1.0)
        movel(posx(0, 80, -50, 0, 0, 0), v=100, a=100, mod=DR_MV_MOD_REL)
        wait(1.0)
        
        # 툴축이 바닥을 향하도록 이동
        c_pos = get_current_posx()
        x, y, z = c_pos[0][0:3]
        target_pos = posx(x, y, z, 0, 180, 0)

        movel(target_pos, v=50, a=50, mod=DR_MV_MOD_ABS)
        wait(3)

        self.cap_pick_posj = get_current_posj()
        
        k_d = [500.0, 500.0, 500.0, 200.0, 200.0, 200.0]
        task_compliance_ctrl(k_d)
        # 강성 제어
        f_d = [0.0, 0.0, -20, 0.0, 0.0, 0.0]
        f_dir = [0, 0, 1, 0, 0, 0]
        set_desired_force(f_d, f_dir)
        wait(2.0)

        # 외력감지
        while True:
            force_ext = get_tool_force()
            # c_pos = get_current_posx()
            # x, y, z = c_pos[0]
            if force_ext[2] > 4:
                release_force()
                release_compliance_ctrl()

                self.gripper.move(0)
                wait(1.5)
                break        

    # 안전 이동 로직
    def safe_move_to_base_target(self, target_m):
        """절대 좌표(베이스 프레임, m 단위)로 안전하게 접근"""
        if target_m is None:
            return
        Xb, Yb, Zb = target_m  # meters

        # 1) 단위 변환 (m → mm)
        tx = Xb * 1000.0
        ty = Yb * 1000.0
        tz = Zb * 1000.0

        # 2) 워크스페이스 / 최소 Z 가드
        if abs(tx) > WS_XY_MM or abs(ty) > WS_XY_MM:
            self.get_logger().warn(f"🚫 XY out of workspace: ({tx:.1f},{ty:.1f})mm")
            return

        # 최종 접근 Z는 '목표점 위로 STANDOFF' 에서 멈춤
        z_approach = max(tz + STANDOFF_MM, MIN_Z_MM)
        z_pre = max(z_approach + PRE_UP_MM, MIN_Z_MM)

        # 3) 이동 실행
        self.is_busy = True
        try:
            from DSR_ROBOT2 import movej, movel, move_stop, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_TOOL
            from DR_common2 import posx as dr_posx

            # 오리엔테이션: 기본 정면(0,180,0). 필요 시 바꾸세요.
            RPY = (0.0, 180.0, 0.0)

            # (a) Pre-approach: 목표 XY, Z_pre 로 관절 이동(안전)
            pre = dr_posx(tx, ty, z_pre, *RPY)
            self.get_logger().info(f"🅰️ movej pre → {pre}")
            movej(pre, v=min(V_MAX, 60), a=min(A_MAX, 60), mod=DR_MV_MOD_ABS)

            # (b) Approach: 직선 접근으로 Z_approach 까지
            ap = dr_posx(tx, ty, z_approach, *RPY)
            self.get_logger().info(f"🅱️ movel approach → {ap}")
            movel(ap, v=min(V_MAX, 50), a=min(A_MAX, 50), mod=DR_MV_MOD_ABS)

            # (c) (필요작업 수행 지점) — 여기서 그리퍼 동작 등 수행 가능
            # 추후 fuel_task_manager에서 트리거하도록 유지

            # (d) Retreat: 직선 후퇴로 다시 Z_pre
            self.get_logger().info("↩️ movel retreat (back to pre)")
            movel(pre, v=min(V_MAX, 50), a=min(A_MAX, 50), mod=DR_MV_MOD_ABS)

            self.get_logger().info("✅ Safe approach sequence completed")
        except Exception as e:
            self.get_logger().error(f"❌ Move sequence failed: {e}")
            # 혹시 모를 중간 정지
            try:
                from DSR_ROBOT2 import move_stop
                move_stop()
            except Exception:
                pass
        finally:
            self.is_busy = False

    # ─────────────────────────────────────────────────────────────
    # 비상 정지 & 후퇴(툴 좌표 상대이동)
    def hard_stop_and_release(self):
        try:
            from DSR_ROBOT2 import move_stop, movel, DR_MV_MOD_REL, DR_TOOL
            from DR_common2 import posx as dr_posx
            move_stop()
            # 툴 좌표 Z+로 10mm 후퇴
            rel = dr_posx(0, 0, 10, 0, 0, 0)
            movel(rel, v=20, a=20, mod=DR_MV_MOD_REL, ref=DR_TOOL)
            self.get_logger().info("🛑 Stopped & retreated (tool Z+10mm)")
        except Exception as e:
            self.get_logger().warn(f"Stop/retreat failed: {e}")


def main(args=None):
    rclpy.init(args=args)

    # ✅ 2️⃣ 노드 생성 순서 정리
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
