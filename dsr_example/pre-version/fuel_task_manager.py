#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Float32, Bool

import json, math
import threading


class FuelTaskManager(Node):
    def __init__(self):
        super().__init__("fuel_task_manager")
        self.get_logger().info("🧠 FuelTaskManager 초기화 (Detection-only)...")

        # ✅ 토픽 설정
        self.sub_car_detected = self.create_subscription(
        String, '/car_detected', self.on_car_detected, 10)
        self.sub_start = self.create_subscription(String, '/fuel_task/start', self.on_task_start, 10)
        self.pub_status = self.create_publisher(String, '/fuel_status', 10)
        self.pub_target_dir = self.create_publisher(Float32, '/target_direction', 10)
        self.pub_target_pos = self.create_publisher(PointStamped, '/target_position', 10)
        self.pub_stop = self.create_publisher(Bool, '/stop_motion', 10)
        
        # 상태 변수
        self.current_state = "IDLE"
        self.detected_target = None

        self.last_car_detected = False

        self.sub_yolo = self.create_subscription(
            String, '/fuel/image_result', self._yolo_callback, 10)

    # ───────────────────────────────
    def on_task_start(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.fuel_type = data.get("fuelType", "unknown").lower()
            self.amount = float(data.get("amount", 0))
            self.order_id = data.get("orderId", "none")

            self.get_logger().info(f"💳 결제 요청: ID={self.order_id} | FuelType={self.fuel_type} | Amount={self.amount:.0f}")
            
            # ✅ Step 1: 현재 차량 감지 여부 확인
            car_present = self.check_car_presence()

            if car_present:
                # 바로 주유 시작
                self.get_logger().info("🚗 차량이 이미 감지됨 → 즉시 주유 프로세스 시작")
                self.pub_status.publish(String(data="progress"))
                self.current_state = "PROGRESS"
                self.orient_robot_to_negative_y()

        except Exception as e:
            self.get_logger().error(f"❌ JSON 파싱 오류: {e}")

    def _yolo_callback(self, msg: String):
        try:
            detections = json.loads(msg.data)
            # 차량 객체가 하나라도 있으면 True
            self.last_car_detected = any(
                det["cls"] in ["car", "truck", "bus"] for det in detections
            )
        except Exception:
            self.last_car_detected = False

    def check_car_presence(self) -> bool:
        """최근 YOLO 감지 결과로 차량 존재 여부 판단"""
        return self.last_car_detected

    def on_car_detected(self, msg: String):
        # if self.current_state != "WAITING_FOR_CAR":
        #     return

        self.get_logger().info("✅ 차량 감지 완료 → 주유 프로세스 시작")
        self.pub_status.publish(String(data="progress"))
        self.current_state = "PROGRESS"

        # 로봇을 -Y 방향으로 회전
        self.orient_robot_to_negative_y()

        if self.fuel_type == "gasoline":
            self.get_logger().info("⛽ 유종: 휘발유 → 일반 주유 시퀀스 준비")
        elif self.fuel_type == "diesel":
            self.get_logger().info("🛢️ 유종: 디젤 → 딥그립 모드 준비")
        else:
            self.get_logger().warn("⚠️ 알 수 없는 유종, 기본 동작으로 진행")

    # ───────────────────────────────
    def handle_detections(self, detections):
        """
        YOLO 탐지 결과 처리 — nozzle/fuel_port 감지 시 방향+좌표 전송
        """
        nozzle, port = None, None
        for det in detections:
            if det["cls"] == "nozzle":
                nozzle = det
            elif det["cls"] == "fuel_port":
                port = det

        if nozzle and port:
            nx, ny = self._bbox_center(nozzle["bbox"])
            px, py = self._bbox_center(port["bbox"])
            dx, dy = px - nx, py - ny
            yaw_deg = math.degrees(math.atan2(dy, dx))
            self.pub_target_dir.publish(Float32(data=yaw_deg))
            self.get_logger().info(f"🎯 방향 각도 퍼블리시: {yaw_deg:.2f}°")

            # RealSense Z-depth 기반 좌표 퍼블리시
            depth_mm = self.realsense.get_center_depth()
            if depth_mm:
                msg = PointStamped()
                msg.header.frame_id = "camera_link"
                msg.point.x = 0.3
                msg.point.y = 0.0
                msg.point.z = depth_mm / 1000.0
                self.pub_target_pos.publish(msg)
                self.get_logger().info(f"📍 목표좌표 퍼블리시: z={msg.point.z:.3f}m")

    def _bbox_center(self, bbox):
        x1, y1, x2, y2 = bbox
        return (x1 + x2) / 2, (y1 + y2) / 2

    def orient_robot_to_negative_y(self):
        """MotionController의 /orient_negative_y 서비스 호출"""
        from std_srvs.srv import Trigger

        def _call_service():
            try:
                cli = self.create_client(Trigger, '/motion_controller/orient_negative_y')
                if not cli.wait_for_service(timeout_sec=5.0):
                    self.get_logger().error("❌ orient_negative_y 서비스가 준비되지 않음.")
                    return
                req = Trigger.Request()
                future = cli.call_async(req)

                def _done_callback(fut):
                    try:
                        result = fut.result()
                        if result and result.success:
                            self.get_logger().info("✅ 로봇 툴 -Y 방향 회전 완료")
                        else:
                            self.get_logger().warn("⚠️ 로봇 회전 실패 또는 무응답")
                    except Exception as e:
                        self.get_logger().error(f"❌ 서비스 응답 처리 실패: {e}")

                # 🔹 완료 콜백 등록 (spin 중단 없이 처리)
                future.add_done_callback(_done_callback)

            except Exception as e:
                self.get_logger().error(f"❌ 로봇 회전 서비스 호출 실패: {e}")

        # 🔹 별도 스레드로 서비스 호출 실행
        threading.Thread(target=_call_service, daemon=True).start()

# ───────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = FuelTaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("🛑 FuelTaskManager 종료됨 (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
