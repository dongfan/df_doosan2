import cv2
import rclpy
from rclpy.node import Node
import numpy as np
import time
import math

from std_msgs.msg import String
import json

from enum import Enum

import DR_init
from dsr_example.gripper_drl_controller import GripperController
from cv_bridge import CvBridge
# from dsr_example.webcam_manager_ros import WebcamManagerROS
# from dsr_example.realsense_manager import RealSenseManager
# from dsr_example.yolo_manager import YoloDetector
# from dsr_example.dual_yolo_manager import DualYoloManager

ROBOT_STATE = Enum('ROBOT_STATE',
                   ['IDLE',
                    'PARKING_CAR',
                    'MOVE_TO_FUEL_POS',
                    'APPROACH_FUEL_NOZZLE',
                    'GRIP_NOZZLE',
                    'LIFT_NOZZLE',
                    'MOVE_TO_CAR_FUEL_PORT',
                    'FUELING',
                    'RETURN_NOZZLE',
                    'RELEASE_NOZZLE',
                    'MOVE_TO_HOME_POS'])

CAR_TYPE = Enum('CAR_TYPE',
                ['orange_car',
                 'truck',
                 'yellow_car'])

detected_car_list = []

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

g_vel_move = 80
g_vel_rotate = 120

g_force_lift = 20.0

# --------------------- 주유 위치 좌표 (더미값) ---------------------#
# 1. 주유구 위치 : 더미 좌표
g_car1_posj = [-13, 33, 82, -52, 58, 40]
g_car2_posj = [500, 0, 300, 0, 0, 0]

# 2. 주유건 잡기 전 위치
# g_oil1_ready_posj = [-10, 63, 32, 91, 85, -104] 기존좌표
g_oil1_ready_posj = [22, 17, 91,-109, 29, -152] 
oil_middle_posj = [13, 7, 68,-29, 34, -250]

# 주유시작 위치 : 더미 좌표
g_fuel_car1_posx = [497, -333, 308, 49, 140, 159]
g_fuel_car2_posx = [496, -352, 293, 49, 140, 160]

# 주유 완료후 주유건 위치 : 더미 좌표
g_oil1_go_posj = [-9, 68, 22, 91, 88, -88]
g_oil2_go_posj = [-14, 65, 48, 87, 86, -123]
g_oil1_end_posj = [-9, 68, 22, 91, 88, -88]
g_oil2_end_posj = [-14, 65, 48, 87, 86, -123]
# -----------------------------------------------------------------#

g_find_car_posj = [0, 0, 0, 0, 0, 0]
g_find_nozzle_posj = [0, 0, 0, 0, 0, 0]

grip_shot = 600
grip_gun = 600

g_Cap_Grip_Off = 440
g_Cap_Grip_On = 580

class FuelTaskManager(Node):
    def __init__(self):
        super().__init__("fuel_task_manager")
        self.get_logger().info("🦾 로봇 제어 노드 초기화 중...")

        # ✅ /fuel_task/start 구독 주유결제 명령 수신
        self.subscription = self.create_subscription(
            String,
            '/fuel_task/start',
            self.on_task_start,
            10)
        self.status_pub = self.create_publisher(String, '/fuel_status', 10)
        self.get_logger().info("🦾 FuelTaskManager started — waiting for /fuel_task/start")

        # --- Gripper 초기화 ---
        self.gripper = None
        try:
            from DSR_ROBOT2 import wait
            self.gripper = GripperController(node=self, namespace=ROBOT_ID)

            if not self.gripper.initialize():
                self.get_logger().error("Gripper initialization failed. Exiting.")
                raise Exception("Gripper initialization failed")
            
            self.get_logger().info("그리퍼를 활성화합니다...")
            self.gripper_is_open = True
            self.gripper.move(0)
            wait(2)
            
        except Exception as e:
            self.get_logger().error(f"An error occurred during gripper setup: {e}")
            rclpy.shutdown()

        # --- YOLO 객체 인식기 생성 ---
        self.bridge = CvBridge()
        # 상태 저장용
        self.tracked_objects = {}
        self.detected_car_list = []

        # 20Hz 루프
        # self.timer = self.create_timer(0.05, self.timer_callback)
        # self.get_logger().info("📸 FuelTaskManager with YOLO initialized")

        self.current_state = ROBOT_STATE.IDLE

    def timer_callback(self):
        # YOLO 감지 결과 저장용
        webcam_detections = []
        realsense_detections = []
        realsense_masks = []

        # -------------------------------------------------------
        # 🎥 1️⃣ 웹캠 (Detection)
        # -------------------------------------------------------
        webcam_frame = self.webcam.capture_and_publish()  # 내부에서 /fuel/image_result 퍼블리시
        if webcam_frame is not None:
            webcam_detections = self.yolo.detect_objects(webcam_frame)
            if webcam_detections:
                annotated_webcam = self.yolo.draw_detections(webcam_frame, webcam_detections)
                self.get_logger().info(f"🎯 [Webcam] Detections: {webcam_detections}")

                # 웹캠 YOLO 결과 ROS로 퍼블리시
                img_msg = self.bridge.cv2_to_imgmsg(annotated_webcam, encoding="bgr8")
                self.pub_webcam_result.publish(img_msg)  # ✅ 퍼블리셔 이름 구분 추천 (/fuel/webcam_result)

        # -------------------------------------------------------
        # 🤖 2️⃣ 리얼센스 (Segmentation)
        # -------------------------------------------------------
        rs_color, rs_depth = self.realsense.get_latest_frames()
        if rs_color is not None:
            realsense_detections, realsense_masks = self.yolo.segment_objects(rs_color)
            if realsense_masks:
                annotated_rs = self.yolo.draw_masks(rs_color, realsense_masks)
                self.get_logger().info(f"🟦 [RealSense] Segmentations: {len(realsense_masks)} masks")

                # 세그멘테이션 결과 퍼블리시
                img_msg = self.bridge.cv2_to_imgmsg(annotated_rs, encoding="bgr8")
                self.pub_realsense_result.publish(img_msg)  # ✅ /fuel/realsense_result

            # 중심 깊이 정보 (거리)
            depth_mm = self.realsense.get_center_depth()
            if depth_mm:
                self.get_logger().info(f"📏 Center depth: {depth_mm:.1f} mm")

        # -------------------------------------------------------
        # 🧠 3️⃣ 웹캠에서 감지된 차량 추적 및 정지 판정
        # -------------------------------------------------------
        current_time = time.time()
        for det in webcam_detections:
            cls = det["cls"]
            x1, y1, x2, y2 = det["bbox"]
            conf = det["conf"]
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            if not hasattr(self, "tracked_objects"):
                self.tracked_objects = {}

            if cls not in self.tracked_objects:
                # 첫 감지
                self.tracked_objects[cls] = {
                    "cx": cx,
                    "cy": cy,
                    "start_time": current_time,
                }
            else:
                prev = self.tracked_objects[cls]
                dist = math.sqrt((cx - prev["cx"])**2 + (cy - prev["cy"])**2)
                elapsed = current_time - prev["start_time"]

                # ✅ 3초 이상 고정 상태면 "차량 정지"로 판단
                if dist < 10 and elapsed >= 3.0:
                    self.get_logger().info(f"🟩 [Webcam] {cls} 정지 상태로 판단됨 (3초 이상 고정)")
                    # 👉 차량 정지 상태에서만 로봇 시퀀스 실행 가능
                    detected_car_list.append(det)

                    # 중복 실행 방지
                    self.tracked_objects[cls]["start_time"] = current_time

                # 위치 갱신
                self.tracked_objects[cls]["cx"] = cx
                self.tracked_objects[cls]["cy"] = cy

    def terminate_gripper(self):
        if self.gripper:
            try:
                print("🧹 Gripper 연결 종료 중...")
                if rclpy.ok():
                    self.gripper.terminate()
                    print("✅ Gripper 종료 완료")
                else:
                    print("⚠️ ROS context 종료됨 — terminate() 생략")
            except Exception as e:
                print(f"⚠️ 그리퍼 종료 중 오류: {e}")

    #--------------------- 초기화 부분 ---------------------#
    def robot_init(self):
        self.pos_init()
        self.grip_init()

        self.get_logger().info("Robot 초기화 완료.")

    def pos_init(self):
        from DSR_ROBOT2 import movej, posj, wait
        p_start = posj(0, 0, 90, 0, 90, 0)
        movej(p_start, 70, 70)
        wait(2.0)

    def grip_init(self):
        from DSR_ROBOT2 import wait
        self.gripper.move(0)
        wait(2.0)

    def orient_z_down(self):
        from DSR_ROBOT2 import (get_current_posx, movel, wait, DR_MV_MOD_ABS)
        from DR_common2 import posx

        c_pos = get_current_posx()
        x, y, z = c_pos[0][0:3]
        target_pos = posx(x, y, z, 0, 180, 0)

        movel(target_pos, v=50, a=50, mod=DR_MV_MOD_ABS)
        wait(3)

    #--------------------- Set Position Callback ---------------------#
    # 주유건이 충돌했는지 확인하고 대응하는 함수
    def check_crash(self):
        from DSR_ROBOT2 import (task_compliance_ctrl, set_desired_force, get_tool_force,
            release_force, release_compliance_ctrl, amovel, wait, DR_MV_MOD_REL)
        from DR_common2 import posx
        
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
    
    # 주유구를 오픈하기 위해 그리퍼를 회전시키는 함수
    def rotate_grip(self, cnt: int, b_open: bool = True):
        from DSR_ROBOT2 import (amovel, DR_MV_MOD_REL,
            movel, movej, wait)
        from DR_common2 import posx, posj
        count = 0
        open_angle = -120 if b_open else 120

        if b_open:
            while count < cnt :
                self.gripper.move(g_Cap_Grip_On)
                wait(1.5)
                
                movej(posj(0, 0, 0, 0, 0, open_angle), v=g_vel_rotate, a=g_vel_rotate, mod=DR_MV_MOD_REL)
                wait(1.0)
                count = count + 1

                if count < cnt:
                    self.gripper.move(g_Cap_Grip_Off)
                    wait(1.5)
                    movej(posj(0, 0, 0, 0, 0, -open_angle), v=g_vel_rotate, a=g_vel_rotate, mod=DR_MV_MOD_REL)
                    wait(1.0)

        else:
            while count < cnt :
                self.gripper.move(g_Cap_Grip_On)
                wait(1.0)
                
                movej(posj(0, 0, 0, 0, 0, open_angle), v=g_vel_rotate, a=g_vel_rotate, mod=DR_MV_MOD_REL)
                wait(1.0)
                count = count + 1

                if count < cnt:
                    self.gripper.move(g_Cap_Grip_Off)
                    wait(1.5)
                    movej(posj(0, 0, 0, 0, 0, -open_angle), v=g_vel_rotate, a=g_vel_move, mod=DR_MV_MOD_REL)
                    wait(1.0)
    
    # 반복적으로 그리퍼를 열고 닫는 작업을 수행 : 주유 시작       
    def run_fuel_task(self, cnt):
        from DSR_ROBOT2 import wait
        try:
            for i in range(cnt):
                self.gripper.move(grip_shot)
                wait(1.0)
                self.gripper.move(grip_gun)
                wait(1.0)

        except Exception as e:
            self.get_logger().error(f"❌ Gripper 반복 동작 중 오류: {e}")  
    
    def on_task_start(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Invalid message: {e}")
            return

        fuel_type = data.get("fuelType")
        amount = data.get("amount")
        order_id = data.get("orderId")

        self.get_logger().info(f"🚀 Starting fueling task for {fuel_type}, {amount}원 (Order {order_id})")

        # 실제 주유 로직 수행 ...
        self.current_state = ROBOT_STATE.MOVE_TO_FUEL_POS
        self.status_pub.publish(String(data="in_progress"))

        # 실제 로봇 주유 시퀀스 로직 연결
        self.execute_fuel_task(fuel_type, amount)

    def execute_fuel_task(self, fuel_type, amount):
        self.get_logger().info(f"🛠️ Executing robot motion for {fuel_type} / {amount}원 ...")
        # TODO: 여기에 로봇 제어 코드 삽입 (movel, 그리퍼, force control 등)

        # 유종별 주유량 로직 예시
        if fuel_type == "휘발유":
            self.start_simulation_fuel(amount)
        elif fuel_type == "경유":
            self.start_diesel_fuel(amount)
        else:
            self.get_logger().warn(f"Unknown fuel type: {fuel_type}")

    def start_simulation_fuel(self, amount):
        try:
            from DSR_ROBOT2 import get_current_posx, get_current_posj, movej, movel, wait, DR_MV_MOD_REL, DR_MV_MOD_ABS
            from DR_common2 import posx, posj
        except ImportError as e:
            print(f"DSR_ROBOT2 라이브러리를 임포트할 수 없습니다: {e}")
            rclpy.shutdown()
            exit(1)

        # 🔧 실제 로봇 주유 동작 시퀀스 작성
        self.get_logger().info(f"⛽ Gasoline fueling sequence for {amount}원 started...")
        
        if amount < 20000:
            amount = 20000 # 최소 1회 주유
        
        m_count = amount // 20000  # 30000원 단위로 주유 횟수 결정

        # gun_posj = get_current_posj()

        # 로봇 위치, 그리퍼 초기화
        self.robot_init()

        #--------------------- 차량 진입 후 작업 시작 ---------------------#
        # 주유구 위치로 이동 
        movej(g_car1_posj, 80, 80)
        wait(2.0)

        # 주유구 뚜껑 잡으러 이동 -> 오픈을 위한 그리퍼 회전
        movel(posx(0, -28, -18, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        self.rotate_grip(2, True)
        
        movel(posx(0, 30, 30, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(1.0)
        movel(posx(0, 80, -50, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(1.0)
        self.orient_z_down()
        cap_pick_posj = get_current_posj()
        self.check_crash()

        # 주유건 위치로 이동 후 그리퍼 닫기
        movej(g_oil1_ready_posj, 100, 100)
        wait(2.0)

        movel(posx(100, -10, 0, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)
        
        g_oil1_end_posj = get_current_posj()
        self.gripper.move(grip_gun)
        wait(2.5)

        # 주유건 그립 이후 주유건 뽑아 가기
        movel(posx(0, 0, 120, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)
        g_oil1_go_posj = get_current_posj()
        movel(posx(-100, -120, 150, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)

        # 주유구 위치로 이동 
        movel(g_fuel_car1_posx, 80, 80)
        wait(3.0)
        movel(g_fuel_car2_posx, 80, 80)
        # movel(posx(0, -75, -65, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)

        #--------------------- 직접 주유 작업 시작 ---------------------#
        # 주유 작업 반복 수행
        # self.run_fuel_task(m_count)
        wait(4.0)
        
        # (car)주유구에서 주유건 빼기
        movel(posx(0, 75, 65, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)

        movel(posx(0, 55, 150, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(1.0)

        movej(g_oil1_go_posj, 80, 80)
        wait(2.0)
        movel(posx(0, 0, -80, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        # movej(g_oil1_end_posj, 80, 80)
        wait(2.0)

        self.grip_init()
        wait(2.0)

        movej(g_oil1_ready_posj, 80, 80)
        wait(2.0)

        movej(cap_pick_posj, 80, 80)
        wait(2.0)
        self.gripper.move(g_Cap_Grip_On)
        wait(2.0)
        self.check_crash()

        movel(posx(0, 0, -10, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)
        self.gripper.move(g_Cap_Grip_On)
        wait(2.0)

        movej(g_car1_posj, 80, 80)
        wait(2.0)

        # # 주유구 뚜껑 닫으러 이동 
        movel(posx(-10, -38, -25, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        self.rotate_grip(2, False)
        self.grip_init()

        self.pos_init()

        # 주유 완료 시:
        self.current_state = ROBOT_STATE.IDLE
        self.status_pub.publish(String(data="completed"))
        self.get_logger().info("✅ Fueling completed.")
    
    def start_diesel_fuel(self, amount):
        self.get_logger().info(f"⛽ Diesel fueling sequence for {amount}원 started...")

    def start_window_cleaning(self):
        print("🚿 Starting window cleaning process...")

def main(args=None):
    # ✅ 1️⃣ ROS 초기화 먼저
    rclpy.init(args=args)

    # ✅ 2️⃣ 노드 생성 순서 정리
    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    # ✅ 3️⃣ FuelTaskManager 생성 (이제 Node 생성 가능)
    fuel_controller = FuelTaskManager()

    try:
        while rclpy.ok():
            rclpy.spin_once(fuel_controller, timeout_sec=0.05)
            # fuel_controller.start_simulation_fuel(10000)
            # Test code
            # fuel_controller.start_gasoline_fuel(60000)
            # for d in detected_car_list:
            #     car_type = d['cls']
            #     if car_type == 'truck' and fuel_controller.current_state == ROBOT_STATE.IDLE:
            #         fuel_controller.get_logger().info(f"🟩 {car_type} 주유 가능")
            #         fuel_controller.current_state = ROBOT_STATE.PARKING_CAR
                    # fuel_controller.start_gasoline_fuel(50000)
                    # fuel_controller.current_state = ROBOT_STATE.MOVE_TO_FUEL_POS
                # elif car_type == 'yellow_car' and self.current_state == ROBOT_STATE.IDLE:
                #     fuel_controller.run_robot_sequence()
                #     self.current_state = ROBOT_STATE.MOVE_TO_FUEL_POS
            
    except KeyboardInterrupt:
        print("🛑 Keyboard Interrupt 감지됨, 로봇 정지 중...")
        fuel_controller.terminate_gripper() 
        pass

    finally:
        try:
            fuel_controller.terminate_gripper()
            fuel_controller.destroy_node()
            dsr_node.destroy_node()
        except Exception:
            print("⚠️ Node 종료 중 오류 무시")

        # 3️⃣ ROS context 마지막에 shutdown
        if rclpy.ok():
            rclpy.shutdown()

        print("✅ 종료 완료.")

if __name__ == '__main__':
    main()