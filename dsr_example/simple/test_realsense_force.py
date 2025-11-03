import cv2
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import time
import math

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters

import DR_init
from dsr_example.simple.gripper_drl_controller import GripperController

VELOCITY, ACC = 70, 70

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

g_vel_move = 120
g_vel_lift = 20.0
g_pos = [0, 0, 0, 0, 0, 0]

g_Grip = 570
g_GripNot = 500

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("robot_controller_node")

        self.bridge = CvBridge()

        self.get_logger().info("ROS 2 구독자 설정을 시작합니다...")

        self.intrinsics = None
        self.latest_cv_color = None
        self.latest_cv_depth_mm = None

        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw'
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info'
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)

        self.get_logger().info("컬러/뎁스/카메라정보 토픽 구독 대기 중...")
        self.get_logger().info("화면이 나오지 않으면 Launch 명령어를 확인하세요.")

        self.gripper = None
        try:
            from DSR_ROBOT2 import wait
            self.gripper = GripperController(node=self, namespace=ROBOT_ID)
            wait(2)
            if not self.gripper.initialize():
                self.get_logger().error("Gripper initialization failed. Exiting.")
                raise Exception("Gripper initialization failed")
            self.get_logger().info("그리퍼를 활성화합니다...")
            self.gripper_is_open = True
            self.gripper.move(0)
            
        except Exception as e:
            self.get_logger().error(f"An error occurred during gripper setup: {e}")
            rclpy.shutdown()

        self.get_logger().info("RealSense ROS 2 구독자와 로봇 컨트롤러가 초기화되었습니다.")

    def synced_callback(self, color_msg, depth_msg, info_msg):
        try:
            self.latest_cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            self.latest_cv_depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge 변환 오류: {e}")
            return

        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = info_msg.width
            self.intrinsics.height = info_msg.height
            self.intrinsics.ppx = info_msg.k[2]
            self.intrinsics.ppy = info_msg.k[5]
            self.intrinsics.fx = info_msg.k[0]
            self.intrinsics.fy = info_msg.k[4]
            
            if info_msg.distortion_model == 'plumb_bob' or info_msg.distortion_model == 'rational_polynomial':
                self.intrinsics.model = rs.distortion.brown_conrady
            else:
                self.intrinsics.model = rs.distortion.none
            
            self.intrinsics.coeffs = list(info_msg.d)
            self.get_logger().info("카메라 내장 파라미터(Intrinsics) 수신 완료.")

    def stop_camera(self):
        pass

    def terminate_gripper(self):
        if self.gripper:
            self.gripper.terminate()

    def mouse_callback(self, event, u, v, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.latest_cv_depth_mm is None or self.intrinsics is None:
                self.get_logger().warn("아직 뎁스 프레임 또는 카메라 정보가 수신되지 않았습니다.")
                return

            try:
                depth_mm = self.latest_cv_depth_mm[v, u]
            except IndexError:
                self.get_logger().warn(f"클릭 좌표(u={u}, v={v})가 이미지 범위를 벗어났습니다.")
                return
            
            if depth_mm == 0:
                print(f"({u}, {v}) 지점의 깊이를 측정할 수 없습니다 (값: 0).")
                return

            depth_m = float(depth_mm) / 1000.0

            point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth_m)

            x_mm = point_3d[1] * 1000
            y_mm = point_3d[0] * 1000
            z_mm = point_3d[2] * 1000

            final_x = 635 + x_mm - 20
            final_y = y_mm
            final_z = 970 - z_mm + 140
            if(final_z <= 150):
                final_z = 150

            if(final_x <= 200):
                final_x = 200

            print("--- 변환된 최종 3D 좌표 ---")
            print(f"픽셀 좌표: (u={u}, v={v}), Depth: {depth_m*1000:.1f} mm")
            print(f"로봇 목표 좌표: X={final_x:.1f}, Y={final_y:.1f}, Z={final_z:.1f}\n")

            self.move_robot_and_control_gripper(final_x, final_y, final_z)
            print("=" * 50)

    def move_robot_and_control_gripper(self, x, y, z):
        from DSR_ROBOT2 import get_current_posx, movel, wait, movej
        from DR_common2 import posx, posj
        try:
            current_pos = get_current_posx()[0]
            target_pos_list_up = [x, y, 430, current_pos[3], current_pos[4], current_pos[5]]
            target_pos_list = [x, y, 430, current_pos[3], current_pos[4], current_pos[5]]
            p_start = posj(0, 0, 90, 0, 90, 0)

            movel(posx(target_pos_list_up), vel=VELOCITY, acc=ACC)
            wait(0.5)

            self.get_logger().info(f"목표 지점으로 이동합니다: {target_pos_list}")
            movel(posx(target_pos_list), vel=VELOCITY, acc=ACC)
            wait(0.5)

            # self.gripper.move(700)
            # wait(2)

            # movel(posx(target_pos_list_up), vel=VELOCITY, acc=ACC)
            # wait(0.5)

            # self.gripper.move(0)
            # wait(2)

            # self.get_logger().info("초기 자세로 복귀합니다.")
            # movej(p_start, VELOCITY, ACC)

        except Exception as e:
            self.get_logger().error(f"로봇 이동 및 그리퍼 제어 중 오류 발생: {e}")

        # dongfan

    def check_crash(self):
        from DSR_ROBOT2 import (task_compliance_ctrl, set_desired_force, get_tool_force,
            release_force, release_compliance_ctrl, amovel, wait, DR_MV_MOD_REL)
        from DR_common2 import posx, posj
        
        k_d = [500.0, 500.0, 500.0, 200.0, 200.0, 200.0]
        task_compliance_ctrl(k_d)
        # 강성 제어
        f_d = [0.0, 0.0, -g_vel_lift, 0.0, 0.0, 0.0]
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

                amovel(posx(0, 0, 79, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
                wait(2.0)
                break

    def rotate_grip(self, s_cnt):
        from DSR_ROBOT2 import (amovel, DR_MV_MOD_REL,
            movel, wait)
        from DR_common2 import posx, posj
        count = 0

        while count < s_cnt :
            self.gripper.move(g_Grip)
            wait(3.0)
            
            if count > 0 :
                movel(posx(0, 0, 5, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
                wait(1.0)
                amovel(posx(0, 0, -5, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
                wait(1.0)

            movel(posx(0, 0, 0, 100, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
            wait(1.5)

            self.gripper.move(g_GripNot)
            wait(1.0)
            movel(posx(0, 0, 0, -100, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
            wait(1.0)
            count = count + 1

    def move_and_control_grip(self, target_pos, approach_axis: str):
        from DSR_ROBOT2 import (amovel, DR_MV_MOD_REL,
            movel, wait)
        from DR_common2 import posx, posj
        """
        물체 기준 접근 방향에 따라 J6(Rz) 각도를 자동으로 맞추는 함수.

        Args:
            target_pos (list or posx): [x, y, z, Rx, Ry, Rz]
            approach_axis (str): 'x+', 'x-', 'y+', 'y-'

        Returns:
            posx: 조정된 자세 (gripper가 물체를 바라보도록 Rz 수정)
        """

        if isinstance(target_pos, list):
            x, y, z, Rx, Ry, Rz = target_pos
        else:
            x, y, z, Rx, Ry, Rz = target_pos._posx  # posx 타입일 경우

        # 기준 Rz (deg)
        if approach_axis == "x+":
            Rz_target = 180.0
        elif approach_axis == "x-":
            Rz_target = 0.0
        elif approach_axis == "y+":
            Rz_target = -90.0
        elif approach_axis == "y-":
            Rz_target = 90.0
        else:
            raise ValueError("approach_axis must be one of ['x+','x-','y+','y-']")

        # 최종 자세 반환
        adjusted_pose = posx(x, y, z, Rx, Ry, Rz_target)
        return adjusted_pose


def main(args=None):
    rclpy.init(args=args)

    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    try:
        from DSR_ROBOT2 import get_current_posx, movel, wait, movej, DR_MV_MOD_REL
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"DSR_ROBOT2 라이브러리를 임포트할 수 없습니다: {e}")
        rclpy.shutdown()
        exit(1)

    robot_controller = RobotControllerNode()

    P0 = posj(0,0,90,0,90,0)
    print("홈 위치로 이동합니다")
    movej(P0, 100, 100)
    wait(1.0)

    # movel(posx(0, 0, -100, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
    # wait(1.0)
    # robot_controller.check_crash()

    # robot_controller.rotate_grip(3)
    # robot_controller.gripper.move(g_Grip)
    # wait(0.5)

    # movel(posx(0, 0, 100, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
    # wait(1.0)

    cv2.namedWindow("RealSense Camera")
    cv2.setMouseCallback("RealSense Camera", robot_controller.mouse_callback)

    print("카메라 영상에서 원하는 지점을 클릭하세요. 'ESC' 키를 누르면 종료됩니다.")

    try:
        while rclpy.ok():
            rclpy.spin_once(robot_controller, timeout_sec=0.001)
            rclpy.spin_once(dsr_node, timeout_sec=0.001)

            if robot_controller.latest_cv_color is not None:
                display_image = robot_controller.latest_cv_color.copy()
                
                h, w, _ = display_image.shape
                cv2.circle(display_image, (w // 2, h // 2), 3, (0, 0, 255), -1)
                
                cv2.imshow("RealSense Camera", display_image)

            if cv2.waitKey(1) & 0xFF == 27:
                break
    
    except KeyboardInterrupt:
        print("Ctrl+C로 종료합니다...")

    # finally:
    print("프로그램을 종료합니다...")
    robot_controller.terminate_gripper()
    # cv2.destroyAllWindows()
    robot_controller.destroy_node()
    dsr_node.destroy_node()
    rclpy.shutdown()
    print("종료 완료.")

if __name__ == '__main__':
    main()