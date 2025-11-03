import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import DrlStart
import DR_init
from dsr_example.simple.gripper_drl_controller import GripperController

grip_on = 440
grip_off = 250


def main(args=None):
    rclpy.init(args=args)
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    VEL = 50
    ACC = 50
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    node = rclpy.create_node('gripper_example_py', namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    
    from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS, posj, movej, wait
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    gripper = None
    try:
        gripper = GripperController(node=node, namespace=ROBOT_ID)
        if not gripper.initialize():
            node.get_logger().error("Gripper initialization failed. Exiting.")
            return

        node.get_logger().info("Sending a priming command to wake up the gripper...")
        wait(5)
        
        
        # p_start = posj(0, 0, 90, 0, 90, 0)
        # movej(p_start, VEL, ACC)
        # wait(4)
        gripper.move(0) 
        wait(3)
        gripper.move(200) 
        wait(3)
        gripper.move(500) 
        wait(3)
        gripper.move(700) 
        wait(3)
        gripper.move(400) 
        wait(3)
        gripper.move(0) 
        wait(3)
        
        # for i in range(6):
        #         stroke = grip_off if i % 2 == 0 else grip_on
        #         node.get_logger().info(f"[{i+1}/{5}] Gripper move → stroke: {stroke}")
        #         result = gripper.move(stroke)

        #         if not result:
        #             node.get_logger().error(f"❌ Gripper move failed at iteration {i+1}")
        #             break

        #         # 대기시간 동안 ROS 콜백은 계속 처리되도록 함
        #         import time
        #         start = time.monotonic()
        #         while time.monotonic() - start < 2.6:
        #             rclpy.spin_once(node, timeout_sec=0.1)
        
        # p1_joint = posj(45, 0, 90, 0, 90, 0)
        # p2_joint = posj(-45, 0, 90, 0, 90, 0)

        # print("movej: p1_joint 위치로 이동합니다.")
        # movej(p1_joint, VEL, ACC)
        # gripper.move(0) 
        # wait(2)

        # print("movej: p2_joint 위치로 이동합니다.")
        # movej(p2_joint, VEL, ACC)
        # gripper.move(700) # 0 -> 700 으로 이동
        # wait(2)

        # print("시작 자세로 복귀합니다.")
        # movej(p_start, VEL, ACC)
        # gripper.move(0) # 700 -> 0 으로 이동
        # wait(2)

    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        if gripper:
            gripper.terminate()
        rclpy.shutdown()
        node.get_logger().info("Shutdown complete.")

if __name__ == '__main__':
    main()