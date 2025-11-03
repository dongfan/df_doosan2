import rclpy
import DR_init
import sys

def main(args=None):
    rclpy.init(args=args)

    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    VEL = 50
    ACC = 50

    node = rclpy.create_node('example_py', namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    from DSR_ROBOT2 import(
        movej, posj, set_robot_mode, 
        ROBOT_MODE_AUTONOMOUS, 
        task_compliance_ctrl, release_compliance_ctrl, wait
    ) 

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    P0 = posj(0,0,90,0,90,0)
    movej(P0, VEL, ACC)
    task_compliance_ctrl([500, 500, 500, 100, 100, 100])
    print("순응제어 30초간 시작")
    wait(30.0)
    release_compliance_ctrl()

    print("예제 프로그램이 종료되었습니다.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()