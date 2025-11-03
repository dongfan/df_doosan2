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
        movej, movel, posj, 
        set_robot_mode, ROBOT_MODE_AUTONOMOUS
    ) 

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    P0 = posj(0,0,90,0,90,0)
    print("홈 위치로 이동합니다")
    movej(P0, VEL, ACC)

    print("예제 프로그램이 종료되었습니다.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()