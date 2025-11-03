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
        movej, set_robot_mode, 
        ROBOT_MODE_AUTONOMOUS, 
        task_compliance_ctrl, release_compliance_ctrl, wait, set_desired_force,
        release_force
    ) 

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    x0 = [0, 0, 90, 0, 90, 0]
    movej(x0, VEL, ACC)
    wait(0.5)
    task_compliance_ctrl([500, 500, 500, 300, 300, 300])

    fd = [0, 0, -10, 0, 0, 0]
    fctrl_dir= [0, 0, 1, 0, 0, 0]
    set_desired_force(fd, dir=fctrl_dir)
    print("10초간 강성제어 시작")
    wait(10.0)
    
    release_force()
    release_compliance_ctrl()

    movej(x0, VEL, ACC)
    wait(0.5)

    print("예제 프로그램이 종료되었습니다.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()