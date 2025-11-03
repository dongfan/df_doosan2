from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    """
    Smart Refueler 통합 런치 템플릿
    ⚙️ 현재는 각 노드 실행 순서만 정의해둔 기본 형태입니다.
    실제 테스트 완료 후 개별 노드를 주석 해제하여 통합 실행하세요.
    """
    Node(package='dsr_example', executable='webcam_manager_ros', output='screen'),

    TimerAction(period=4.0, actions=[
        Node(package='dsr_example', executable='realsense_manager_ros', output='screen')
    ]),

    TimerAction(period=8.0, actions=[
        Node(package='dsr_example', executable='vision_target_node', output='screen')
    ]),
    return LaunchDescription([
        # === CAMERA NODES ===
        Node(
            package='dsr_example',
            executable='webcam_manager_ros',
            name='webcam_manager_ros',
            output='screen'
        ),
        Node(
            package='dsr_example',
            executable='realsense_manager_ros',
            name='realsense_manager_ros',
            output='screen'
        ),

        # === VISION + MOTION ===
        Node(
            package='dsr_example',
            executable='vision_target_node',
            name='vision_target_node',
            output='screen'
        ),
        Node(
            package='dsr_example',
            executable='motion_controller',
            name='motion_controller',
            output='screen'
        ),

        # === FASTAPI Bridge ===
        Node(
            package='dsr_example',
            executable='fuel_listener_node',
            name='fuel_listener_node',
            output='screen'
        )
    ])
