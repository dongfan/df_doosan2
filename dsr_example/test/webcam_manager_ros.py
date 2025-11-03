# webcam_manager_ros.py
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

class WebcamManagerROS:
    def __init__(self, node, device_index=1):
        self.node = node
        self.bridge = CvBridge()
        self.cap = None

        try:
            cap = cv2.VideoCapture(device_index)
            if cap.isOpened():
                self.cap = cap
                self.node.get_logger().info(f"📷 WebcamManagerROS started (index={device_index})")
            else:
                raise RuntimeError()
        except Exception as e:
            self.node.get_logger().warn(f"⚠️ Webcam (index={device_index}) not available. Continuing without webcam.")
            self.cap = None  # 안전하게 비활성화

        self.image_pub = node.create_publisher(Image, "/fuel/image_result", qos_profile_sensor_data)

    def capture_and_publish(self):
        if self.cap is None:
            # 카메라가 없을 경우 그냥 None 반환
            return None

        ret, frame = self.cap.read()
        if not ret:
            self.node.get_logger().warn("⚠️ Failed to read from webcam.")
            return None

        # 정상 프레임이면 퍼블리시
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        return frame

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.node.get_logger().info("📷 Webcam released")