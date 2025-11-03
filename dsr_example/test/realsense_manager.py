#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class RealSenseManager:
    def __init__(self, node=None):
        self.node = node
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)

        # ✅ RealSense 설정
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # ✅ 파이프라인 시작
        profile = self.pipeline.start(config)
        self.node.get_logger().info("📷 RealSense 파이프라인 시작됨")

        # ✅ Intrinsics 읽기 (Color 스트림 기준)
        color_stream = profile.get_stream(rs.stream.color)
        self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        self.node.get_logger().info(
            f"🎯 Intrinsics 읽기 완료: fx={self.intrinsics.fx:.1f}, fy={self.intrinsics.fy:.1f}, "
            f"ppx={self.intrinsics.ppx:.1f}, ppy={self.intrinsics.ppy:.1f}"
        )

        self.color_frame = None
        self.depth_frame = None

    # ✅ Intrinsics getter
    def get_intrinsics(self):
        return self.intrinsics

    # ✅ 최신 프레임 가져오기 (Color + Depth)
    def get_latest_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        self.color_frame = color_image
        self.depth_frame = depth_image
        return color_image, depth_image

    # ✅ 중심부 깊이 값 반환
    def get_center_depth(self):
        if self.depth_frame is None:
            return None
        h, w = self.depth_frame.shape
        return float(self.depth_frame[h//2, w//2])

    # ✅ 종료 처리
    def stop(self):
        try:
            self.pipeline.stop()
            if self.node:
                self.node.get_logger().info("🧹 RealSense 파이프라인 종료")
        except Exception as e:
            print(f"⚠️ RealSense 종료 중 오류: {e}")
