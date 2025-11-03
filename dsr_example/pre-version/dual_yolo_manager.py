#!/usr/bin/env python3
import os
import cv2
import numpy as np
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class DualYoloManager:
    """
    두 YOLO 모델(Webcam + RealSense)을 모두 Detection 전용으로 관리.
    각 카메라 입력 이미지를 받아 객체 bbox와 cls를 반환.
    """

    def __init__(self, conf_det=0.6):
        package_share = get_package_share_directory("dsr_example")
        weights_dir = os.path.join(package_share, "weights")

        # ✅ 두 카메라 모두 같은 Detection 모델 사용
        webcam_model_path = os.path.join(weights_dir, "best_3.pt")
        realsense_model_path = os.path.join(weights_dir, "best_3.pt")

        self.webcam_model = YOLO(webcam_model_path)
        self.realsense_model = YOLO(realsense_model_path)

        self.conf_det = conf_det

        print(f"✅ Webcam YOLO (Detection): {webcam_model_path}")
        print(f"✅ RealSense YOLO (Detection): {realsense_model_path}")

    # ----------------------------
    # 🎯 Webcam 객체 탐지
    # ----------------------------
    def detect_webcam_objects(self, frame):
        results = self.webcam_model.predict(frame, conf=self.conf_det, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls)
                conf = float(box.conf)
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                detections.append({
                    "cls": self.webcam_model.names[cls_id],
                    "conf": conf,
                    "bbox": (x1, y1, x2, y2)
                })
        return detections

    # ----------------------------
    # 🎯 RealSense 객체 탐지
    # ----------------------------
    def detect_realsense_objects(self, frame):
        results = self.realsense_model.predict(frame, conf=self.conf_det, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls)
                conf = float(box.conf)
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                detections.append({
                    "cls": self.realsense_model.names[cls_id],
                    "conf": conf,
                    "bbox": (x1, y1, x2, y2)
                })
        return detections

    # ----------------------------
    # 🖼️ 시각화 (Bounding Box)
    # ----------------------------
    def draw_detections(self, frame, detections, color=(0, 255, 0)):
        annotated = frame.copy()
        for det in detections:
            if "bbox" in det:
                x1, y1, x2, y2 = det["bbox"]
                cls_name = det["cls"]
                conf = det["conf"]
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                label = f"{cls_name} {conf:.2f}"
                cv2.putText(annotated, label, (x1, max(20, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        return annotated
