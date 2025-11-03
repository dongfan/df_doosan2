import os
import cv2
import torch
import numpy as np
from ultralytics import YOLO  # pip install ultralytics
from ament_index_python.packages import get_package_share_directory

class YoloDetector:
    def __init__(self, conf=0.6):
        package_share = get_package_share_directory("dsr_example")  # 패키지명
        model_path = os.path.join(package_share, "weights", "best.pt")

        self.model = YOLO(model_path)
        self.conf = conf

    def detect(self, frame):
        results = self.model.predict(frame, conf=self.conf, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls)
                conf = float(box.conf)
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                detections.append({
                    "cls": self.model.names[cls_id],
                    "conf": conf,
                    "bbox": (x1, y1, x2, y2)
                })
        return detections

    def draw_detections(self, frame, detections):
        annotated = frame.copy()
        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            cls_name = det["cls"]
            conf = det["conf"]

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{cls_name} {conf:.2f}"
            cv2.putText(annotated, label, (x1, max(20, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        return annotated
