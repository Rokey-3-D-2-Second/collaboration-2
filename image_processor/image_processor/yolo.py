from pathlib import Path
from ultralytics import YOLO
import yaml
import numpy as np
import os
from collections import Counter

from util import exceptions

# ultralytics 내부 로그 레벨을 ERROR로 올림
import logging
logging.getLogger("ultralytics").setLevel(logging.ERROR)

class Yolo:
    def __init__(self):
        # 모델 파일 경로 설정 및 로드
        # model_path = Path(__file__).parent.parent.parent.parent /"src" /"collaboration-2" /"image_processor" / "resource" / "yolo11n.pt"
        model_path = os.path.expanduser("~/ros2_ws/src/collaboration-2/image_processor/resource/yolo11n.pt")
        # if not model_path.exists():
        #     raise FileNotFoundError(f"모델 경로가 존재하지 않음: {model_path}")
        
        # label_path = Path(__file__).parent.parent.parent.parent /"src" /"collaboration-2" /"image_processor" / "resource" / "coco.yaml"
        label_path = os.path.expanduser("~/ros2_ws/src/collaboration-2/image_processor/resource/coco.yaml")
        # if not label_path.exists():
        #     raise FileNotFoundError(f"라벨 경로가 존재하지 않음: {label_path}")

        self.model = YOLO(model_path, verbose=False)

        with open(label_path, "r") as f:
            data = yaml.safe_load(f)
        self.label = data.get('names', {})

    def detect_target(self, image, target_name):
        """
        입력 이미지에서 YOLO로 객체 검출 후,
        지정한 클래스(target_name)에 대해 가장 신뢰도 높은 박스 반환
        """
        results = self.model(image)  # YOLO 추론 결과
        detections = self._aggregate_detections(results)  # 박스 통합
        return self._get_best_detection(detections, target_name)  # 최고 score 박스 반환

    def _aggregate_detections(
            self, 
            results, 
            confidence_threshold=0.4, 
            iou_threshold=0.4
    ):
        """
        여러 프레임의 YOLO 결과를 IoU 기반으로 그룹핑하여
        평균 박스를 산출 (노이즈 완화, 신뢰도 향상)
        """
        raw = []
        # 각 결과에서 confidence가 일정 이상인 박스만 추출
        for res in results:
            for box, score, label in zip(
                res.boxes.xyxy.tolist(),
                res.boxes.conf.tolist(),
                res.boxes.cls.tolist(),
            ):
                if score >= confidence_threshold:
                    raw.append({"box": box, "score": score, "label": int(label)})
        # print(f"_aggregate_detections - raw: {raw}")

        final = []
        used = [False] * len(raw)
        # IoU가 일정 이상인 박스끼리 그룹핑
        for i, det in enumerate(raw):
            if used[i]: continue

            group = [det]
            used[i] = True
            for j, other in enumerate(raw):
                if not used[j] and other["label"] == det["label"]:
                    if self._iou(det["box"], other["box"]) >= iou_threshold:
                        group.append(other)
                        used[j] = True

            # 그룹 내 박스 좌표/score 평균, label은 다수결
            boxes = np.array([g["box"] for g in group])
            scores = np.array([g["score"] for g in group])
            labels = [g["label"] for g in group]

            final.append(
                {
                    "box": boxes.mean(axis=0).tolist(),
                    "score": float(scores.mean()),
                    "label": Counter(labels).most_common(1)[0][0],
                }
            )

        # print(f"_aggregate_detections - final: {final}")
        return final

    def _iou(self, box1, box2):
        # print("_iou")
        """
        두 박스의 IoU(Intersection over Union) 계산
        """
        x1, y1 = max(box1[0], box2[0]), max(box1[1], box2[1])
        x2, y2 = min(box1[2], box2[2]), min(box1[3], box2[3])
        inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter
        return inter / union if union > 0 else 0.0
    
    def _get_best_detection(self, detections, target_name):
        """
        지정한 클래스(target_name)에 대해 score가 가장 높은 박스 반환
        matches에는 label을 이름(str)으로 변환해서 저장
        """
        matches = [
            {**d, "label": self.label[d["label"]]}  # label을 이름(str)으로 변환
            for d in detections if self.label[d["label"]] == target_name
        ]
        if not matches:
            raise exceptions.IMAGE_PROCESSOR_ERROR(101)
        
        # print(f"_get_best_detection: {matches}")
        return max(matches, key=lambda x: x["score"])
