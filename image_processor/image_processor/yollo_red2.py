#!/usr/bin/env python3
"""
detect_yolo_realsense.py
────────────────────────────────────────────────────────
● RealSense 카메라로 컬러·깊이·CameraInfo 구독
● YOLOv8 추론 → BBox 검출 + “색상 스티커(오염)” 판정
   └ ROI 안 특정 색상 픽셀 비율이 임계값 이상이면 contaminated
● 중심 픽셀의 깊이 → 3‑D 카메라 좌표(mm) 계산
● 결과를 .npy로 저장 + OpenCV 시각화
● 클래스별 가장 가까운 점을 /detected_points/* 토픽으로 퍼블리시
"""

import os, cv2, numpy as np, rclpy
from rclpy.node             import Node
from sensor_msgs.msg        import Image, CameraInfo, CompressedImage
from cv_bridge              import CvBridge
from ultralytics            import YOLO
from geometry_msgs.msg      import PointStamped

# ───────── “색상 스티커” 판정 파라미터 ─────────
# 🔄 색상 변경 1 ─ HSV 범위 -------------------------------------------------
#   • 기본은 빨간색(H 0‑10, 160‑180°) 두 구간
#   • 파란색으로 바꿀 땐 H 90‑130° 정도 한 구간이면 충분
# ------------------------------------------------------------------------
HSV_LOWER1 = (  0,  15,  15)      # 빨강(저 H) 0‑10
HSV_UPPER1 = ( 10, 255, 255)
HSV_LOWER2 = (160,  30,  30)      # 빨강(고 H) 160‑180
HSV_UPPER2 = (180, 255, 255)

# # ───────── 파란 스티커 판정 파라미터 ───────── ★
# #   *Hue :   90‑130° 부근이 순청색 계열
# #   *Sat·Val 범위는 기존과 동일하게 넉넉히 잡음
# HSV_LOWER1 = ( 90,  50,  50)
# HSV_UPPER1 = (130, 255, 255)
# # 두 번째 범위는 필요 없지만, 기존 구현을 그대로 살리기 위해 동일 범위를 한 번 더 둠
# HSV_LOWER2 = HSV_LOWER1
# HSV_UPPER2 = HSV_UPPER1


CONTAM_THRESHOLD = 0.1         # ROI 내 색상 ≥ 0.15 % → 오염물
                                   # 필요시 값 조정

def color_ratio(img_bgr: np.ndarray, x1:int, y1:int, x2:int, y2:int) -> float:
    """
    ROI 안에서 ‘타깃 색상’ 픽셀 비율(0‑1)을 반환
    🔄 색상 변경 2 ─ 이 함수는 그대로 두고 HSV 범위만 바꾸면 됨
    """
    hsv   = cv2.cvtColor(img_bgr[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
    mask  = cv2.inRange(hsv, HSV_LOWER1, HSV_UPPER1) | \
            cv2.inRange(hsv, HSV_LOWER2, HSV_UPPER2)
    hit   = cv2.countNonZero(mask)
    total = mask.size
    return hit / total if total else 0.0

# ─────────────────────────────────────────────
class RealSenseYOLO(Node):
    def __init__(self):
        super().__init__("realsense_yolo_node")

        # 내부 버퍼
        self.bridge        = CvBridge()
        self.color_image   = None
        self.depth_image   = None
        self.intrinsics    = None

        # YOLO 모델
        self.model = YOLO("/home/moonseungyeon/Downloads/moon6.pt")

        # 타깃 클래스 + 오염(가상) 클래스
        self.target_classes = ["fork", "knife", "spoon", "contaminated"]
        self.en2ko = {"fork":"포크","knife":"나이프","spoon":"숟가락",
                      "contaminated":"오염물"}
        self.detection_counts = {c:0 for c in self.target_classes}
        self.detected_points  = {c:[] for c in self.target_classes}

        # ROS 통신
        qos = 10
        self.create_subscription(CompressedImage,
            "/camera/camera/color/image_raw/compressed",
            self.color_cb, qos)
        self.create_subscription(Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_cb,  qos)
        self.create_subscription(CameraInfo,
            "/camera/camera/color/camera_info",
            self.info_cb,   qos)
        self.point_pubs = {c: self.create_publisher(PointStamped,
                         f"/detected_points/{c}", 10)
                           for c in self.target_classes}

        # OpenCV 창
        cv2.namedWindow("YOLO", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow("YOLO", 640, 640)
        self.get_logger().info("✅ node ready")

    # ───────── 콜백들 ─────────
    def color_cb(self, msg: CompressedImage):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None: return
        self.color_image = img
        self.process_frame()               # 매 컬러 프레임마다 처리

    def depth_cb(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def info_cb(self, msg: CameraInfo):
        self.intrinsics = {"fx":msg.k[0], "fy":msg.k[4],
                           "cx":msg.k[2], "cy":msg.k[5]}

    # ───────── 메인 처리 루프 ─────────
    def process_frame(self):
        if any(v is None for v in (self.color_image,
                                   self.depth_image,
                                   self.intrinsics)):
            return

        # 감지 버퍼 초기화
        for c in self.target_classes:
            self.detection_counts[c] = 0
            self.detected_points[c].clear()

        # YOLO 추론
        preds    = self.model(self.color_image, verbose=False)[0]
        rendered = self.color_image.copy()
        h, w, _  = rendered.shape

        # bbox 순회
        for *xyxy, conf, cls_id in preds.boxes.data.cpu().numpy():
            x1, y1, x2, y2 = map(int, xyxy)
            cls_name = self.model.names[int(cls_id)]
            if cls_name not in ("spoon", "knife", "fork"):
                continue

            # bbox 좌표 보정
            x1, y1 = max(x1,0), max(y1,0)
            x2, y2 = min(x2,w), min(y2,h)
            if x2 <= x1 or y2 <= y1: continue

            # 픽셀 비율 계산
            ratio = color_ratio(self.color_image, x1, y1, x2, y2)
            contaminated = ratio >= CONTAM_THRESHOLD
            label  = "contaminated" if contaminated else cls_name

            # 🔄 색상 변경 3 ─ 테두리·텍스트 색상 ----------------------------
            #   • 기본: 빨간색 사람 눈에 잘 보이도록 (BGR = (0,0,255))
            #   • 파란색으로 바꿀 땐 (255,0,0)
            color  = (0,0,255) if contaminated else (255,255,255)
            # -------------------------------------------------------------

            # 로그
            self.get_logger().info(
                f"[{label}] bbox=({x1},{y1},{x2},{y2}) color_ratio={ratio*100:.2f}%")

            # 깊이 → 3‑D
            cx, cy = (x1+x2)//2, (y1+y2)//2
            Z = self.depth_image[cy, cx]
            if Z <= 0: continue
            fx, fy = self.intrinsics["fx"], self.intrinsics["fy"]
            cx0, cy0 = self.intrinsics["cx"], self.intrinsics["cy"]
            X = (cx - cx0)*Z/fx
            Y = (cy - cy0)*Z/fy

            # 버퍼 누적
            self.detection_counts[label] += 1
            self.detected_points[label].append([X,Y,Z])

            # 시각화
            cv2.rectangle(rendered, (x1,y1), (x2,y2), color, 2)
            cv2.putText(rendered, f"{label} {conf:.2f}",
                        (x1, max(y1-8,0)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 2)

        # 클래스별 가장 가까운 점 저장·퍼블리시
        for c in self.target_classes:
            pts = np.asarray(self.detected_points[c])
            f_all, f_clo = f"detected_points_{c}.npy", f"closest_{c}.npy"

            if pts.size:
                np.save(f_all, pts)
                np.save(f_clo, pts[pts[:,2].argmin()])

                pt = pts[pts[:,2].argmin()]
                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_color_optical_frame"
                msg.point.x, msg.point.y, msg.point.z = map(float, pt)
                self.point_pubs[c].publish(msg)

                self.get_logger().info(
                    f"{self.en2ko[c]} 감지 {len(pts)}개, "
                    f"가장 가까운 Z={pt[2]:.1f} mm")
            else:
                for f in (f_all, f_clo):
                    if os.path.exists(f): os.remove(f)

        cv2.imshow("YOLO", rendered)
        cv2.waitKey(1)

# ─────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYOLO()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
