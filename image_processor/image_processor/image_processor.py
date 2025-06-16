import rclpy, os, sys, cv2, numpy as np
from rclpy.node import Node
from cb_interfaces.srv import Target, TargetCoord
from sensor_msgs.msg import Image, CameraInfo, CompressedImage, JointState
from cv_bridge import CvBridge

from .yolo import Yolo
from .transfomer import Transformer

from util import config, exceptions
# from ros2_controller import mover
from collections import deque

from dsr_msgs2.srv import GetCurrentPosx
import DR_init
DR_init.__dsr__id = config.ROBOT_ID
DR_init.__dsr__model = config.ROBOT_MODEL

# robot_pos = None

class ImageProcessor(Node):
# class ImageProcessor:
    def __init__(self, ):
        super().__init__('image_processor')

        # self.node: Node = node
        
        self.yolo = Yolo()
        self.tf = Transformer()
        self.bridge = CvBridge()
        self.intrinsics = None  # 카메라 내부 파라미터
        self.color_image = None # 컬러 이미지
        self.depth_image = None # 깊이 이미지
        
        qos = 10  # QoS 깊이 (구독 품질 설정)

        # self.create_subscription(JointState, "/dsr01/joint_states", self.joint_cb, qos)
        # self.get_current_posx = get_current_posx

        # RealSense 카메라의 토픽 구독 설정
        self.create_subscription(CameraInfo, config.CAMERA_INFO, self.info_cb, qos)         # 카메라 내부 파라미터
        self.create_subscription(CompressedImage, config.COLOR_IMAGE, self.color_cb, qos)   # 컬러 영상 (압축)
        self.create_subscription(Image, config.ALIGNED_IMAGE, self.depth_cb, qos)           # 깊이 영상 (컬러와 정렬됨)

        self.before_tf = None
        self.after_tf = None
        self.pose_queue = deque()

        # 서비스 서버: Target
        # self.create_service(Target, config.TARGET, self.handle_target_request)
        self.create_service(Target, config.TARGET, self.handle_target)

        # 서비스 클라이언트: TargetCoord
        self.target_coord_client = self.create_client(TargetCoord, config.TARGET_COORD)
        self.get_current_posx = self.create_client(GetCurrentPosx, "/dsr01/aux_control/get_current_posx")

        # 타이머: 큐에 pose가 있으면 TargetCoord로 전달
        # self.create_timer(0.1, self.process_pose_queue)

        # CV
        cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow("YOLO Detection", 640, 480)

        # 타이머: 주기적으로 detection 결과를 OpenCV로 출력
        self.create_timer(0.05, self.visualize_detection)

        self.create_timer(1.0, self.get_current_pose)

    # def joint_cb(self, msg: JointState):
    #     self.robot_pos = msg.position

    def get_current_pose(self):
        req = GetCurrentPosx.Request()  
        req.ref = 0

        future = self.get_current_posx.call_async(req)
        future.add_done_callback(self.set_current_pose)

    def set_current_pose(self, future):
        if future.done() and future.result() is not None:
            result = future.result()

        if not result.task_pos_info or len(result.task_pos_info) == 0:
            self.get_logger().warn("result.task_pos_info가 비어 있습니다.")
            return
        
        posx_info = result.task_pos_info[0].data
        pos = []
        for i in range(6):
            pos.append(posx_info[i])
        sol = int(round( posx_info[6] ))

        self.robot_pos = pos

    def info_cb(self, msg: CameraInfo):
        # 카메라 내부 파라미터 콜백 (intrinsics 행렬에서 fx, fy, cx, cy 추출)
        self.intrinsics = {
            "fx": msg.k[0],
            "fy": msg.k[4],
            "cx": msg.k[2],
            "cy": msg.k[5]
        }

    def color_cb(self, msg: CompressedImage):
        # 컬러 이미지 콜백 - 압축된 이미지 디코딩
        self.color_image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)

    def depth_cb(self, msg: Image):
        # 깊이 이미지 콜백 - ROS Image 메시지를 OpenCV 형식으로 변환
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def visualize_detection(self):
        # 컬러 이미지가 없으면 아무것도 하지 않음
        if self.color_image is None:
            return
        img = self.color_image.copy()

        try:
            # YOLO 추론을 항상 실행
            results = self.yolo.model(self.color_image)

            for res in results:
                for box, score, label in zip(
                    res.boxes.xyxy.tolist(),
                    res.boxes.conf.tolist(),
                    res.boxes.cls.tolist(),
                ):
                    if score < 0.5:
                        continue
                    x1, y1, x2, y2 = map(int, box)
                    class_name = self.yolo.label[label] if hasattr(self.yolo, "label") else str(label)
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(img, f"{class_name} {score:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        except Exception as e:
            self.get_logger().warn(f"YOLO 시각화 오류: {e}")

        cv2.imshow("YOLO Detection", img)
        cv2.waitKey(1)

    def handle_target(self, request, response):
        # request 수신 
        target_name = request.target_name
        self.get_logger().info(f"요청 받은 target_name: {target_name}")

        try:
            response.found = True

            # 1. 컬러/깊이/내부파라미터 준비 확인
            if self.color_image is None or self.depth_image is None or self.intrinsics is None:
                raise exceptions.IMAGE_PROCESSOR_ERROR(100)
            self.get_logger().info("이미지/깊이/내부파라미터 수신")
            
            # 2. Object Detection
            detection = self.yolo.detect_target(self.color_image, target_name)
            self.get_logger().info(f"YOLO 검출: {target_name}/{detection['label']}, conf: {detection['score']}")

            # 3. Coord
            before_tf = self.get_coord(detection)
            self.get_logger().info(f"before_tf: {before_tf}")

            # 3. Transformation(camera to tcp)
            # after_tf = self.tf.camera2base(before_tf, self.robot_pos)
            after_tf = self.tf.camera2base(before_tf, self.robot_pos)
            self.get_logger().info(f"after_tf: {after_tf}")

            # 4. target_coord_client 비동기 통신으로 pose 전달
            coord_req = TargetCoord.Request()
            coord_req.pose = after_tf
            future = self.target_coord_client.call_async(coord_req)
            future.add_done_callback(self.target_coord_done_callback)

        except Exception as e:
            response.found = False
            self.get_logger().error(str(e))
            # self.exit()
        finally:
            return response
    
    def get_coord(self, detection):
        # bbox 중심 좌표 계산
        x1, y1, x2, y2 = detection["box"]
        cx, cy = map(int, [(x1 + x2) / 2, (y1 + y2) / 2])
        Z = self.depth_image[cy, cx]
        if Z <= 0:
            raise exceptions.IMAGE_PROCESSOR_ERROR(103)
        self.get_logger().info("유효한 깊이 값")

        # 좌표 계산
        fx, fy = self.intrinsics["fx"], self.intrinsics["fy"]
        cx0, cy0 = self.intrinsics["cx"], self.intrinsics["cy"]
        X = (cx - cx0) * Z / fx
        Y = (cy - cy0) * Z / fy

        return [X, Y, Z]

    def target_coord_done_callback(self, future):
        if future.done() and future.result() is not None:
            result = future.result()
            self.get_logger().info(f"[RESPONSE] TargetCoord 응답: succeed={result.succeed}")
        else:
            self.get_logger().error("[RESPONSE] TargetCoord 서비스 호출 실패 또는 타임아웃")

    def exit(self):
        raise exceptions.IMAGE_PROCESSOR_ERROR(106)

    # def handle_target_request(self, request, response):
    #     self.get_logger().info(f"요청 받은 target_name: {request.target_name}")
    #     if request.target_name in self.valid_targets:
    #         pose = [472.49, 16.510, 50.42, 90.0, 180.0, 180.0]  # 예시 pose
    #         self.pose_queue.append(pose)
            
    #         response.found = True
    #         self.get_logger().info(f"pose 큐에 저장: {pose}")
    #         self.get_logger().info(f"Target {request.target_name} FOUND")
    #     else:
    #         response.found = False
    #         self.get_logger().info(f"Target {request.target_name} NOT FOUND")
        
    #     return response

    # def process_pose_queue(self):
    #     # 큐가 비었으면 리턴
    #     if not self.pose_queue:
    #         # self.get_logger().info('pose_queue is empty')
    #         return
    #     self.get_logger().info('pose_queue has pose')
        
    #     # 서비스 서버가 없으면 리턴
    #     if not self.target_coord_client.wait_for_service(timeout_sec=2.0):
    #         self.get_logger().error('TargetCoord 서비스 서버를 찾을 수 없습니다.')
    #         return
    #     self.get_logger().info('service server is running')

    #     coord_req = TargetCoord.Request()
    #     coord_req.pose = self.pose_queue.popleft()
    #     future = self.target_coord_client.call_async(coord_req)
    #     future.add_done_callback(self.target_coord_done_callback)
    #     self.get_logger().info("service is done")
    
    # def target_coord_done_callback(self, future):
    #     if future.done() and future.result() is not None:
    #         result = future.result()
    #         self.get_logger().info(f"[RESPONSE] TargetCoord 응답: succeed={result.succeed}")
    #     else:
    #         self.get_logger().error("[RESPONSE] TargetCoord 서비스 호출 실패 또는 타임아웃")

# def get_current_pose(get_current_posx):
#     global robot_pos
#     robot_pos = get_current_posx()[0]

def main():
    rclpy.init()
    ip = ImageProcessor()

    try:
        rclpy.spin(ip)
    except KeyboardInterrupt:
        pass
    except exceptions.IMAGE_PROCESSOR_ERROR as e:
        print(str(e))
    finally:
        cv2.destroyAllWindows()
        ip.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()