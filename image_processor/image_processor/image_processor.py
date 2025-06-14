import rclpy
from rclpy.node import Node
from cb_interfaces.srv import Target, TargetCoord

from util import config
from collections import deque

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.valid_targets = config.valid_targets
        self.pose_queue = deque()

        # 서비스 서버: Target
        self.create_service(Target, config.TARGET, self.handle_target_request)
        # 서비스 클라이언트: TargetCoord
        self.target_coord_client = self.create_client(TargetCoord, config.TARGET_COORD)
        # 타이머: 큐에 pose가 있으면 TargetCoord로 전달
        self.create_timer(0.1, self.process_pose_queue)

    def handle_target_request(self, request, response):
        self.get_logger().info(f"요청 받은 target_name: {request.target_name}")
        if request.target_name in self.valid_targets:
            pose = [472.49, 16.510, 50.42, 90.0, 180.0, 180.0]  # 예시 pose
            self.pose_queue.append(pose)
            
            response.found = True
            self.get_logger().info(f"pose 큐에 저장: {pose}")
            self.get_logger().info(f"Target {request.target_name} FOUND")
        else:
            response.found = False
            self.get_logger().info(f"Target {request.target_name} NOT FOUND")
        
        return response

    def process_pose_queue(self):
        # 큐가 비었으면 리턴
        if not self.pose_queue:
            # self.get_logger().info('pose_queue is empty')
            return
        self.get_logger().info('pose_queue has pose')
        
        # 서비스 서버가 없으면 리턴
        if not self.target_coord_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('TargetCoord 서비스 서버를 찾을 수 없습니다.')
            return
        self.get_logger().info('service server is running')

        coord_req = TargetCoord.Request()
        coord_req.pose = self.pose_queue.popleft()
        future = self.target_coord_client.call_async(coord_req)
        future.add_done_callback(self.target_coord_done_callback)
        self.get_logger().info("service is done")
    
    def target_coord_done_callback(self, future):
        if future.done() and future.result() is not None:
            result = future.result()
            self.get_logger().info(f"[RESPONSE] TargetCoord 응답: succeed={result.succeed}")
        else:
            self.get_logger().error("[RESPONSE] TargetCoord 서비스 호출 실패 또는 타임아웃")

def main():
    rclpy.init()
    ip = ImageProcessor()

    try:
        rclpy.spin(ip)
    except KeyboardInterrupt:
        pass
    finally:
        ip.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()