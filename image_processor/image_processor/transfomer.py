from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

from util import exceptions

class Transformer:
    def __init__(self):
        # 모델 파일 경로 설정 및 로드
        # T_path = Path(__file__).parent.parent.parent.parent /"src" /"collaboration-2" /"image_processor" / "resource" / "T_gripper2camera.npy"
        T_path = "/home/moonseungyeon/ros2_ws/src/collaboration-2/image_processor/resource/T_gripper2camera.npy"
        # if not T_path.exists():
        #     raise FileNotFoundError(f"파일 경로가 존재하지 않음: {T_path}")
        
        self.T_gripper2camera = np.load(T_path)
        # print(self.T_gripper2camera)

    def camera2base(self, xyz, robot_pos):
        """
        Camera 좌표계 물체 좌표 (xyz) 를 로봇 Base 좌표계로 변환한다.
        """
        if robot_pos is None:
            raise exceptions.IMAGE_PROCESSOR_ERROR(104)

        base2gripper = self.base2gripper(robot_pos)
        gripper2camera = self.T_gripper2camera

        camera2target = np.eye(4)
        camera2target[:3, 3] = xyz  # rotation은 필요 없으므로 translation만

        base2target = base2gripper @ gripper2camera @ camera2target

        return [
            float(base2target[0, 3]),
            float(base2target[1, 3]),
            float(base2target[2, 3]),
            # 90.0, 180.0, 180.0,
            89.99, 179.99, 179.99,
        ]
    
    def base2gripper(self, robot_pos):
        x, y, z, rx, ry, rz = robot_pos
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T