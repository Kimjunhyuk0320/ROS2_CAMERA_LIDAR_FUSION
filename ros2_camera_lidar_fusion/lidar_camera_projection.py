#!/usr/bin/env python3  # ✅ Python3 인터프리터 사용 선언

# ✅ 필수 라이브러리 임포트
import os  # 파일 및 디렉토리 작업을 위한 라이브러리
import rclpy  # ROS2 기본 라이브러리
from rclpy.node import Node  # ROS2 노드 클래스

import cv2  # OpenCV 라이브러리 (이미지 처리)
import numpy as np  # NumPy (수학 연산)
import yaml  # YAML 파일 입출력을 위한 라이브러리
import struct  # 바이너리 데이터 처리 라이브러리

# ✅ ROS2 메시지 관련 라이브러리
from sensor_msgs.msg import Image, PointCloud2  # ROS2 메시지 타입 (이미지, 포인트 클라우드)
from cv_bridge import CvBridge  # ROS Image ↔ OpenCV 변환
from message_filters import Subscriber, ApproximateTimeSynchronizer  # 메시지 동기화

# ✅ 설정 파일을 로드하는 함수 (사용자 정의 모듈)
from ros2_camera_lidar_fusion.read_yaml import extract_configuration


# ✅ Extrinsic 변환 행렬을 YAML 파일에서 로드하는 함수
def load_extrinsic_matrix(yaml_path: str) -> np.ndarray:
    """YAML 파일에서 4x4 Extrinsic 변환 행렬을 로드"""
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"No extrinsic file found: {yaml_path}")

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    if 'extrinsic_matrix' not in data:
        raise KeyError(f"YAML {yaml_path} has no 'extrinsic_matrix' key.")

    matrix_list = data['extrinsic_matrix']
    T = np.array(matrix_list, dtype=np.float64)

    if T.shape != (4, 4):
        raise ValueError("Extrinsic matrix is not 4x4.")

    return T  # ✅ LiDAR -> Camera 변환 행렬 반환


# ✅ 카메라 내부 행렬 및 왜곡 계수 로드 함수
def load_camera_calibration(yaml_path: str) -> (np.ndarray, np.ndarray):
    """YAML 파일에서 카메라 내부 행렬과 왜곡 계수를 로드"""
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"No camera calibration file: {yaml_path}")

    with open(yaml_path, 'r') as f:
        calib_data = yaml.safe_load(f)

    cam_mat_data = calib_data['camera_matrix']['data']
    camera_matrix = np.array(cam_mat_data, dtype=np.float64)  # 3x3 카메라 내부 행렬

    dist_data = calib_data['distortion_coefficients']['data']
    dist_coeffs = np.array(dist_data, dtype=np.float64).reshape((1, -1))  # 1xN 왜곡 계수

    return camera_matrix, dist_coeffs  # ✅ 반환 (카메라 내부 행렬, 왜곡 계수)


# ✅ PointCloud2 메시지를 NumPy 배열로 변환하는 함수
def pointcloud2_to_xyz_array_fast(cloud_msg: PointCloud2, skip_rate: int = 1) -> np.ndarray:
    """ROS2 PointCloud2 메시지를 NumPy 배열 (N x 3)로 변환"""
    if cloud_msg.height == 0 or cloud_msg.width == 0:
        return np.zeros((0, 3), dtype=np.float32)

    field_names = [f.name for f in cloud_msg.fields]
    if not all(k in field_names for k in ('x', 'y', 'z')):  # x, y, z 필드가 있는지 확인
        return np.zeros((0, 3), dtype=np.float32)

    dtype = np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('_', 'V{}'.format(cloud_msg.point_step - 12))  # ✅ 추가 필드 무시
    ])

    raw_data = np.frombuffer(cloud_msg.data, dtype=dtype)
    points = np.vstack((raw_data['x'], raw_data['y'], raw_data['z'])).T  # ✅ NumPy 배열 변환

    if skip_rate > 1:
        points = points[::skip_rate]  # ✅ 샘플링 (skip_rate 간격으로 점 선택)

    return points  # ✅ 변환된 포인트 클라우드 반환


# ✅ ROS2 노드 정의 (LiDAR → 카메라 투영)
class LidarCameraProjectionNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_projection_node')  # ✅ ROS2 노드 생성

        config_file = extract_configuration()
        if config_file is None:
            self.get_logger().error("Failed to extract configuration file.")
            return
        
        # ✅ YAML 설정 로드 (Extrinsic 변환 행렬 & 카메라 내부 행렬)
        config_folder = config_file['general']['config_folder']
        extrinsic_yaml = os.path.join(config_folder, config_file['general']['camera_extrinsic_calibration'])
        self.T_lidar_to_cam = load_extrinsic_matrix(extrinsic_yaml)

        camera_yaml = os.path.join(config_folder, config_file['general']['camera_intrinsic_calibration'])
        self.camera_matrix, self.dist_coeffs = load_camera_calibration(camera_yaml)

        # ✅ 설정 정보 출력
        self.get_logger().info("Loaded extrinsic:\n{}".format(self.T_lidar_to_cam))
        self.get_logger().info("Camera matrix:\n{}".format(self.camera_matrix))
        self.get_logger().info("Distortion coeffs:\n{}".format(self.dist_coeffs))

        # ✅ ROS2 토픽 구독 설정
        lidar_topic = config_file['lidar']['lidar_topic']
        image_topic = config_file['camera']['image_topic']

        self.image_sub = Subscriber(self, Image, image_topic)
        self.lidar_sub = Subscriber(self, PointCloud2, lidar_topic)

        # ✅ ApproximateTimeSynchronizer로 두 개의 센서 데이터 동기화
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub], queue_size=5, slop=0.07)
        self.ts.registerCallback(self.sync_callback)

        # ✅ 변환된 이미지를 퍼블리시할 토픽
        projected_topic = config_file['camera']['projected_topic']
        self.pub_image = self.create_publisher(Image, projected_topic, 1)

        self.bridge = CvBridge()
        self.skip_rate = 1  # ✅ LiDAR 포인트 샘플링 비율

    def sync_callback(self, image_msg: Image, lidar_msg: PointCloud2):
        """📌 LiDAR → 카메라 프레임 변환 후 이미지에 투영"""

        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        xyz_lidar = pointcloud2_to_xyz_array_fast(lidar_msg, skip_rate=self.skip_rate)
        if xyz_lidar.shape[0] == 0:
            self.get_logger().warn("Empty cloud. Nothing to project.")
            return

        # ✅ 4x4 변환 행렬을 사용하여 LiDAR 포인트를 카메라 좌표계로 변환
        xyz_lidar_h = np.hstack((xyz_lidar.astype(np.float64), np.ones((xyz_lidar.shape[0], 1), dtype=np.float64)))
        xyz_cam = (xyz_lidar_h @ self.T_lidar_to_cam.T)[:, :3]

        # ✅ 카메라 앞쪽(z > 0) 포인트만 선택
        mask_in_front = (xyz_cam[:, 2] > 0.0)
        xyz_cam_front = xyz_cam[mask_in_front]
        if xyz_cam_front.shape[0] == 0:
            self.get_logger().info("No points in front of camera (z>0).")
            return

        # ✅ OpenCV projectPoints()로 3D → 2D 변환
        image_points, _ = cv2.projectPoints(xyz_cam_front, np.zeros((3,1)), np.zeros((3,1)), self.camera_matrix, self.dist_coeffs)
        image_points = image_points.reshape(-1, 2).astype(np.int32)

        # ✅ 투영된 점을 이미지에 그리기
        for (u, v) in image_points:
            if 0 <= u < cv_image.shape[1] and 0 <= v < cv_image.shape[0]:
                cv2.circle(cv_image, (u, v), 2, (0, 255, 0), -1)

        self.pub_image.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))


# ✅ ROS2 노드 실행
def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
