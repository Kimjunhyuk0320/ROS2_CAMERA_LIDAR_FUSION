#!/usr/bin/env python3  # ✅ Python3 인터프리터에서 실행하도록 설정

# ✅ 필수 라이브러리 임포트
import os  # 파일 및 디렉토리 조작 (파일 존재 여부 확인, 디렉토리 생성 등)
import yaml  # YAML 파일을 읽고 쓰기 위한 라이브러리
import numpy as np  # 수치 연산을 위한 NumPy 라이브러리
import cv2  # OpenCV 라이브러리 (solvePnP 활용)
from rclpy.node import Node  # ROS2 노드 클래스
import rclpy  # ROS2 기본 라이브러리

# ✅ 사용자 정의 모듈 (설정 파일을 읽어오기 위한 함수)
from ros2_camera_lidar_fusion.read_yaml import extract_configuration  

class CameraLidarExtrinsicNode(Node):
    """📌 Camera-LiDAR Extrinsic Calibration을 수행하는 ROS2 노드"""

    def __init__(self):
        super().__init__('camera_lidar_extrinsic_node')  # ✅ ROS2 노드 생성

        # ✅ 설정 파일 로드
        config_file = extract_configuration()
        if config_file is None:
            self.get_logger().error("Failed to extract configuration file.")  # ❌ 설정 파일을 찾을 수 없으면 오류 출력
            return  # 노드 실행 중지

        # ✅ 설정 파일에서 각 파일 경로 로드
        self.corr_file = config_file['general']['correspondence_file']  # 2D-3D 대응점 데이터 파일
        self.corr_file = f'/home/junhyuk/ros2_ws/src/ROS2_CAMERA_LIDAR_FUSION/data/{self.corr_file}'  # 전체 경로 구성

        self.camera_yaml = config_file['general']['camera_intrinsic_calibration']  # 카메라 내부 파라미터 파일
        self.camera_yaml = f'/home/junhyuk/ros2_ws/src/ROS2_CAMERA_LIDAR_FUSION/config/{self.camera_yaml}'  # 전체 경로 구성

        self.output_dir = config_file['general']['config_folder']  # Extrinsic 매트릭스를 저장할 폴더
        self.file = config_file['general']['camera_extrinsic_calibration']  # 저장할 Extrinsic 매트릭스 파일 이름

        self.get_logger().info('Starting extrinsic calibration...')  # ✅ 로그 출력
        self.solve_extrinsic_with_pnp()  # ✅ Extrinsic Calibration 실행

    def load_camera_calibration(self, yaml_path: str):
        """📌 카메라 캘리브레이션 파일을 로드하는 함수"""
        if not os.path.isfile(yaml_path):  # 🔍 파일이 존재하는지 확인
            raise FileNotFoundError(f"Calibration file not found: {yaml_path}")

        # ✅ YAML 파일 읽기
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        # ✅ 카메라 내부 행렬 및 왜곡 계수 추출
        mat_data = config['camera_matrix']['data']
        camera_matrix = np.array(mat_data, dtype=np.float64)  # 내부 행렬 (3x3)

        dist_data = config['distortion_coefficients']['data']
        dist_coeffs = np.array(dist_data, dtype=np.float64).reshape((1, -1))  # 왜곡 계수 (1x5)

        return camera_matrix, dist_coeffs  # ✅ 반환 (카메라 내부 행렬, 왜곡 계수)

    def solve_extrinsic_with_pnp(self):
        """📌 SolvePnP를 이용한 Camera-LiDAR Extrinsic Calibration 수행"""

        # ✅ 1. 카메라 내부 파라미터 로드
        camera_matrix, dist_coeffs = self.load_camera_calibration(self.camera_yaml)
        self.get_logger().info(f"Camera matrix:\n{camera_matrix}")  # ✅ 로그 출력
        self.get_logger().info(f"Distortion coefficients: {dist_coeffs}")  # ✅ 로그 출력

        # ✅ 2. 2D-3D 대응점 데이터 파일 확인
        if not os.path.isfile(self.corr_file):
            raise FileNotFoundError(f"Correspondence file not found: {self.corr_file}")

        # ✅ 3. 2D-3D 매칭 데이터 로드
        pts_2d = []  # 2D 이미지 좌표 리스트
        pts_3d = []  # 3D LiDAR 좌표 리스트

        with open(self.corr_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):  # 🔍 주석(#)이나 빈 줄은 무시
                    continue

                splitted = line.split(',')  # ✅ CSV 형식의 데이터 읽기
                if len(splitted) != 5:  # 🔍 데이터가 5개 (u, v, X, Y, Z)인지 확인
                    continue

                u, v, X, Y, Z = [float(val) for val in splitted]  # ✅ 문자열 데이터를 실수로 변환
                pts_2d.append([u, v])  # 이미지 좌표 추가
                pts_3d.append([X, Y, Z])  # LiDAR 3D 좌표 추가

        # ✅ NumPy 배열로 변환
        pts_2d = np.array(pts_2d, dtype="double")
        pts_3d = np.array(pts_3d, dtype="double")
        # ✅ 로그로 출력
        self.get_logger().info(f"📌 2D Points (Image Coordinates):\n{np.array2string(pts_2d, precision=4, separator=', ')}")
        self.get_logger().info(f"📌 3D Points (LiDAR Coordinates):\n{np.array2string(pts_3d, precision=4, separator=', ')}")

        num_points = len(pts_2d)  # ✅ 대응점 개수 확인
        self.get_logger().info(f"Loaded {num_points} correspondences from {self.corr_file}")

        if num_points < 4:  # 🔍 SolvePnP 실행을 위해 최소 4개 이상의 대응점 필요
            raise ValueError("At least 4 correspondences are required for solvePnP")

        # ✅ 4. SolvePnP 실행 (Extrinsic Calibration)
        success, rvec, tvec = cv2.solvePnP(
            pts_3d,  # 3D LiDAR 포인트
            pts_2d,  # 2D 이미지 포인트
            camera_matrix,  # 카메라 내부 행렬
            dist_coeffs,  # 왜곡 계수
            flags=cv2.SOLVEPNP_ITERATIVE  # 반복 알고리즘을 사용
        )

        if not success:
            raise RuntimeError("solvePnP failed to find a solution.")  # ❌ SolvePnP 실패 시 오류 발생

        self.get_logger().info("solvePnP succeeded.")  # ✅ 로그 출력
        self.get_logger().info(f"rvec: {rvec.ravel()}")  # ✅ 회전 벡터 출력
        self.get_logger().info(f"tvec: {tvec.ravel()}")  # ✅ 변환 벡터 출력

        # ✅ 5. 회전 벡터(rvec) → 회전 행렬(R) 변환
        R, _ = cv2.Rodrigues(rvec)

        # ✅ 6. LiDAR → Camera 변환 행렬 생성
        T_lidar_to_cam = np.eye(4, dtype=np.float64)  # 4x4 단위 행렬 생성
        T_lidar_to_cam[0:3, 0:3] = R  # 회전 행렬 적용
        T_lidar_to_cam[0:3, 3] = tvec[:, 0]  # 변환 벡터 적용

        self.get_logger().info(f"Transformation matrix (LiDAR -> Camera):\n{T_lidar_to_cam}")

        # ✅ 7. 변환 행렬을 YAML 파일로 저장
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        out_yaml = os.path.join(self.output_dir, self.file)  # ✅ 저장 경로 설정
        data_out = {"extrinsic_matrix": T_lidar_to_cam.tolist()}  # YAML 저장 형식

        with open(out_yaml, 'w') as f:
            yaml.dump(data_out, f, sort_keys=False)

        self.get_logger().info(f"Extrinsic matrix saved to: {out_yaml}")

# ✅ 메인 함수 (ROS2 노드 실행)
def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarExtrinsicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
