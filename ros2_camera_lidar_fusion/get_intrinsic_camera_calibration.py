#!/usr/bin/env python3

import rclpy
import os
import cv2
import yaml
import numpy as np
from datetime import datetime
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ros2_camera_lidar_fusion.read_yaml import extract_configuration

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')

        # ✅ YAML 설정 파일 로드
        config_file = extract_configuration()
        if config_file is None:
            self.get_logger().error("📌 설정 파일을 불러오지 못했습니다. 실행을 중단합니다.")
            return
        
        # ✅ 체스보드 패턴 및 캘리브레이션 설정
        self.chessboard_rows = config_file['chessboard']['pattern_size']['rows']
        self.chessboard_cols = config_file['chessboard']['pattern_size']['columns']
        self.square_size = config_file['chessboard']['square_size_meters']

        self.image_topic = config_file['camera']['image_topic']
        self.image_width = config_file['camera']['image_size']['width']
        self.image_height = config_file['camera']['image_size']['height']

        self.output_path = config_file['general']['config_folder']
        self.file = config_file['general']['camera_intrinsic_calibration']

        # ✅ 저장 폴더 확인 및 생성
        os.makedirs(self.output_path, exist_ok=True)

        # ✅ ROS2 이미지 구독
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.bridge = CvBridge()

        # ✅ 체스보드 감지를 위한 객체 및 이미지 포인트 리스트
        self.obj_points = []
        self.img_points = []

        self.objp = np.zeros((self.chessboard_rows * self.chessboard_cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_cols, 0:self.chessboard_rows].T.reshape(-1, 2)
        self.objp *= self.square_size

        self.get_logger().info("📌 Camera calibration node initialized. 이미지 수신 대기 중...")

    def image_callback(self, msg):
        """📷 ROS2 이미지 콜백 함수 (ZED 화면 실시간 표시)"""
        try:
            # ✅ ZED 카메라의 현재 프레임을 가져옴
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # ✅ 체스보드 감지 시도
            ret, corners = cv2.findChessboardCorners(gray, (self.chessboard_cols, self.chessboard_rows), None)

            if ret:
                # ✅ 코너를 더 정밀하게 보정
                refined_corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1),
                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )

                # ✅ 좌표 저장
                self.obj_points.append(self.objp)
                self.img_points.append(refined_corners)

                # ✅ 감지된 체스보드를 이미지 위에 표시
                cv2.drawChessboardCorners(cv_image, (self.chessboard_cols, self.chessboard_rows), refined_corners, ret)
                self.get_logger().info("✅ 체스보드 감지 완료! 데이터 추가됨.")


            # ✅ 실시간 화면 표시 (체스보드 감지 여부와 관계없이 항상 표시)
            cv2.imshow("ZED Camera View", cv_image)
            cv2.waitKey(10)  # 1ms마다 업데이트 (프레임 유지)
        except Exception as e:
            self.get_logger().error(f"❌ 이미지 처리 중 오류 발생: {e}")

    def save_calibration(self):
        """📌 캘리브레이션 데이터 저장"""
        if len(self.obj_points) < 10:
            print("❌ 캘리브레이션을 위한 이미지가 부족합니다. 최소 10장이 필요합니다.")  # ✅ get_logger() 대신 print 사용
            return

        try:
            # ✅ 카메라 캘리브레이션 수행
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                self.obj_points, self.img_points, (self.image_width, self.image_height), None, None
            )

            # ✅ 캘리브레이션 결과 저장할 데이터 구성
            calibration_data = {
                'calibration_date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'camera_matrix': {
                    'rows': 3,
                    'columns': 3,
                    'data': camera_matrix.tolist()
                },
                'distortion_coefficients': {
                    'rows': 1,
                    'columns': len(dist_coeffs[0]),
                    'data': dist_coeffs[0].tolist()
                },
                'chessboard': {
                    'pattern_size': {
                        'rows': self.chessboard_rows,
                        'columns': self.chessboard_cols
                    },
                    'square_size_meters': self.square_size
                },
                'image_size': {
                    'width': self.image_width,
                    'height': self.image_height
                },
                'rms_reprojection_error': ret
            }

            output_file = f"{self.output_path}/{self.file}"

            # ✅ YAML 파일로 저장
            with open(output_file, 'w') as file:
                yaml.dump(calibration_data, file)

            print(f"✅ 캘리브레이션 데이터 저장 완료: {output_file}")  # ✅ get_logger() 대신 print 사용

        except Exception as e:
            print(f"❌ 캘리브레이션 저장 실패: {e}")  # ✅ get_logger() 대신 print 사용

def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    try:
        rclpy.spin(node)  # ROS2 노드 실행
    except KeyboardInterrupt:
        print("🛑 프로그램 강제 종료 요청. 데이터 저장 중...")  # ✅ get_logger() 대신 print 사용
        node.save_calibration()
        print("✅ 캘리브레이션 프로세스 완료.")  # ✅ get_logger() 대신 print 사용
    finally:
        node.destroy_node()
        if rclpy.ok():  # ✅ shutdown이 실행되었는지 확인 후 호출
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"⚠️ Warning: {e}")  # ✅ 오류 발생 시 경고 출력 후 무시

if __name__ == '__main__':
    main()
