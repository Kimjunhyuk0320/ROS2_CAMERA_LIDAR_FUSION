#!/usr/bin/env python3

import rclpy, os, cv2, datetime, time
import numpy as np
from cv_bridge import CvBridge
import open3d as o3d
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
import threading

from ros2_camera_lidar_fusion.read_yaml import extract_configuration

class SaveData(Node):
    def __init__(self):
        super().__init__('save_data_node')
        self.get_logger().info('✅ Save data node has been started!')

        # ✅ YAML 설정 파일 로드
        config_file = extract_configuration()
        if config_file is None:
            self.get_logger().error("❌ Failed to extract configuration file.")
            return

        # ✅ ROS2 토픽 설정
        self.max_file_saved = config_file['general']['max_file_saved']
        self.storage_path = config_file['general']['data_folder']
        self.image_topic = config_file['camera']['image_topic']
        self.lidar_topic = config_file['lidar']['lidar_topic']
        self.keyboard_listener_enabled = config_file['general']['keyboard_listener']
        self.slop = config_file['general']['slop']

        # ✅ 저장 폴더 생성
        if not os.path.exists(self.storage_path):
            os.makedirs(self.storage_path)
        self.get_logger().warn(f'⚡ Data will be saved at {self.storage_path}')

        # ✅ 최근 데이터 수신 시간 초기화
        self.last_image_time = time.time()
        self.last_lidar_time = time.time()

        # ✅ ROS2 토픽 구독 설정 (카메라 & LiDAR)
        self.image_sub = Subscriber(self, Image, self.image_topic)
        self.pointcloud_sub = Subscriber(self, PointCloud2, self.lidar_topic)

        # ✅ 동기화 필터 설정 (카메라 & LiDAR 데이터 동기화)
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.pointcloud_sub],
            queue_size=10,
            slop=self.slop
        )
        self.ts.registerCallback(self.synchronize_data)

        # ✅ 개별 토픽 수신 확인을 위한 콜백 추가
        self.image_sub.registerCallback(self.image_callback)
        self.pointcloud_sub.registerCallback(self.lidar_callback)

        # ✅ 키보드 리스너 설정 (Enter 키 입력 시 데이터 저장)
        self.save_data_flag = not self.keyboard_listener_enabled
        if self.keyboard_listener_enabled:
            self.start_keyboard_listener()

        # ✅ 데이터 수신 상태 모니터링 (5초마다 확인)
        self.create_timer(5.0, self.check_data_reception)

    def start_keyboard_listener(self):
        """📌 키보드 이벤트 리스너 시작 (Enter 키 입력 시 데이터 저장)"""
        def listen_for_enter():
            while True:
                key = input("Press 'Enter' to save data: ")
                if key.strip() == '':
                    self.save_data_flag = True
                    self.get_logger().info('🎯 Enter key pressed, ready to save data!')
        thread = threading.Thread(target=listen_for_enter, daemon=True)
        thread.start()

    def image_callback(self, msg):
        """📷 카메라 이미지 데이터 확인 (토픽 데이터 출력 및 타임스탬프 업데이트)"""
        self.last_image_time = time.time()
        # self.get_logger().info(f"📸 Received Image Message: {self.image_topic}")
        # self.get_logger().info(f"   - Encoding: {msg.encoding}")
        # self.get_logger().info(f"   - Width: {msg.width}, Height: {msg.height}")
        # self.get_logger().info(f"   - Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

        # # ✅ OpenCV를 이용한 이미지 표시
        # bridge = CvBridge()
        # try:
        #     cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #     cv2.imshow("Camera Image", cv_image)
        #     cv2.waitKey(1)  # OpenCV 창 유지
        # except Exception as e:
        #     self.get_logger().error(f"❌ Image conversion failed: {e}")

    def lidar_callback(self, msg):
        """🛑 LiDAR 데이터 확인 (수신 여부 로그 출력 및 타임스탬프 업데이트)"""
        self.last_lidar_time = time.time()
        # self.get_logger().info(f"🛠️ Received LiDAR Data: {self.lidar_topic}")
        # self.get_logger().info(f"   - Height: {msg.height}, Width: {msg.width}")
        # self.get_logger().info(f"   - Point Step: {msg.point_step} bytes, Row Step: {msg.row_step} bytes")
        # self.get_logger().info(f"   - Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

    def check_data_reception(self):
        """📌 5초 이상 데이터 미수신 시 경고 출력"""
        current_time = time.time()
        if current_time - self.last_image_time > 5.0:
            self.get_logger().error(f"🚨 No image data received from {self.image_topic} in the last 5 seconds!")
        if current_time - self.last_lidar_time > 5.0:
            self.get_logger().error(f"🚨 No LiDAR data received from {self.lidar_topic} in the last 5 seconds!")

    def synchronize_data(self, image_msg, pointcloud_msg):
        """📌 카메라 & LiDAR 데이터를 동기화하여 저장"""

        if self.save_data_flag:
            file_name = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')  # 현재 시간 기반 파일명 생성
            self.get_logger().info(f'📁 Saving synchronized data at {file_name}')

            total_files = len(os.listdir(self.storage_path))
            if total_files < self.max_file_saved:
                self.save_data(image_msg, pointcloud_msg, file_name)
                if self.keyboard_listener_enabled:
                    self.save_data_flag = False  # ✅ 키보드 입력 모드에서는 한 번 저장 후 대기

    def pointcloud2_to_open3d(self, pointcloud_msg):
        """📌 ROS2 PointCloud2 메시지를 Open3D 포맷으로 변환"""
        points = []
        for p in point_cloud2.read_points(pointcloud_msg, skip_nans=True):
            points.append([p[0], p[1], p[2]])

        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float32))
        return pointcloud

    def save_data(self, image_msg, pointcloud_msg, file_name):
        """📌 카메라 이미지 및 LiDAR 포인트클라우드 데이터 저장"""
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        pointcloud = self.pointcloud2_to_open3d(pointcloud_msg)

        # ✅ LiDAR 데이터 PCD(Point Cloud Data) 파일로 저장
        o3d.io.write_point_cloud(f'{self.storage_path}/{file_name}.pcd', pointcloud)

        # ✅ 카메라 이미지 PNG 파일로 저장
        cv2.imwrite(f'{self.storage_path}/{file_name}.png', image)

        self.get_logger().info(f'✅ Data saved: {self.storage_path}/{file_name}.png')


def main(args=None):
    """📌 ROS2 노드 실행"""
    rclpy.init(args=args)
    node = SaveData()
    try:
        rclpy.spin(node)  # ROS2 노드 실행 유지
    except KeyboardInterrupt:
        pass  # Ctrl+C로 종료 시 처리
    finally:
        node.destroy_node()  # 노드 삭제
        rclpy.shutdown()  # ROS2 종료


if __name__ == '__main__':
    main()  # ✅ 메인 실행
