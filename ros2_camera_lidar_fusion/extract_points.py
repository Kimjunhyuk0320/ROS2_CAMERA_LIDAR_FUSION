#!/usr/bin/env python3  # ✅ Python3 인터프리터에서 실행하도록 설정

# ✅ 필수 라이브러리 임포트
import os  # 파일 및 디렉토리 조작 (파일 존재 여부 확인, 디렉토리 생성 등)
import cv2  # OpenCV (이미지 처리 및 시각화)
import open3d as o3d  # Open3D (포인트 클라우드 시각화 및 선택)
import numpy as np  # 수치 연산을 위한 NumPy
from rclpy.node import Node  # ROS2 노드 클래스
import rclpy  # ROS2 기본 라이브러리

# ✅ 사용자 정의 모듈 (설정 파일을 읽어오기 위한 함수)
from ros2_camera_lidar_fusion.read_yaml import extract_configuration  

class ImageCloudCorrespondenceNode(Node):
    """📌 카메라 이미지와 LiDAR 포인트 클라우드에서 2D-3D 대응점을 수동 선택하는 ROS2 노드"""

    def __init__(self):
        super().__init__('image_cloud_correspondence_node')  # ✅ ROS2 노드 생성

        # ✅ 설정 파일 로드
        config_file = extract_configuration()
        if config_file is None:
            self.get_logger().error("Failed to extract configuration file.")  # ❌ 설정 파일이 없으면 오류 출력
            return  # 노드 종료

        # ✅ YAML에서 설정값 가져오기
        self.data_dir = config_file['general']['data_folder']  # 데이터 저장 폴더
        self.file = config_file['general']['correspondence_file']  # 대응점 파일 이름

        # ✅ 데이터 폴더 확인 및 생성
        if not os.path.exists(self.data_dir):
            self.get_logger().warn(f"Data directory '{self.data_dir}' does not exist. Creating it now.")
            os.makedirs(self.data_dir)

        self.get_logger().info(f"Searching for .png and .pcd file pairs in '{self.data_dir}'")
        
        # ✅ 이미지 - 포인트 클라우드 파일 쌍을 찾고 처리
        self.process_file_pairs()

    def get_file_pairs(self, directory):
        """📌 `.png`(이미지)와 `.pcd`(포인트 클라우드) 파일 쌍을 찾는 함수"""
        files = os.listdir(directory)  # ✅ 폴더 내 파일 목록 가져오기
        pairs_dict = {}

        for f in files:
            full_path = os.path.join(directory, f)
            if not os.path.isfile(full_path):  # ✅ 파일이 아닐 경우 건너뛴다.
                continue
            name, ext = os.path.splitext(f)  # ✅ 파일 이름과 확장자 분리

            # ✅ PNG 또는 PCD 파일만 처리
            if ext.lower() in [".png", ".jpg", ".jpeg", ".pcd"]:
                if name not in pairs_dict:
                    pairs_dict[name] = {}
                if ext.lower() == ".png":
                    pairs_dict[name]['png'] = full_path
                elif ext.lower() == ".pcd":
                    pairs_dict[name]['pcd'] = full_path

        # ✅ 이미지와 포인트 클라우드가 모두 있는 경우만 선택
        file_pairs = [(prefix, d['png'], d['pcd']) for prefix, d in pairs_dict.items() if 'png' in d and 'pcd' in d]
        file_pairs.sort()
        return file_pairs  # ✅ 이미지-포인트 클라우드 파일 쌍 리스트 반환

    def pick_image_points(self, image_path):
        """📌 OpenCV를 이용하여 2D 이미지 좌표 선택"""
        img = cv2.imread(image_path)  # ✅ 이미지 로드
        if img is None:
            self.get_logger().error(f"Error loading image: {image_path}")
            return []

        points_2d = []  # ✅ 선택한 2D 좌표 저장 리스트
        window_name = "Select points on the image (press 'q' or ESC to finish)"

        def mouse_callback(event, x, y, flags, param):
            """📌 마우스 클릭 이벤트: 좌표 저장"""
            if event == cv2.EVENT_LBUTTONDOWN:
                points_2d.append((x, y))
                self.get_logger().info(f"Image: click at ({x}, {y})")

        # ✅ OpenCV 윈도우 설정
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(window_name, mouse_callback)

        while True:
            display_img = img.copy()
            for pt in points_2d:
                cv2.circle(display_img, pt, 5, (0, 0, 255), -1)  # ✅ 선택한 점을 빨간색 원으로 표시

            cv2.imshow(window_name, display_img)
            key = cv2.waitKey(10)
            if key == 27 or key == ord('q'):  # ✅ 'q' 또는 ESC 키를 누르면 종료
                break

        cv2.destroyWindow(window_name)
        return points_2d  # ✅ 선택된 2D 좌표 리스트 반환

    def pick_cloud_points(self, pcd_path):
        """📌 Open3D를 이용하여 3D 포인트 클라우드 좌표 선택"""
        pcd = o3d.io.read_point_cloud(pcd_path)
        if pcd.is_empty():
            self.get_logger().error(f"Empty or invalid point cloud: {pcd_path}")
            return []

        self.get_logger().info("\n[Open3D Instructions]")
        self.get_logger().info("  - Shift + left click to select a point")
        self.get_logger().info("  - Press 'q' or ESC to close the window when finished\n")

        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="Select points on the cloud", width=1280, height=720)
        vis.add_geometry(pcd)

        vis.run()
        vis.destroy_window()
        picked_indices = vis.get_picked_points()  # ✅ 선택된 포인트 인덱스 가져오기

        np_points = np.asarray(pcd.points)
        return [(float(np_points[idx][0]), float(np_points[idx][1]), float(np_points[idx][2])) for idx in picked_indices]  # ✅ 선택된 3D 좌표 반환

    def process_file_pairs(self):
        """📌 이미지 - 포인트 클라우드 파일 쌍을 찾아 2D-3D 매칭 후 `.txt` 파일에 계속 추가 저장"""
        file_pairs = self.get_file_pairs(self.data_dir)
        if not file_pairs:
            self.get_logger().error(f"No .png / .pcd pairs found in '{self.data_dir}'")
            return

        out_txt = os.path.join(self.data_dir, self.file)  # ✅ 대응점 저장할 파일 경로 설정

        # ✅ 첫 실행 여부 확인 (파일이 없으면 헤더 추가)
        first_write = not os.path.exists(out_txt)

        with open(out_txt, 'a') as f:  # ✅ 'a' 모드로 파일 열기 (기존 데이터 유지)
            if first_write:
                f.write("# u, v, x, y, z\n")  # ✅ 첫 실행 시 헤더 추가

            for prefix, png_path, pcd_path in file_pairs:
                self.get_logger().info(f"Processing pair: {prefix} -> {png_path}, {pcd_path}")

                image_points = self.pick_image_points(png_path)  # ✅ 2D 이미지에서 대응점 선택
                cloud_points = self.pick_cloud_points(pcd_path)  # ✅ 3D 포인트 클라우드에서 대응점 선택

                for i in range(min(len(image_points), len(cloud_points))):
                    f.write(f"{image_points[i][0]},{image_points[i][1]},{cloud_points[i][0]},{cloud_points[i][1]},{cloud_points[i][2]}\n")  # ✅ 대응점 저장

                self.get_logger().info(f"Saved {len(image_points)} correspondences in: {out_txt}")  # ✅ 로그 출력

        self.get_logger().info("\nProcessing complete! Correspondences saved.")  # ✅ 로그 출력


def main(args=None):
    """📌 ROS2 노드 실행"""
    rclpy.init(args=args)
    node = ImageCloudCorrespondenceNode()
    try:
        rclpy.spin(node)  # ✅ ROS2 노드 실행
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()  # ✅ 메인 실행
