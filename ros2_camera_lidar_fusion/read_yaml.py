import yaml  # ✅ YAML 파일을 다루기 위한 라이브러리 (설정 파일 로드)
import os  # ✅ 경로 조작을 위한 OS 라이브러리
from ament_index_python.packages import get_package_share_directory  # ✅ ROS2 패키지 디렉터리를 가져오는 함수

def extract_configuration():
    """📌 ROS2 패키지 내부의 YAML 설정 파일을 로드하여 반환하는 함수"""

    # ✅ 현재 패키지(`ros2_camera_lidar_fusion`)의 공유 디렉터리 경로 가져오기
    package_share_directory = get_package_share_directory('ros2_camera_lidar_fusion')

    # ✅ 설정 파일의 전체 경로 생성
    config_file = os.path.join(
        package_share_directory,  # 패키지의 공유 디렉터리 경로
        'config',  # 설정 파일이 들어있는 폴더
        'general_configuration.yaml'  # 불러올 설정 파일 이름
    )

    # ✅ YAML 설정 파일을 읽어오기
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)  # YAML 데이터를 파이썬 딕셔너리로 변환

    return config  # ✅ 설정 값을 반환
