# LiDAR-카메라 센서 융합을 이용한 지형 인식 및 분석 시스템

## 1. 프로젝트 개요
최근 자율주행, 로봇 비전, 정밀 측위 등 다양한 응용 분야에서 **센서 융합 기술**이 중요한 연구 주제로 떠오르고 있습니다. 본 연구에서는 **LiDAR(Light Detection and Ranging) 센서**와 **카메라**를 활용하여 건설 현장에서 지형을 인식하고, 해당 지형이 굴착이 가능한지를 판단하는 시스템을 구축하는 것을 목표로 합니다.

본 프로젝트는 **Ubuntu 22.04 환경에서 ROS2 Humble**을 기반으로 **Avia Livox LiDAR**와 **ZED 2.0i 스테레오 카메라**를 활용하여 **LiDAR-카메라 융합 시스템**을 구축하였습니다.

### 🔹 연구 목표
- LiDAR와 카메라 데이터를 융합하여 **정확한 환경 인식**을 수행
- **포인트 클라우드 데이터**를 활용하여 **객체 탐지 및 거리 계산**
- **Extrinsic Calibration**을 통해 LiDAR-카메라 간 **정확한 좌표 변환 매트릭스** 도출
- LiDAR 데이터를 카메라 이미지에 투영하여 **직관적인 시각화** 구현
- 건설 현장에서 **굴착 가능 여부 분석 및 지형 특징** 파악

---

## 2. 사용 기술 및 알고리즘

### 📌 **환경 및 프레임워크**
- 운영체제: **Ubuntu 22.04 LTS**
- ROS2 배포판: **Humble**
- 프로그래밍 언어: **Python**
- 주요 라이브러리: **OpenCV, NumPy, Open3D, YAML**
- 사용 센서:
  - **LiDAR**: Avia Livox
  - **카메라**: ZED 2.0i 스테레오 카메라

### 📌 **주요 알고리즘**
| 알고리즘 | 설명 |
|-----------|----------------------------------------------------------------|
| **SolvePnP** | LiDAR-카메라 외부 캘리브레이션 수행 (3D-2D 좌표 변환) |
| **cv2.projectPoints()** | 3D LiDAR 포인트를 2D 카메라 이미지 좌표로 투영 |
| **K-Nearest Neighbor (K-NN)** | 추가적인 대응점 자동 추출 기법 적용 (추후 연구 방향) |
| **딥러닝 기반 객체 탐지** | YOLO, Faster R-CNN 등과 결합하여 객체 탐지 강화 (추후 연구) |
| **Machine Learning 기반 분석** | 포인트 클라우드를 활용한 굴착 가능성 예측 모델 개발 |

---

## 3. 프로젝트 주요 기능

### **카메라 보정 (Intrinsic Calibration)**
- 체커보드 패턴을 이용하여 카메라 매트릭스 및 왜곡 계수 계산
- OpenCV의 **cv2.calibrateCamera()** 활용하여 보정 수행
- 결과를 **YAML 파일**로 저장

### **LiDAR-카메라 외부 캘리브레이션 (Extrinsic Calibration)**
- SolvePnP 알고리즘을 이용하여 **4x4 변환 행렬(Transformation Matrix)** 도출
- LiDAR 데이터를 카메라 좌표계로 변환하여 정합 수행
- 변환 행렬을 **YAML 파일**로 저장하여 다른 모듈에서 활용 가능

### **포인트 클라우드 데이터 처리**
- ROS2의 **sensor_msgs/PointCloud2** 메시지를 NumPy 배열로 변환하여 활용
- Open3D 라이브러리를 사용하여 포인트 클라우드 시각화

### **LiDAR-카메라 데이터 융합**
- **cv2.projectPoints()**를 사용하여 LiDAR 3D 포인트를 카메라 2D 이미지로 투영
- 거리 기반 색상 매핑 적용 (가까운 물체는 **파란색**, 먼 물체는 **빨간색**)
- 최종 결과를 **이미지 및 영상 출력**

### **데이터 저장 및 분석**
- **PCD(Point Cloud Data) 파일** 및 **이미지 데이터 저장**
- LiDAR 데이터를 Open3D를 통해 처리 및 필터링

---

## 4. 설치 및 실행 방법

### 1️⃣ **필수 패키지 설치**
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-opencv ros-humble-desktop
pip install numpy pyyaml open3d
```

### 2️⃣ **ROS2 환경 설정**
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/사용자이름/ros2_camera_lidar_fusion.git
cd ~/ros2_ws && colcon build
source install/setup.bash
```

### 3️⃣ **Livox Avia LiDAR 실행**
```bash
cd ~/ros2_ws/src/ros2_camera_lidar_fusion
source install/setup.bash
ros2 launch livox_ros2_driver livox_lidar_launch.py
```

### 4️⃣ **ZED 2.0i 카메라 실행**
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

### 5️⃣ **LiDAR-카메라 캘리브레이션 수행**
```bash
ros2 run ros2_camera_lidar_fusion get_extrinsic_camera_calibration.py
```

### 6️⃣ **센서 융합 실행 (LiDAR 데이터 카메라에 투영)**
```bash
ros2 run ros2_camera_lidar_fusion lidar_camera_projection.py
```

---

## 5. 프로젝트 결과 및 성능 평가
### 📊 **실험 결과**
| LiDAR 측정 거리 (m) | 실제 측정 거리 (m) | 오차율 (%) |
|---------------------|------------------|-----------|
| 4.43 | 4.58 | 3.28% |
| 4.70 | 4.81 | 2.28% |
| 5.10 | 5.35 | 4.67% |
| 5.20 | 5.39 | 3.52% |

- 전체 평균 오차율: **3.44%**
- LiDAR의 데이터 정합을 통해 **높은 정확도로 거리 측정 가능**

### 📌 **결론**
- 본 연구를 통해 **LiDAR와 카메라의 보완적인 특성을 활용하여 정밀한 환경 인식**을 수행할 수 있음을 확인
- **Extrinsic Calibration을 통해 3D 포인트 데이터를 2D 이미지에 효과적으로 투영 가능**
- 건설 현장에서 **굴착 가능 여부를 분석하고, 작업 효율성을 증가**시키는 데 기여할 수 있음

### 📌 **향후 연구 방향**
- **K-NN 기반 추가 대응점 자동 추출 기법 적용**
- **YOLO 기반 객체 탐지 모델과 LiDAR 데이터 융합**
- **머신러닝 기반의 지형 분류 및 굴착 난이도 예측 시스템 구축**

---

## 6. 라이선스
본 프로젝트는 MIT 라이선스를 따릅니다. 자유롭게 활용하시되, 출처를 밝혀주세요.


