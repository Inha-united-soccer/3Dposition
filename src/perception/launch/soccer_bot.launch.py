# soccer_bot.launch.py

import os
# ▼▼▼ 추가된 import 구문 ▼▼▼
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
# ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'perception'

    # ▼▼▼ 추가된 RealSense 카메라 실행 부분 ▼▼▼
    # realsense2_camera 패키지의 설치 경로에서 launch 폴더를 찾음
    realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch')

    # 카메라 노드를 rs_launch.py를 포함하여 실행
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_launch_dir, 'rs_launch.py')
        ),
        # rs_launch.py에 전달할 파라미터 (필터 활성화)
        launch_arguments={
            'align_depth.enable': 'true',
            'filters': '"temporal,spatial"',
            'temporal_filter.filter_smooth_alpha': '0.4',
            'temporal_filter.filter_smooth_delta': '20',
            'spatial_filter.filter_magnitude': '2',
            'spatial_filter.filter_smooth_alpha': '0.5',
            'spatial_filter.filter_smooth_delta': '20',
            'enable_imu': 'false',
            'enable_accel': 'false',
            'enable_gyro': 'false',
        }.items()
    )
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

    # 공 탐지 노드
    ball_detector_node = Node(
        package=package_name,
        executable='ball_detector',
        name='ball_detector_node',
        parameters=[
            # setup.py에서 모델 경로를 지정했으므로 이 파라미터는 생략 가능
            # {'yolo_model': 'yolov8n.pt'},
            {'confidence_threshold': 0.3}
        ]
    )

    # 공 추적 및 제어 노드 (뇌 역할)
    ball_follower_node = Node(
        package=package_name,
        executable='ball_follower',
        name='ball_follower_node'
    )

    # 실행할 노드 리스트에 camera_node 추가
    return LaunchDescription([
        camera_node,
        ball_detector_node
    ])