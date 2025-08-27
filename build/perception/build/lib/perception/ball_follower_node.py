# ball_follower_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
import message_filters
from cv_bridge import CvBridge

# 표준 메시지 타입인 vision_msgs를 사용합니다.
from vision_msgs.msg import Detection2DArray

class BallFollowerNode(Node):
    def __init__(self):
        super().__init__('ball_follower_node')
        
        # ROS <-> OpenCV 이미지 변환을 위한 CvBridge
        self.bridge = CvBridge()
        
        # 터틀봇 제어 명령을 발행할 Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 카메라 내부 파라미터 (fx, fy, cx, cy)를 저장할 변수
        self.camera_intrinsics = None
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # --- 메시지 동기화 설정 ---
        # Depth 이미지와 YOLO 탐지 결과를 동시에 받기 위함
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')
        self.yolo_sub = message_filters.Subscriber(self, Detection2DArray, '/yolo/detections')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.yolo_sub],
            queue_size=10,
            slop=0.1,  # 0.1초 이내의 타임스탬프 차이를 가진 메시지들을 동기화
        )
        self.ts.registerCallback(self.synced_callback)

        self.get_logger().info("공 추적 노드가 시작되었습니다.")

    def camera_info_callback(self, msg):
        # 카메라 파라미터는 한 번만 받으면 되므로, 수신 후 구독을 해제하여 효율성 증대
        if self.camera_intrinsics is None:
            self.camera_intrinsics = msg
            self.get_logger().info("카메라 파라미터를 성공적으로 수신했습니다.")
            self.destroy_subscription(self.camera_info_sub) 

    def synced_callback(self, depth_msg, yolo_msg):
        # 카메라 파라미터를 아직 받지 못했다면 로직을 실행하지 않음
        if self.camera_intrinsics is None:
            self.get_logger().warn("카메라 파라미터를 기다리는 중...")
            return
            
        # Depth 이미지를 OpenCV 형식으로 변환 (16비트 단일 채널)
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

        # YOLO가 탐지한 객체들 중 'sports ball' 찾기
        for detection in yolo_msg.detections:
            # vision_msgs의 class_id는 문자열, COCO 모델에서 공은 '32'
            if detection.results[0].hypothesis.class_id == '32':
                
                # 1. 공의 2D 중심 좌표 계산
                center_x = int(detection.bbox.center.position.x)
                center_y = int(detection.bbox.center.position.y)

                # 2. 중심 좌표의 깊이(거리) 값 추출 (단위: 미터)
                try:
                    distance_mm = depth_image[center_y, center_x]
                    # 깊이 값이 0이면 유효하지 않은 값이므로 무시
                    if distance_mm == 0:
                        self.get_logger().warn("공의 깊이 값이 0입니다. 다음 프레임을 기다립니다.")
                        continue
                    
                    distance_m = distance_mm / 1000.0  # RealSense는 mm 단위로 값을 주므로 m로 변환

                    # 3. 공의 3D 좌표 계산 (카메라 기준)
                    fx = self.camera_intrinsics.k[0]
                    cx = self.camera_intrinsics.k[2]
                    
                    # 좌우 위치(X)와 전방 거리(Z) 계산
                    ball_x = (center_x - cx) * distance_m / fx
                    ball_z = distance_m

                    # 4. 계산된 3D 위치를 바탕으로 터틀봇 제어
                    self.control_turtlebot(ball_x, ball_z)
                    return  # 공을 하나 찾았으면 콜백 함수 종료
                except IndexError:
                    self.get_logger().warn("이미지 좌표가 범위를 벗어났습니다.")
                    continue
        
        # for 루프가 끝날 때까지 공을 찾지 못했다면 로봇 정지
        self.stop_turtlebot()

    def control_turtlebot(self, ball_x, ball_z):
        """ P-제어(비례 제어)를 이용한 간단한 로봇 제어 로직 """
        twist_msg = Twist()
        
        # --- 튜닝 파라미터 ---
        desired_distance = 0.6  # 로봇과 공의 목표 거리
        angular_p_gain = 1.0    # 회전 속도 게인 (값이 클수록 빠르게 회전)
        linear_p_gain = 0.5    # 전진 속도 게인 (값이 클수록 빠르게 전진)
        
        # 회전 제어: 공이 중심에서 벗어난 정도(ball_x)에 비례하여 회전 속도 결정
        twist_msg.angular.z = -angular_p_gain * ball_x

        # 전진/후진 제어: 목표 거리와의 차이에 비례하여 전진/후진 속도 결정
        twist_msg.linear.x = linear_p_gain * (ball_z - desired_distance)

        self.get_logger().info(f"공 위치: [좌우:{ball_x:.2f}m, 전방:{ball_z:.2f}m] | 제어: [선속도:{twist_msg.linear.x:.2f}, 각속도:{twist_msg.angular.z:.2f}]")
        self.cmd_vel_pub.publish(twist_msg)

    def stop_turtlebot(self):
        """ 공을 찾지 못했을 때 로봇을 정지시키는 함수 """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트로 노드를 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()