import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesis, ObjectHypothesisWithPose

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')
        
        # 파라미터
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.3)
        
        # 파라미터 값 가져오기
        model_name = self.get_parameter('yolo_model').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        # YOLO 모델 로드
        self.model = YOLO(model_name)

        # ROS <-> OpenCV 이미지 변환을 위한 CvBridge
        self.bridge = CvBridge()

        # 이미지 토픽을 구독
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw', # 입력 이미지 토픽
            self.image_callback,
            10
        )
        
        # 탐지 결과(바운딩 박스 정보)를 발행할 Publisher
        self.detections_publisher = self.create_publisher(
            Detection2DArray,
            '/yolo/detections', # 이 토픽을 ball_follower 노드가 구독하게 됨
            10
        )
        
        # 시각화된 이미지를 발행할 Publisher (디버깅용)
        self.result_image_publisher = self.create_publisher(
            Image,
            '/yolo/result_image',
            10
        )

        self.get_logger().info("공 탐지 노드가 준비되었습니다.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge fail: {e}")
            return

        # YOLO 모델 실행
        results = self.model(cv_image, conf=self.conf_threshold, verbose=False)

        # 탐지 결과를 담을 메시지 생성
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header # 원본 이미지의 헤더 정보 사용

        # 결과에서 바운딩 박스, 신뢰도, 클래스 ID 추출
        for result in results:
            for box in result.boxes:
                # 클래스 이름 확인 (COCO dataset에서 'sports ball'은 class id 32)
                class_id = int(box.cls)
                class_name = self.model.names[class_id]

                if class_name == 'sports ball':
                    # Detection 메시지 생성
                    detection = Detection2D()
                    detection.header = msg.header 
                    
                    # 바운딩 박스 중심 좌표와 크기
                    (x, y, w, h) = box.xywh[0]
                    detection.bbox.center.position.x = float(x)
                    detection.bbox.center.position.y = float(y)
                    detection.bbox.size_x = float(w)
                    detection.bbox.size_y = float(h)

                    
                    # 클래스 정보와 신뢰도
                    # 1. 핵심 가설 정보(클래스, 신뢰도)를 ObjectHypothesis에 담습니다.
                    obj_hypothesis = ObjectHypothesis()
                    obj_hypothesis.class_id = str(class_id)
                    obj_hypothesis.score = float(box.conf)

                    # 2. 위 정보를 ObjectHypothesisWithPose 라는 상자에 다시 담습니다.
                    #    (2D 탐지에서는 pose 정보는 비워둬도 괜찮습니다.)
                    hypothesis_with_pose = ObjectHypothesisWithPose()
                    hypothesis_with_pose.hypothesis = obj_hypothesis

                    # 3. 올바른 타입의 객체를 results 리스트에 추가합니다.
                    detection.results.append(hypothesis_with_pose)
                    
                    detections_msg.detections.append(detection)

        # 탐지된 공이 있을 경우에만 메시지 발행
        if len(detections_msg.detections) > 0:
            self.detections_publisher.publish(detections_msg)

        # 시각화된 이미지 생성 및 발행 (디버깅에 매우 유용)
        plotted_image = results[0].plot()
        result_image_msg = self.bridge.cv2_to_imgmsg(plotted_image, 'bgr8')
        result_image_msg.header = msg.header
        self.result_image_publisher.publish(result_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()