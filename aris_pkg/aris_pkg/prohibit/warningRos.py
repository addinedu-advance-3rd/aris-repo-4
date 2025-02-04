import cv2
import numpy as np
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool  # Bool 메시지 타입

class WarningPublisher(Node):
    def __init__(self):
        super().__init__('warning_publisher')  # 노드 이름
        self.publisher_ = self.create_publisher(Bool, 'robot_warning', 10)  # 'robot_warning' 토픽, Bool 타입
        #self.timer = self.create_timer(0.1, self.timer_callback)  # 타이머로 메시지 주기적으로 발행

    def publish_warning(self, warning_condition):
        msg = Bool()
        msg.data = warning_condition  # True/False 값으로 메시지 발행
        self.publisher_.publish(msg)
        print(f"Published warning: {warning_condition}")  # 메시지 출력 (터미널에 바로 출력)
        self.get_logger().info(f'Published warning: {warning_condition}')  # ROS2 로깅 시스템에 출력
            
    # def timer_callback(self):
    #     # 여기에 웹캠 처리 코드 넣기 (아래 코드에서 경고 메시지 확인 후 publish)
    #     warning_condition = False  # 기본값은 경고 없음 (True로 설정하면 경고 없음)
    #     self.publish_warning(warning_condition)

def main(args=None):
    rclpy.init(args=args)  # ROS2 초기화
    warning_publisher = WarningPublisher()

    # 카메라 캡처 시작
    cap = cv2.VideoCapture(0)  # 기본 카메라
    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        exit()

    print("웹캠 실행 중... 종료하려면 'q'를 누르세요.")

    # YOLO 모델 불러오기
    model_path = "/home/addinedu/dev_ws/src/aris_pkg/aris_pkg/prohibit/best.pt"
    session = YOLO(model_path)

    while rclpy.ok():
        ret, frame = cap.read()  # 카메라로부터 프레임 읽기
        if not ret:
            print("프레임을 읽을 수 없습니다. 종료합니다.")
            break

        # YOLO 모델로 추론
        results = session(frame)
        result = results[0]
        masks = result.masks  # 마스크 객체 추출

        hand_mask = None
        arm_mask = None
        warning_condition = True

        if masks is not None:
            mask_data = masks.data.cpu().numpy()
            for idx, mask in enumerate(mask_data):
                class_name = result.names[int(result.boxes[idx].cls)]
                mask_img = (mask * 255).astype(np.uint8)
                mask_img = cv2.merge([mask_img] * 3)

                if class_name == 'hand':
                    frame[mask == 1] = (0, 255, 0)  # 손은 초록색
                    hand_mask = mask
                elif class_name == 'arm':
                    frame[mask == 1] = (0, 0, 255)  # 로봇팔은 빨간색
                    arm_mask = mask

        if hand_mask is not None and arm_mask is not None:
            intersection = np.logical_and(hand_mask, arm_mask)
            intersection_area = np.sum(intersection)
            arm_area = np.sum(arm_mask)
            
            # 경고 메시지 발송 조건
            if intersection_area > 0.5 * arm_area:
                warning_condition = False  # 경고가 발생했으므로 False 발행
            else:
                warning_condition = True  # 경고가 없으므로 True 발행
        
        warning_publisher.publish_warning(warning_condition)

        cv2.imshow("Segmented Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 종료 시
    cap.release()
    cv2.destroyAllWindows()
    warning_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
