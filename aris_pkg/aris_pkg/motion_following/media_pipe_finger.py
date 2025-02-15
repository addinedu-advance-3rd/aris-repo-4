import cv2
import numpy as np
import mediapipe as mp
import rclpy
from rclpy.node import Node
from my_first_pkg_msgs.msg import Delta
import math

class DetectMotion(Node):

    def __init__(self):    
        super().__init__('motion_publisher')
        self.publisher = self.create_publisher(Delta, '/motion_topic', 10)
        self.timer_period = 0.1  # 발행 주기 (0.1초)
        self.previous_wrist_x = None
        self.previous_wrist_y = None
        self.previous_hand_area = None  # 🔥 손 크기(면적) 저장
        self.current_distance_status = "Hand Distance Stable"  # 🔥 초기 상태는 Stable
        self.cap = cv2.VideoCapture(0)  # 카메라 초기화
        self.hand_area = 2
        # Mediapipe 초기화
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6
        )
        self.mp_drawing = mp.solutions.drawing_utils

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.detect_and_publish)

    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to grab frame.")
            return

        # RGB로 변환 및 Mediapipe 처리
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        if results.multi_hand_landmarks:
            # 🔥 가장 큰 손 찾기
            largest_hand = None
            largest_area = 0
            largest_bbox = (0, 0, 0, 0)
            largest_landmarks = None

            for hand_landmarks in results.multi_hand_landmarks:
                hand_area, bbox_coords = self.calculate_hand_area(hand_landmarks, frame.shape)
                if hand_area > largest_area:
                    largest_area = hand_area
                    largest_bbox = bbox_coords
                    largest_landmarks = hand_landmarks

            # 🔥 가장 큰 손만 처리
            if largest_landmarks:
                self.process_hand(frame, largest_landmarks, largest_area, largest_bbox)

        # 프레임 디스플레이
        cv2.imshow("Hand Detection with Gripper and Area", frame)

        # 'q' 키로 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()

    def process_hand(self, frame, hand_landmarks, hand_area, bbox_coords):
        """ 🔥 선택된 손에 대한 처리 및 메시지 발행 """
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        wrist_x = int(wrist.x * frame.shape[1])
        wrist_y = int(wrist.y * frame.shape[0])

        base_x = wrist_x
        base_y = frame.shape[0] - wrist_y

        delta_x, delta_y = 0, 0
        if self.previous_wrist_x is not None and self.previous_wrist_y is not None:
            delta_x = base_x - self.previous_wrist_x
            delta_y = base_y - self.previous_wrist_y

        # 이전 손목 위치 업데이트
        self.previous_wrist_x, self.previous_wrist_y = base_x, base_y

        # 바운딩 박스 그리기
        (min_x, min_y, max_x, max_y) = bbox_coords
        cv2.rectangle(frame, (min_x, min_y), (max_x, max_y), (0, 255, 255), 2)

        # 엄지와 검지 거리 측정
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

        thumb_tip_x = int(thumb_tip.x * frame.shape[1])
        thumb_tip_y = int(thumb_tip.y * frame.shape[0])

        index_tip_x = int(index_tip.x * frame.shape[1])
        index_tip_y = int(index_tip.y * frame.shape[0])

        distance_thumb_index = math.sqrt((index_tip_x - thumb_tip_x) ** 2 + (index_tip_y - thumb_tip_y) ** 2)

        # 엄지-검지 거리로 그리퍼 상태 결정
        gripper_threshold = 50  # 임계값 (픽셀 단위)
        gripper_state = distance_thumb_index < gripper_threshold  # True: 닫힘, False: 열림

        # "Stable" 상태일 때만 거리 변화 판단
        if self.current_distance_status == "Hand Distance Stable":
            self.hand_area = 2
            self.update_distance_status(hand_area)

        # Delta 메시지 생성 및 발행
        if abs(delta_x) < 100 and abs(delta_y) < 100:
            msg = Delta()
            msg.x = float(delta_x)
            msg.y = float(delta_y)
            msg.dis = float(self.hand_area)  # 손 면적 정보를 저장
            msg.gripper = bool(gripper_state)  # 엄지-검지 거리 기반 그리퍼 상태 저장

            self.publisher.publish(msg)

            # 디버깅 메시지
            gripper_status = True if gripper_state else False
            self.get_logger().info(f"Delta: x={msg.x}, y={msg.y}, Area={msg.dis:.2f}, Gripper={gripper_status}")

            # 화면에 디버깅 메시지 표시
            cv2.putText(frame, f'Delta X: {msg.x:.2f}, Delta Y: {msg.y:.2f}', 
                        (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, f'Hand Area: {self.hand_area:.2f}', 
                        (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, f'Gripper: {gripper_status}', 
                        (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
        else:
            self.get_logger().info("Wrist movement exceeds threshold, ignoring.")

    def calculate_hand_area(self, hand_landmarks, frame_shape):
        """ 🔥 손 랜드마크의 바운딩 박스를 기반으로 손 크기(면적)를 계산하되, 엄지와 검지의 모든 관절 제외 """
        
        excluded_landmarks = {
            self.mp_hands.HandLandmark.THUMB_CMC,
            self.mp_hands.HandLandmark.THUMB_MCP,
            self.mp_hands.HandLandmark.THUMB_IP,
            self.mp_hands.HandLandmark.THUMB_TIP,
            self.mp_hands.HandLandmark.INDEX_FINGER_MCP,
            self.mp_hands.HandLandmark.INDEX_FINGER_PIP,
            self.mp_hands.HandLandmark.INDEX_FINGER_DIP,
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP
        }

        x_coords = [int(landmark.x * frame_shape[1]) for i, landmark in enumerate(hand_landmarks.landmark) if i not in excluded_landmarks]
        y_coords = [int(landmark.y * frame_shape[0]) for i, landmark in enumerate(hand_landmarks.landmark) if i not in excluded_landmarks]

        if not x_coords or not y_coords:
            return 0, (0, 0, 0, 0)

        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)

        width = max_x - min_x
        height = max_y - min_y

        area = width * height
        bbox_coords = (min_x, min_y, max_x, max_y)
        return area, bbox_coords

    def update_distance_status(self, hand_area):
        """ 🔥 "Stable" 상태일 때만 손의 거리 변화 감지 """
        if self.previous_hand_area is None:
            self.previous_hand_area = hand_area
            return

        if hand_area > self.previous_hand_area * 1.1:
            self.current_distance_status = "Hand Moving Closer"
            self.schedule_reset_status()
            self.hand_area = 0
        elif hand_area < self.previous_hand_area * 0.9:
            self.current_distance_status = "Hand Moving Away"
            self.schedule_reset_status()
            self.hand_area = 1
        else:
            self.hand_area = 2
        self.previous_hand_area = hand_area

    def schedule_reset_status(self):
        """ 🔥 일정 시간 후 상태를 Stable로 초기화 """
        self.get_logger().info("Status will reset to Stable in 1 second.")
        self.create_timer(1.0, self.reset_distance_status)

    def reset_distance_status(self):
        """ 🔥 상태를 Stable로 초기화 """
        if self.current_distance_status in ["Hand Moving Closer", "Hand Moving Away"]:
            self.get_logger().info("Resetting distance status to Stable.")
            self.current_distance_status = "Hand Distance Stable"

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motion_publisher = DetectMotion()
    try:
        rclpy.spin(motion_publisher)
    except KeyboardInterrupt:
        motion_publisher.get_logger().info("Node interrupted by user.")
    finally:
        motion_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
