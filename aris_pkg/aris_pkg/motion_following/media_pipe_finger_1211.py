import cv2
import time
import numpy as np
import mediapipe as mp
import rclpy
from rclpy.node import Node
from my_first_pkg_msgs.msg import Delta
from my_first_pkg_msgs.action import MotionFlag
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import math

class DetectMotion(Node):

    def __init__(self):    
        super().__init__('motion_publisher')

        # Delta 메시지 퍼블리셔
        self.publisher = self.create_publisher(Delta, '/motion_topic', 10)
        
        # 액션 서버 설정 (motion_following 네임스페이스 사용)
        self.action_server = ActionServer(self, MotionFlag, '/motion_following', self.execute_callback)
        
        self.timer_period = 0.1  # 발행 주기 (0.1초)
        self.previous_wrist_x = None
        self.previous_wrist_y = None
        
        self.timer_start = 0
        self.timer_end = 0
        self.receive_msg = True  # 액션 서버가 시작되었는지 여부
        
        self.cap = cv2.VideoCapture(0)  # 카메라 초기화

        # Mediapipe 초기화
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6
        )
        self.mp_drawing = mp.solutions.drawing_utils

    def execute_callback(self, goal_handle):
        """
        액션 서버가 호출되었을 때 실행되는 콜백 함수
        """
        self.get_logger().info("📡 Motion following started.")
        feedback_msg = MotionFlag.Feedback()

        # 타이머 시작
        self.timer_start = time.time()
        
        while True:
            # 손 인식 및 메시지 발행
            self.detect_and_publish()

            # 피드백 업데이트
            elapsed_time = time.time() - self.timer_start
            feedback_msg.time = elapsed_time
            goal_handle.publish_feedback(feedback_msg)

            if elapsed_time > 30:  # 30초 후 액션 종료
                break

        # 액션 성공 처리
        goal_handle.succeed()
        result = MotionFlag.Result()
        result.end = True  # 액션 종료 플래그 설정
        
        self.get_logger().info("🏁 Motion following completed.")
        self.receive_msg = True  # 다시 메시지 수신 대기 상태로 변경
        
        return result

    def detect_and_publish(self):
        try:
            """
            Mediapipe를 통해 손을 감지하고 Delta 메시지를 발행
            """
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warning("Failed to grab frame.")
                return

            # Mediapipe 손 인식 처리
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(frame_rgb)
            
            delta_x, delta_y, gripper_state, delta_z = None, None, None, None

            if results.multi_hand_landmarks and results.multi_handedness:
                largest_left_hand = None
                largest_right_hand = None
                largest_left_area = 0
                largest_right_area = 0
                left_bbox = None
                right_bbox = None

                # 모든 감지된 손에 대해 처리
                for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                    label = handedness.classification[0].label  # 'Left' 또는 'Right'
                    hand_area, bbox_coords = self.calculate_hand_area(hand_landmarks, frame.shape)

                    if label == 'Left' and hand_area > largest_left_area:
                        largest_left_area = hand_area
                        largest_left_hand = hand_landmarks
                        left_bbox = bbox_coords  # 왼손 바운딩 박스 저장

                    elif label == 'Right' and hand_area > largest_right_area:
                        largest_right_area = hand_area
                        largest_right_hand = hand_landmarks
                        right_bbox = bbox_coords  # 오른손 바운딩 박스 저장

                # 왼손과 오른손 바운딩 박스가 겹치는지 확인
                hands_overlap = self.check_overlap(left_bbox, right_bbox) if (left_bbox and right_bbox) else False

                if hands_overlap:
                    self.get_logger().info("Hands are overlapping, skipping gesture detection.")
                    return  # 손이 겹치는 경우 바로 종료하여 판단하지 않음

                # 왼손 감지 시 위치 변화량 및 그리퍼 상태 감지
                if largest_left_hand:
                    delta_x, delta_y, gripper_state = self.process_hand(frame, largest_left_hand)
                else:
                    self.get_logger().info("No left hand detected.")

                # 오른손 감지 시 제스처 기반 Z 방향 이동 감지
                if largest_right_hand:
                    delta_z = self.process_delta_z(frame, largest_right_hand)
                else:
                    self.get_logger().info("No right hand detected.")

                self.timer_end = time.time()

                # 메시지 발행 조건 확인
                msg = Delta()
                if delta_x is not None:
                    msg.x = float(delta_x)
                if delta_y is not None:
                    msg.y = float(delta_y)
                if gripper_state is not None:
                    msg.gripper = gripper_state
                if delta_z is not None:
                    msg.z = int(delta_z)

                # 메시지 발행 조건
                if delta_x is not None and delta_y is not None and gripper_state is not None:
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Published Delta: x={msg.x}, y={msg.y}, z={msg.z if delta_z is not None else 'N/A'}, Gripper={'Closed' if gripper_state else 'Open'}")

                    # 화면에 결과 표시
                    cv2.putText(frame, f'Delta X: {msg.x:.2f}', (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f'Delta Y: {msg.y:.2f}', (20, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f'Gripper: {"Closed" if gripper_state else "Open"}', (20, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    if delta_z is not None:
                        cv2.putText(frame, f'Delta Z: {msg.z}', (20, 350), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                else:
                    self.get_logger().info("Incomplete data, message not published.")

            # 프레임 디스플레이
            cv2.imshow('Hand Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.destroy_node()

        except Exception as e:
            self.get_logger().error(f"Unexpected error in detect_and_publish: {e}")



    def calculate_hand_area(self, hand_landmarks, frame_shape):
        """
        손의 바운딩 박스를 계산하여 면적을 반환
        """
        image_height, image_width, _ = frame_shape
        x_coords = [landmark.x * image_width for landmark in hand_landmarks.landmark]
        y_coords = [landmark.y * image_height for landmark in hand_landmarks.landmark]

        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)

        area = (x_max - x_min) * (y_max - y_min)
        bbox_coords = (int(x_min), int(y_min), int(x_max), int(y_max))

        return area, bbox_coords

    def process_hand(self, frame, hand_landmarks):
        """
        왼손의 위치 변화량 및 그리퍼 상태 감지
        """
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        wrist_x = int(wrist.x * frame.shape[1])
        wrist_y = int(wrist.y * frame.shape[0])

        base_x = wrist_x
        base_y = frame.shape[0] - wrist_y

        delta_x, delta_y = 0, 0
        if self.previous_wrist_x is not None and self.previous_wrist_y is not None:
            delta_x = base_x - self.previous_wrist_x
            delta_y = base_y - self.previous_wrist_y

        self.previous_wrist_x, self.previous_wrist_y = base_x, base_y

        # 그리퍼 상태 감지 (엄지와 검지의 거리)
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

        thumb_tip_x = int(thumb_tip.x * frame.shape[1])
        thumb_tip_y = int(thumb_tip.y * frame.shape[0])

        index_tip_x = int(index_tip.x * frame.shape[1])
        index_tip_y = int(index_tip.y * frame.shape[0])

        distance_thumb_index = math.hypot(index_tip_x - thumb_tip_x, index_tip_y - thumb_tip_y)
        gripper_state = distance_thumb_index < 50  # 50 픽셀 이하면 그리퍼 닫힘

        return delta_x, delta_y, gripper_state

    def process_delta_z(self, frame, hand_landmarks):
        """
        오른손의 제스처에 따라 Z 방향 이동 값을 결정
        """
        try:
            if hand_landmarks is None:
                self.get_logger().warning("No hand landmarks detected for right hand.")
                return None

            self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
            
            finger_tips = [
                self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
                self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                self.mp_hands.HandLandmark.RING_FINGER_TIP,
                self.mp_hands.HandLandmark.PINKY_TIP
            ]
            
            finger_mcp = [
                self.mp_hands.HandLandmark.INDEX_FINGER_MCP,
                self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP,
                self.mp_hands.HandLandmark.RING_FINGER_MCP,
                self.mp_hands.HandLandmark.PINKY_MCP
            ]
            
            # 주먹 감지 로직
            is_fist = all(hand_landmarks.landmark[tip].y > hand_landmarks.landmark[mcp].y for tip, mcp in zip(finger_tips, finger_mcp))

            if is_fist:
                cv2.putText(frame, 'Stop', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)
                return 2
            else:
                thumb_x = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].x
                ring_x = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].x
                
                if thumb_x > ring_x:
                    cv2.putText(frame, 'Back', (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
                    return 0
                else:
                    cv2.putText(frame, 'Go', (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
                    return 1

        except Exception as e:
            self.get_logger().error(f"Error in process_delta_z: {e}")
            return None

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motion_publisher = DetectMotion()
    
    # MultiThreadedExecutor를 사용하여 액션 서버와 퍼블리셔를 동시에 실행
    executor = MultiThreadedExecutor()
    executor.add_node(motion_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        motion_publisher.get_logger().info("Node interrupted by user.")
    finally:
        motion_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
