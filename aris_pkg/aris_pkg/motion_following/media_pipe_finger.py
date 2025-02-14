# import cv2
# import numpy as np
# import mediapipe as mp
# import rclpy
# from rclpy.node import Node
# from my_first_pkg_msgs.msg import Delta
# import math
# import threading
# class DetectMotion(Node):
#     def __init__(self):
#         super().__init__('motion_publisher')
#         self.publisher = self.create_publisher(Delta, '/motion_topic', 10)
#         self.timer_period = 0.1  # 발행 주기 (0.1초)
#         self.previous_wrist_x = None
#         self.previous_wrist_y = None
#         self.before_gripper = False
#         self.cap = cv2.VideoCapture(0)  # 카메라 초기화
#         # Mediapipe 초기화
#         self.mp_hands = mp.solutions.hands
#         self.hands = self.mp_hands.Hands(
#             static_image_mode=False,
#             model_complexity=1,
#             min_detection_confidence=0.6,
#             min_tracking_confidence=0.6
#         )
#         self.mp_drawing = mp.solutions.drawing_utils
#         # 타이머 설정
#         self.timer = self.create_timer(self.timer_period, self.detect_and_publish)
#     def detect_and_publish(self):
#         try:
#             ret, frame = self.cap.read()
#             if not ret:
#                 self.get_logger().warning("Failed to grab frame.")
#                 return
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#             results = self.hands.process(frame_rgb)
#             delta_x, delta_y, gripper_state, delta_z = None, None, None, None
#             if results.multi_hand_landmarks and results.multi_handedness:
#                 largest_left_hand = None
#                 largest_right_hand = None
#                 largest_left_area = 0
#                 largest_right_area = 0
#                 left_bbox = None
#                 right_bbox = None
#                 # 모든 감지된 손에 대해 처리
#                 for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
#                     label = handedness.classification[0].label  # 'Left' 또는 'Right'
#                     hand_area, bbox_coords = self.calculate_hand_area(hand_landmarks, frame.shape)
#                     if label == 'Left' and hand_area > largest_left_area:
#                         largest_left_area = hand_area
#                         largest_left_hand = hand_landmarks
#                         left_bbox = bbox_coords  # 왼손 바운딩 박스 저장
#                     elif label == 'Right' and hand_area > largest_right_area:
#                         largest_right_area = hand_area
#                         largest_right_hand = hand_landmarks
#                         right_bbox = bbox_coords  # 오른손 바운딩 박스 저장
#                 # 왼손과 오른손 바운딩 박스가 겹치는지 확인
#                 hands_overlap = self.check_overlap(left_bbox, right_bbox) if (left_bbox and right_bbox) else False
#                 if hands_overlap:
#                     self.get_logger().info("Hands are overlapping, skipping gesture detection.")
#                     return  # 손이 겹치는 경우 바로 종료하여 판단하지 않음
#                 # 왼손 감지 시 위치 변화량 및 그리퍼 상태 감지
#                 if largest_left_hand:
#                     delta_x, delta_y, gripper_state = self.process_hand(frame, largest_left_hand)
#                 else:
#                     self.get_logger().info("No left hand detected.")
#                 # 오른손 감지 시 제스처 기반 Z 방향 이동 감지
#                 if largest_right_hand:
#                     delta_z = self.process_delta_z(frame, largest_right_hand)
#                 else:
#                     self.get_logger().info("No right hand detected.")

#                 if gripper_state is not None:
#                     self.before_gripper = gripper_state
                    
#                 else:
#                     gripper_state = self.before_gripper
#                     print('ㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜ')

#                 print(f"self.before_gripper : {self.before_gripper}")
#                 # 메시지 발행
#                 if None not in (delta_x, delta_y, gripper_state, delta_z):
#                     msg = Delta()
#                     msg.x = float(delta_x)
#                     msg.y = float(delta_y)
#                     msg.z = int(delta_z)
#                     msg.gripper = gripper_state
#                     self.publisher.publish(msg)
#                 elif None not in (delta_x, delta_y, gripper_state):
#                     msg = Delta()
#                     msg.x = float(delta_x)
#                     msg.y = float(delta_y)
#                     msg.gripper = gripper_state
#                     self.publisher.publish(msg)

#                 elif None not in (delta_z,):
#                     msg = Delta()
#                     msg.z = int(delta_z)
#                     msg.gripper = gripper_state
#                     self.publisher.publish(msg)

#             cv2.imshow('Hand Detection', frame)
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 self.destroy_node()
#         except Exception as e:
#             self.get_logger().error(f"Unexpected error in detect_and_publish: {e}")
#     def calculate_hand_area(self, hand_landmarks, frame_shape):
#         try:
#             image_height, image_width, _ = frame_shape
#             x_coords = [landmark.x * image_width for landmark in hand_landmarks.landmark]
#             y_coords = [landmark.y * image_height for landmark in hand_landmarks.landmark]
#             x_min, x_max = min(x_coords), max(x_coords)
#             y_min, y_max = min(y_coords), max(y_coords)
#             area = (x_max - x_min) * (y_max - y_min)
#             bbox_coords = (int(x_min), int(y_min), int(x_max), int(y_max))
#             return area, bbox_coords
#         except Exception as e:
#             self.get_logger().error(f"Error calculating hand area: {e}")
#             return 0, (0, 0, 0, 0)
#     def check_overlap(self, bbox1, bbox2):
#         """
#         두 바운딩 박스가 겹치는지 확인
#         :param bbox1: (x_min1, y_min1, x_max1, y_max1) - 첫 번째 바운딩 박스 (왼손)
#         :param bbox2: (x_min2, y_min2, x_max2, y_max2) - 두 번째 바운딩 박스 (오른손)
#         :return: 겹치면 True, 겹치지 않으면 False
#         """
#         if not bbox1 or not bbox2:
#             return False  # 둘 중 하나라도 없으면 겹치지 않음
#         x_min1, y_min1, x_max1, y_max1 = bbox1
#         x_min2, y_min2, x_max2, y_max2 = bbox2
#         # 바운딩 박스 겹침 여부 확인
#         overlap_x = max(x_min1, x_min2) < min(x_max1, x_max2)
#         overlap_y = max(y_min1, y_min2) < min(y_max1, y_max2)
#         return overlap_x and overlap_y
#     def process_delta_z(self, frame, hand_landmarks):
#         """
#         오른손의 제스처에 따라 Z 방향 이동 값을 결정
#         """
#         try:
#             if hand_landmarks is None:
#                 self.get_logger().warning("No hand landmarks detected for right hand.")
#                 return None
#             self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
#             finger_tips = [
#                 self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
#                 self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
#                 self.mp_hands.HandLandmark.RING_FINGER_TIP,
#                 self.mp_hands.HandLandmark.PINKY_TIP
#             ]
#             finger_mcp = [
#                 self.mp_hands.HandLandmark.INDEX_FINGER_MCP,
#                 self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP,
#                 self.mp_hands.HandLandmark.RING_FINGER_MCP,
#                 self.mp_hands.HandLandmark.PINKY_MCP
#             ]
#             # 주먹 감지 로직
#             is_fist = all(hand_landmarks.landmark[tip].y > hand_landmarks.landmark[mcp].y for tip, mcp in zip(finger_tips, finger_mcp))
#             if is_fist:
#                 cv2.putText(frame, 'Stop', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)
#                 return 2
#             else:
#                 thumb_x = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].x
#                 ring_x = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].x
#                 if thumb_x > ring_x:
#                     cv2.putText(frame, 'Back', (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
#                     return 0
#                 else:
#                     cv2.putText(frame, 'Go', (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
#                     return 1
#         except Exception as e:
#             self.get_logger().error(f"Error in process_delta_z: {e}")
#             return None
#     def process_hand(self, frame, hand_landmarks):
#         """
#         왼손의 위치 변화량 (X, Y) 및 그리퍼 상태 감지
#         """
#         try:
#             if hand_landmarks is None:
#                 self.get_logger().warning("No hand landmarks detected for left hand.")
#                 return None, None, None
#             wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
#             wrist_x = int(wrist.x * frame.shape[1])
#             wrist_y = int(wrist.y * frame.shape[0])
#             base_x = wrist_x
#             base_y = frame.shape[0] - wrist_y
#             delta_x, delta_y = 0, 0
#             if self.previous_wrist_x is not None and self.previous_wrist_y is not None:
#                 delta_x = base_x - self.previous_wrist_x
#                 delta_y = base_y - self.previous_wrist_y
#             self.previous_wrist_x, self.previous_wrist_y = base_x, base_y
#             # 그리퍼(손가락 벌림/닫힘) 상태 감지
#             thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
#             index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
#             thumb_tip_x = int(thumb_tip.x * frame.shape[1])
#             thumb_tip_y = int(thumb_tip.y * frame.shape[0])
#             index_tip_x = int(index_tip.x * frame.shape[1])
#             index_tip_y = int(index_tip.y * frame.shape[0])
#             distance_thumb_index = math.hypot(index_tip_x - thumb_tip_x, index_tip_y - thumb_tip_y)
#             gripper_threshold = 20
#             print(f"gripper_state: {distance_thumb_index}")
#             gripper_state = distance_thumb_index < gripper_threshold
#             # 시각화 (겹치지 않도록 위치 조정)
#             cv2.putText(frame, f'Delta X: {delta_x:.2f}', (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#             cv2.putText(frame, f'Delta Y: {delta_y:.2f}', (20, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#             cv2.putText(frame, f'Gripper: {"Closed" if gripper_state else "Open"}',
#                         (20, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
#             return delta_x, delta_y, gripper_state
#         except Exception as e:
#             self.get_logger().error(f"Error in process_hand: {e}")
#             return None, None, None
#     def destroy_node(self):
#         self.cap.release()
#         cv2.destroyAllWindows()
#         super().destroy_node()
#     def motion_following(self):
#             print('와 신난다')
#             num = self.capsule_check
#             self.mode = 'order_check'
#             if num in (1, 2, 3):
#                 self.mode = 'check_motion'
#                 self.motion_publisher = DetectMotion()  # DetectMotion 로그에 통화 가능하도록 추가
#                 threading.Thread(target=rclpy.spin, args=(self.motion_publisher,), daemon=True).start()
# def main(args=None):
#     rclpy.init(args=args)
#     motion_publisher = DetectMotion()
#     try:
#         rclpy.spin(motion_publisher)
#     except KeyboardInterrupt:
#         motion_publisher.get_logger().info("Node interrupted by user.")
#     finally:
#         motion_publisher.destroy_node()
#         rclpy.shutdown()
# if __name__ == '__main__':
#     main()


import cv2
import time
import numpy as np
import mediapipe as mp
import rclpy
from rclpy.node import Node
from my_first_pkg_msgs.msg import Delta
import math
from my_first_pkg_msgs.action import MotionFlag
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

class DetectMotion(Node):

    def __init__(self):    
        super().__init__('motion_publisher')
        self.publisher = self.create_publisher(Delta, '/motion_topic', 10)
        self.action_server = ActionServer(self, MotionFlag, '/motion_following', self.execute_callback)
        
        self.previous_wrist_x = None
        self.previous_wrist_y = None
        self.timer_start = 0
        self.timer_end = 0
        self.receive_msg = True
        
        self.cap = cv2.VideoCapture(1)

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6
        )
        self.mp_drawing = mp.solutions.drawing_utils

    def clear_queue(self):
        self.get_logger().info("Clearing remaining queue...")
        self.previous_wrist_x = None
        self.previous_wrist_y = None
        self.timer_start = 0
        self.timer_end = 0
        self.get_logger().info("Queue cleared.")

    def execute_callback(self, goal_handle):
        feedback_msg = MotionFlag.Feedback()
        self.get_logger().info("Received start=True request. Starting motion detection.")

        if self.receive_msg:
            self.timer_start = time.time()
            self.receive_msg = False

        while rclpy.ok():
            self.timer_end = time.time()
            elapsed_time = self.timer_end - self.timer_start 

            self.detect_and_publish()
            feedback_msg.time = elapsed_time
            goal_handle.publish_feedback(feedback_msg)    

            if elapsed_time >= 30:
                self.get_logger().info("Motion detection completed after 10 seconds.")
                break

        goal_handle.succeed()
        result = MotionFlag.Result()
        result.end = True
        self.receive_msg = True
        self.clear_queue()
        
        return result

    def detect_and_publish(self):
        try:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warning("Failed to grab frame.")
                return

            if self.receive_msg:
                self.get_logger().warning("Motion Following not started.")
                return
            
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(frame_rgb)

            delta_x, delta_y, gripper_state, delta_z = None, None, None, None

            if results.multi_hand_landmarks and results.multi_handedness:
                largest_left_hand, largest_right_hand = None, None
                largest_left_area, largest_right_area = 0, 0
                left_bbox, right_bbox = None, None

                for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                    label = handedness.classification[0].label
                    hand_area, bbox_coords = self.calculate_hand_area(hand_landmarks, frame.shape)

                    if label == 'Left' and hand_area > largest_left_area:
                        largest_left_area, largest_left_hand, left_bbox = hand_area, hand_landmarks, bbox_coords
                    elif label == 'Right' and hand_area > largest_right_area:
                        largest_right_area, largest_right_hand, right_bbox = hand_area, hand_landmarks, bbox_coords

                if self.check_overlap(left_bbox, right_bbox):
                    self.get_logger().info("Hands are overlapping, skipping gesture detection.")
                    return

                if largest_left_hand:
                    delta_x, delta_y, gripper_state = self.process_hand(frame, largest_left_hand)
                if largest_right_hand:
                    delta_z = self.process_delta_z(frame, largest_right_hand)

                if None not in (delta_x, delta_y, gripper_state):
                    msg = Delta()
                    msg.x = float(delta_x)
                    msg.y = float(delta_y)
                    msg.gripper = gripper_state
                    if delta_z is not None:
                        msg.z = int(delta_z)
                    self.publisher.publish(msg)

            cv2.imshow('Hand Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.destroy_node()
        
        except Exception as e:
            self.get_logger().error(f"Unexpected error in detect_and_publish: {e}")

    def calculate_hand_area(self, hand_landmarks, frame_shape):
        image_height, image_width, _ = frame_shape
        x_coords = [landmark.x * image_width for landmark in hand_landmarks.landmark]
        y_coords = [landmark.y * image_height for landmark in hand_landmarks.landmark]
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        area = (x_max - x_min) * (y_max - y_min)
        return area, (int(x_min), int(y_min), int(x_max), int(y_max))

    def check_overlap(self, bbox1, bbox2):
        if not bbox1 or not bbox2:
            return False
        x_min1, y_min1, x_max1, y_max1 = bbox1
        x_min2, y_min2, x_max2, y_max2 = bbox2
        return max(x_min1, x_min2) < min(x_max1, x_max2) and max(y_min1, y_min2) < min(y_max1, y_max2)

    def process_delta_z(self, frame, hand_landmarks):
        try:
            self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
            finger_tips = [self.mp_hands.HandLandmark.INDEX_FINGER_TIP, self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP, 
                           self.mp_hands.HandLandmark.RING_FINGER_TIP, self.mp_hands.HandLandmark.PINKY_TIP]
            finger_mcp = [self.mp_hands.HandLandmark.INDEX_FINGER_MCP, self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP, 
                          self.mp_hands.HandLandmark.RING_FINGER_MCP, self.mp_hands.HandLandmark.PINKY_MCP]
            is_fist = all(hand_landmarks.landmark[tip].y > hand_landmarks.landmark[mcp].y for tip, mcp in zip(finger_tips, finger_mcp))
            if is_fist:
                cv2.putText(frame, 'Stop', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)
                return 2
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

    def process_hand(self, frame, hand_landmarks):
        try:
            wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
            wrist_x = int(wrist.x * frame.shape[1])
            wrist_y = int(wrist.y * frame.shape[0])

            base_x, base_y = wrist_x, frame.shape[0] - wrist_y
            delta_x = base_x - self.previous_wrist_x if self.previous_wrist_x is not None else 0
            delta_y = base_y - self.previous_wrist_y if self.previous_wrist_y is not None else 0
            self.previous_wrist_x, self.previous_wrist_y = base_x, base_y

            thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
            index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
            distance_thumb_index = math.hypot(index_tip.x * frame.shape[1] - thumb_tip.x * frame.shape[1], 
                                              index_tip.y * frame.shape[0] - thumb_tip.y * frame.shape[0])
            gripper_state = distance_thumb_index < 50

            cv2.putText(frame, f'Delta X: {delta_x:.2f}', (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f'Delta Y: {delta_y:.2f}', (20, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f'Gripper: {"Closed" if gripper_state else "Open"}', (20, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            return delta_x, delta_y, gripper_state
        except Exception as e:
            self.get_logger().error(f"Error in process_hand: {e}")
            return None, None, None

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motion_publisher = DetectMotion()
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
