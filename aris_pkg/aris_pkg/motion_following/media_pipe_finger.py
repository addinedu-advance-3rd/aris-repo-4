# import cv2
# import numpy as np
# import mediapipe as mp
# import threading
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Float32
# from my_first_pkg_msgs.msg import Delta
# import numpy as np

# class DetectMotion(Node):
    
#     def __init__(self):    
#         super().__init__('motion_publisher')
#         self.publisher = self.create_publisher(Delta, '/motion_topic', 10)
#         self.timer_period = 0.5  # seconds
#         self.previous_wrist_x = None
#         self.previous_wrist_y = None
#         self.cap = cv2.VideoCapture(0)  # Initialize camera
        
#         # # Thread for the detection loop
#         # self.thread = threading.Thread(target=self.detect_and_extract_hand_real_time, daemon=True)
#         # self.thread.start()

#     def detect_and_extract_hand_real_time(self):
#         try:
#             mp_hands = mp.solutions.hands
#             hands = mp_hands.Hands(static_image_mode=False, 
#                                    model_complexity=1, 
#                                    min_detection_confidence=0.6, 
#                                    min_tracking_confidence=0.6)
#             mp_drawing = mp.solutions.drawing_utils
            
#             delta_x, delta_y = 0, 0  # Default initialization for delta_x and delta_y

#             while True:
#                 ret, frame = self.cap.read()
#                 if not ret:
#                     self.get_logger().warning("Failed to grab frame.")
#                     break

#                 frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#                 results = hands.process(frame_rgb)

#                 if results.multi_hand_landmarks:
#                     for hand_landmarks in results.multi_hand_landmarks:
#                         wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
#                         wrist_x = int(wrist.x * frame.shape[1])
#                         wrist_y = int(wrist.y * frame.shape[0])

#                         base_x = wrist_x
#                         base_y = frame.shape[0] - wrist_y

#                         if self.previous_wrist_x is not None and self.previous_wrist_y is not None:
#                             delta_x = base_x - self.previous_wrist_x
#                             delta_y = base_y - self.previous_wrist_y
                        
#                         self.previous_wrist_x, self.previous_wrist_y = base_x, base_y

#                         print("Done00~~~~~~~~~~~~~~~")

#                         print(type(delta_x), delta_x)
#                         print(type(delta_y), delta_y)

#                         msg = Delta()
#                         msg.x = float(delta_x)  # Ensure float type
#                         msg.y = float(delta_y)  # Ensure float type

#                         print(type(msg.x), msg.x)
#                         print(type(msg.y), msg.y)

#                         print(msg)
#                         self.publisher.publish(msg)

#                         # Display the frame for debugging
#                         cv2.circle(frame, (wrist_x, wrist_y), 10, (0, 255, 0), -1)
#                         cv2.putText(frame, f'Delta: ({delta_x}, {delta_y})', 
#                                     (wrist_x + 20, wrist_y - 20), 
#                                     cv2.FONT_HERSHEY_SIMPLEX, 1, 
#                                     (0, 255, 255), 2, cv2.LINE_AA)
#                 cv2.imshow("Hand Detection", frame)
                
#                 # Exit loop on 'q' key press
#                 if cv2.waitKey(1) & 0xFF == ord('q'):
#                     break

#         except Exception as e:
#             self.get_logger().error(f"Error in detection loop: {e}")
#         finally:
#             hands.close()
#             self.cap.release()
#             cv2.destroyAllWindows()

# def main(args=None):
#     rclpy.init(args=args)
#     motion_publisher = DetectMotion()
#     try:
#         motion_publisher.detect_and_extract_hand_real_time()
#         rclpy.spin(motion_publisher)
#     except KeyboardInterrupt:
#         motion_publisher.get_logger().info("Node interrupted by user.")
#     finally:
#         motion_publisher.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import cv2
# import numpy as np
# import mediapipe as mp
# import rclpy
# from rclpy.node import Node
# from my_first_pkg_msgs.msg import Delta
# import math

# class DetectMotion(Node):

#     def __init__(self):    
#         super().__init__('motion_publisher')
#         self.publisher = self.create_publisher(Delta, '/motion_topic', 10)
#         self.timer_period = 0.1  # 발행 주기 (1초)
#         self.previous_wrist_x = None
#         self.previous_wrist_y = None
#         self.cap = cv2.VideoCapture(0)  # Initialize camera

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
#         ret, frame = self.cap.read()
#         if not ret:
#             self.get_logger().warning("Failed to grab frame.")
#             return

#         # RGB로 변환 및 Mediapipe 처리
#         frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         results = self.hands.process(frame_rgb)

#         if results.multi_hand_landmarks:
#             for hand_landmarks in results.multi_hand_landmarks:
#                 wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
#                 wrist_x = int(wrist.x * frame.shape[1])
#                 wrist_y = int(wrist.y * frame.shape[0])

#                 base_x = wrist_x
#                 base_y = frame.shape[0] - wrist_y

#                 delta_x, delta_y = 0, 0
#                 if self.previous_wrist_x is not None and self.previous_wrist_y is not None:
#                     delta_x = base_x - self.previous_wrist_x
#                     delta_y = base_y - self.previous_wrist_y

#                 # Update previous positions
#                 self.previous_wrist_x, self.previous_wrist_y = base_x, base_y



#                 # 엄지와 검지 좌표 추출
#                 thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
#                 index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

#                 thumb_tip_x = int(thumb_tip.x * frame.shape[1])
#                 thumb_tip_y = int(thumb_tip.y * frame.shape[0])

#                 index_tip_x = int(index_tip.x * frame.shape[1])
#                 index_tip_y = int(index_tip.y * frame.shape[0])


#                 # 유클리디안 거리 계산
#                 distance = math.sqrt((index_tip_x - thumb_tip_x) ** 2 + (index_tip_y - thumb_tip_y) ** 2)


#                 # Delta 메시지 생성 및 발행
#                 msg = Delta()
#                 msg.x = float(delta_x)
#                 msg.y = float(delta_y)
#                 msg.dis = float(distance)
#                 self.publisher.publish(msg)
#                 # Debug 메시지
#                 self.get_logger().info(f"Published Delta: x={msg.x}, y={msg.y}")


#                 # 디버깅용 화면에 점 그리기
#                 cv2.circle(frame, (wrist_x, wrist_y), 10, (0, 255, 0), -1)  # 손목
#                 cv2.circle(frame, (thumb_tip_x, thumb_tip_y), 10, (0, 0, 255), -1)  # 엄지끝
#                 cv2.circle(frame, (index_tip_x, index_tip_y), 10, (255, 0, 0), -1)  # 검지끝

#                 # 텍스트 표시
#                 cv2.putText(frame, f'Delta: ({delta_x}, {delta_y})', 
#                             (wrist_x + 20, wrist_y - 20), 
#                             cv2.FONT_HERSHEY_SIMPLEX, 1, 
#                             (0, 255, 255), 2, cv2.LINE_AA)

#                 cv2.putText(frame, f'Thumb: ({thumb_tip_x}, {thumb_tip_y})', 
#                             (thumb_tip_x + 20, thumb_tip_y - 20), 
#                             cv2.FONT_HERSHEY_SIMPLEX, 1, 
#                             (255, 0, 0), 2, cv2.LINE_AA)

#                 cv2.putText(frame, f'Index: ({index_tip_x}, {index_tip_y})', 
#                             (index_tip_x + 20, index_tip_y - 20), 
#                             cv2.FONT_HERSHEY_SIMPLEX, 1, 
#                             (255, 255, 0), 2, cv2.LINE_AA)

#                 # 유클리디안 거리 표시
#                 cv2.putText(frame, f'Distance: {distance:.2f}px', 
#                             (wrist_x + 20, wrist_y + 30),  # 위치를 손목 주변에 적절히 이동
#                             cv2.FONT_HERSHEY_SIMPLEX, 1, 
#                             (0, 255, 255), 2, cv2.LINE_AA)  # 밝은 색으로 구분

#         # 프레임 디스플레이
#         cv2.imshow("Hand Detection", frame)

#         # 'q' 키로 종료
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             self.destroy_node()

#     def destroy_node(self):
#         self.cap.release()
#         cv2.destroyAllWindows()
#         super().destroy_node()

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
        self.timer_period = 0.1  # 발행 주기 (1초)
        self.previous_wrist_x = None
        self.previous_wrist_y = None
        self.cap = cv2.VideoCapture(0)  # Initialize camera

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

        # 프레임 크기 가져오기
        frame_height, frame_width, _ = frame.shape

        # # 중앙 ROI 설정 (사각형의 크기는 조정 가능)
        # roi_size = 400
        # roi_x1 = frame_width // 2 - roi_size // 2
        # roi_y1 = frame_height // 2 - roi_size // 2
        # roi_x2 = frame_width // 2 + roi_size // 2
        # roi_y2 = frame_height // 2 + roi_size // 2


        # # ROI 영역 그리기
        # cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 0), 2)

        # RGB로 변환 및 Mediapipe 처리
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                wrist_x = int(wrist.x * frame.shape[1])
                wrist_y = int(wrist.y * frame.shape[0])

                base_x = wrist_x
                base_y = frame.shape[0] - wrist_y

                # ROI 내에 손목 좌표가 있는지 확인
                # if roi_x1 <= wrist_x <= roi_x2 and roi_y1 <= wrist_y <= roi_y2:
                if True:
                    delta_x, delta_y = 0, 0
                    if self.previous_wrist_x is not None and self.previous_wrist_y is not None:
                        delta_x = base_x - self.previous_wrist_x
                        delta_y = base_y - self.previous_wrist_y

                    # Update previous positions
                    self.previous_wrist_x, self.previous_wrist_y = base_x, base_y

                    # 엄지와 검지 좌표 추출
                    thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
                    index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

                    thumb_tip_x = int(thumb_tip.x * frame.shape[1])
                    thumb_tip_y = int(thumb_tip.y * frame.shape[0])

                    index_tip_x = int(index_tip.x * frame.shape[1])
                    index_tip_y = int(index_tip.y * frame.shape[0])

                    # 유클리디안 거리 계산
                    distance = math.sqrt((index_tip_x - thumb_tip_x) ** 2 + (index_tip_y - thumb_tip_y) ** 2)
                    if abs(delta_x) < 100 and  abs(delta_y)<100: 
                        # Delta 메시지 생성 및 발행
                        msg = Delta()
                        msg.x = float(delta_x)
                        msg.y = float(delta_y)
                        msg.dis = float(distance)
                        self.publisher.publish(msg)

                        # Debug 메시지
                        self.get_logger().info(f"Published Delta: x={msg.x}, y={msg.y}, dis={msg.dis}")

                        # 디버깅용 화면에 점 그리기
                        cv2.circle(frame, (wrist_x, wrist_y), 10, (0, 255, 0), -1)  # 손목
                        cv2.circle(frame, (thumb_tip_x, thumb_tip_y), 10, (0, 0, 255), -1)  # 엄지끝
                        cv2.circle(frame, (index_tip_x, index_tip_y), 10, (255, 0, 0), -1)  # 검지끝

                        # 텍스트 표시
                        cv2.putText(frame, f'Delta: ({delta_x}, {delta_y})', 
                                    (wrist_x + 20, wrist_y - 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                    (0, 255, 255), 2, cv2.LINE_AA)

                        cv2.putText(frame, f'Distance: {distance:.2f}px', 
                                    (wrist_x + 20, wrist_y + 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                    (0, 255, 255), 2, cv2.LINE_AA)
                    else:
                        self.get_logger().info("Wrist is outside of ROI.")

        # 프레임 디스플레이
        cv2.imshow("Hand Detection with ROI", frame)

        # 'q' 키로 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()

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

# import cv2
# import numpy as np
# import mediapipe as mp
# import rclpy
# from rclpy.node import Node
# from my_first_pkg_msgs.msg import Delta
# import math

# class DetectMotion(Node):

#     def __init__(self):    
#         super().__init__('motion_publisher')
#         self.publisher = self.create_publisher(Delta, '/motion_topic', 10)
#         self.timer_period = 0.1  # 발행 주기 (1초)
#         self.previous_wrist_x = None
#         self.previous_wrist_y = None
#         self.cap = cv2.VideoCapture(0)  # Initialize camera

#         # Mediapipe 초기화
#         self.mp_hands = mp.solutions.hands
#         self.hands = self.mp_hands.Hands(
#             static_image_mode=False,
#             model_complexity=1,
#             min_detection_confidence=0.6,
#             min_tracking_confidence=0.6
#         )
#         self.mp_pose = mp.solutions.pose
#         self.pose = self.mp_pose.Pose(
#             static_image_mode=False,
#             model_complexity=2,
#             min_detection_confidence=0.6,
#             min_tracking_confidence=0.6
#         )
#         self.mp_drawing = mp.solutions.drawing_utils

#         # 타이머 설정
#         self.timer = self.create_timer(self.timer_period, self.detect_and_publish)

#     def detect_and_publish(self):
#         ret, frame = self.cap.read()
#         if not ret:
#             self.get_logger().warning("Failed to grab frame.")
#             return

#         # 프레임 크기 가져오기
#         frame_height, frame_width, _ = frame.shape

#         # 중앙 ROI 설정 (사각형의 크기는 조정 가능)
#         roi_size = 300
#         roi_x1 = frame_width // 2 - roi_size // 2
#         roi_y1 = frame_height // 2 - roi_size // 2
#         roi_x2 = frame_width // 2 + roi_size // 2
#         roi_y2 = frame_height // 2 + roi_size // 2

#         # ROI 영역 그리기
#         cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 0), 2)

#         # RGB로 변환 및 Mediapipe 처리
#         frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         results_hands = self.hands.process(frame_rgb)
#         results_pose = self.pose.process(frame_rgb)

#         if results_pose.pose_landmarks:
#             # For 3D distance calculation: Use wrist or any other landmark
#             wrist_landmark = results_pose.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST]  # Example using the right wrist
#             wrist_x = int(wrist_landmark.x * frame.shape[1])
#             wrist_y = int(wrist_landmark.y * frame.shape[0])
#             wrist_z = wrist_landmark.z  # The Z coordinate is used to estimate the distance

#             # Distance threshold (e.g., wrist too close if Z < -0.5)
#             if wrist_z < -0.7:  # This value may need adjustment based on testing
#                 cv2.putText(frame, 'Warning: Wrist too close to camera!', (wrist_x + 20, wrist_y - 20), 
#                             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

#         if results_hands.multi_hand_landmarks:
#             for hand_landmarks in results_hands.multi_hand_landmarks:
#                 wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
#                 wrist_x = int(wrist.x * frame.shape[1])
#                 wrist_y = int(wrist.y * frame.shape[0])

#                 base_x = wrist_x
#                 base_y = frame.shape[0] - wrist_y

#                 # ROI 내에 손목 좌표가 있는지 확인
#                 if roi_x1 <= wrist_x <= roi_x2 and roi_y1 <= wrist_y <= roi_y2:
#                     delta_x, delta_y = 0, 0
#                     if self.previous_wrist_x is not None and self.previous_wrist_y is not None:
#                         delta_x = base_x - self.previous_wrist_x
#                         delta_y = base_y - self.previous_wrist_y

#                     # Update previous positions
#                     self.previous_wrist_x, self.previous_wrist_y = base_x, base_y

#                     # 엄지와 검지 좌표 추출
#                     thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
#                     index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

#                     thumb_tip_x = int(thumb_tip.x * frame.shape[1])
#                     thumb_tip_y = int(thumb_tip.y * frame.shape[0])

#                     index_tip_x = int(index_tip.x * frame.shape[1])
#                     index_tip_y = int(index_tip.y * frame.shape[0])

#                     # 유클리디안 거리 계산
#                     distance = math.sqrt((index_tip_x - thumb_tip_x) ** 2 + (index_tip_y - thumb_tip_y) ** 2)

#                     # Delta 메시지 생성 및 발행
#                     msg = Delta()
#                     msg.x = float(delta_x)
#                     msg.y = float(delta_y)
#                     msg.dis = float(distance)
#                     self.publisher.publish(msg)

#                     # Debug 메시지
#                     self.get_logger().info(f"Published Delta: x={msg.x}, y={msg.y}, dis={msg.dis}")

#                     # 디버깅용 화면에 점 그리기
#                     cv2.circle(frame, (wrist_x, wrist_y), 10, (0, 255, 0), -1)  # 손목
#                     cv2.circle(frame, (thumb_tip_x, thumb_tip_y), 10, (0, 0, 255), -1)  # 엄지끝
#                     cv2.circle(frame, (index_tip_x, index_tip_y), 10, (255, 0, 0), -1)  # 검지끝

#                     # 텍스트 표시
#                     cv2.putText(frame, f'Delta: ({delta_x}, {delta_y})', 
#                                 (wrist_x + 20, wrist_y - 20), 
#                                 cv2.FONT_HERSHEY_SIMPLEX, 1, 
#                                 (0, 255, 255), 2, cv2.LINE_AA)

#                     cv2.putText(frame, f'Distance: {distance:.2f}px', 
#                                 (wrist_x + 20, wrist_y + 30), 
#                                 cv2.FONT_HERSHEY_SIMPLEX, 1, 
#                                 (0, 255, 255), 2, cv2.LINE_AA)
#                 else:
#                     self.get_logger().info("Wrist is outside of ROI.")

#         # 프레임 디스플레이
#         cv2.imshow("Hand Detection with ROI", frame)

#         # 'q' 키로 종료
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             self.destroy_node()

#     def destroy_node(self):
#         self.cap.release()
#         cv2.destroyAllWindows()
#         super().destroy_node()

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
