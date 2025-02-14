import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from my_first_pkg_msgs.msg import Delta
import math
import threading

class DetectMotion(Node):

    def __init__(self, webcam1_id=0, webcam2_id=2):    
        super().__init__('motion_publisher')
        self.publisher = self.create_publisher(Delta, '/motion_topic', 10)
        self.timer_period = 0.1  # Publishing period (0.1 seconds)
        self.previous_wrist_x = None
        self.previous_wrist_y = None
        self.current_distance_status = "Hand Distance Stable"  # Initial state is Stable
        self.hand_area = 2  # Initial hand area status

        # OpenCV webcam setup
        self.cap1 = cv2.VideoCapture(webcam1_id)
        self.cap2 = cv2.VideoCapture(webcam2_id)

        # Check if webcams are opened successfully
        if not self.cap1.isOpened() or not self.cap2.isOpened():
            self.get_logger().error("Failed to open one or both webcams.")
            rclpy.shutdown()

        # Mediapipe initialization
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6
        )
        self.mp_drawing = mp.solutions.drawing_utils

        # Thread setup
        self.stop_event = threading.Event()
        self.image_processing_thread = threading.Thread(target=self.process_images)
        self.image_processing_thread.start()

    def detect_and_publish(self, frame, mode):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        if results.multi_hand_landmarks:
            largest_hand = None
            largest_area = 0
            largest_bbox = (0, 0, 0, 0)
            largest_landmarks = None
            delta_x = None
            delta_y = None
            delta_z = None

            for hand_landmarks in results.multi_hand_landmarks:
                hand_area, bbox_coords = self.calculate_hand_area(hand_landmarks, frame.shape)
                if hand_area > largest_area:
                    largest_area = hand_area
                    largest_bbox = bbox_coords
                    largest_landmarks = hand_landmarks

            if largest_landmarks:
                if mode == 1:
                    delta_x, delta_y = self.process_hand(frame, largest_landmarks, largest_area, largest_bbox)
                    self.publish_motion(delta_x, delta_y, 0)
                elif mode == 2:
                    delta_z = self.process_delta_z(frame, largest_landmarks, largest_area, largest_bbox)
                    self.publish_motion(0, 0, delta_z)
            if (delta_x is not None) and (delta_y is not None) and (delta_z is not None):
                self.publish_motion(delta_x, delta_y, delta_z)
        if mode == 1:
            cv2.imshow("Webcam 1 - Hand Detection", frame)
        elif mode == 2:
            cv2.imshow("Webcam 2 - Hand Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stop_event.set()
            self.destroy_node()

    def process_hand(self, frame, hand_landmarks, hand_area, bbox_coords):
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        wrist_x = int(wrist.x * frame.shape[1])
        wrist_y = int(wrist.y * frame.shape[0])

        delta_x, delta_y = 0, 0
        if self.previous_wrist_x is not None and self.previous_wrist_y is not None:
            delta_x = wrist_x - self.previous_wrist_x
            delta_y = wrist_y - self.previous_wrist_y

        self.previous_wrist_x, self.previous_wrist_y = wrist_x, wrist_y

        cv2.putText(frame, f'Delta X: {delta_x:.2f}, Delta Y: {delta_y:.2f}', 
                    (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Draw wrist point
        cv2.circle(frame, (wrist_x, wrist_y), 10, (0, 255, 0), -1)
        return delta_x, delta_y

    def process_delta_z(self, frame, hand_landmarks, hand_area, bbox_coords):
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        wrist_x = int(wrist.x * frame.shape[1])
        wrist_y = int(wrist.y * frame.shape[0])

        # delta_x, delta_y = 0, 0
        delta_z = 0
        if self.previous_wrist_x is not None and self.previous_wrist_y is not None:
            delta_z = wrist_y - self.previous_wrist_y
        self.previous_wrist_x, self.previous_wrist_y = wrist_x, wrist_y

        cv2.putText(frame, f'Delta Z: {delta_z:.2f}', 
                    (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Draw wrist point
        cv2.circle(frame, (wrist_x, wrist_y), 10, (0, 255, 0), -1)
        return delta_z

    def publish_motion(self, delta_x, delta_y, delta_z):
        msg = Delta()
        msg.x = float(delta_x)
        msg.y = float(delta_y)
        msg.dis = float(delta_z)
        self.publisher.publish(msg)

    def calculate_hand_area(self, hand_landmarks, frame_shape):
        x_coords = [int(lm.x * frame_shape[1]) for lm in hand_landmarks.landmark]
        y_coords = [int(lm.y * frame_shape[0]) for lm in hand_landmarks.landmark]

        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)

        width = max_x - min_x
        height = max_y - min_y

        area = width * height
        bbox_coords = (min_x, min_y, max_x, max_y)
        return area, bbox_coords

    def process_images(self):
        while not self.stop_event.is_set():
            ret1, frame1 = self.cap1.read()
            ret2, frame2 = self.cap2.read()

            if ret1:
                self.detect_and_publish(frame1, mode=1)
            if ret2:
                self.detect_and_publish(frame2, mode=2)

    def destroy_node(self):
        self.stop_event.set()
        self.cap1.release()
        self.cap2.release()
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
