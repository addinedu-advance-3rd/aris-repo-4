import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ultralytics import YOLO
from gtts import gTTS
import os
import threading
import time
from collections import Counter
from my_first_pkg_msgs.srv import CapsulePosition  # 서비스 메시지

class RobotMonitor(Node):
    def __init__(self, yolo_processor):
        super().__init__('robot_monitor')

        self.yolo_processor = yolo_processor
        # 서비스 서버 생성 (서비스 요청을 직접 처리)
        self.srv = self.create_service(CapsulePosition, '/process_order', self.process_order_callback)

        self.warning_publisher = self.create_publisher(Bool, '/warning', 10)

        # 경고 상태 관리 변수
        self.start_time = time.time()
        self.warning_state = False
        self.warning_time = None
        self.processing_time = 10
        
    def process_order_callback(self, request, response):
        self.get_logger().info(f"📡 Server requested capsule position. Request: {request.req}")
        bit_status = self.yolo_processor.get_bit_status()
        self.get_logger().info(f"📡 Capsule bit status: {bit_status}")
        
        if request.req:

            #bit_status = self.yolo_processor.get_bit_status()
            response.success = True
            response.message = bit_status
            
        else:
            response.success = False
            response.message = "Invalid request"
        return response

    def send_warning(self):
        msg = Bool()
        msg.data = self.warning_state
        self.warning_publisher.publish(msg)
        self.get_logger().info(f"⚠ Warning status sent: {self.warning_state}")


class YOLO_Processor:
    def __init__(self, model_path, roi_centers, roi_radius=15):
        self.model = YOLO(model_path)
        self.confidence_threshold = 0.8
        self.hand_color, self.arm_color = (0, 255, 0), (0, 0, 255)
        self.capsule_color = (255, 0, 0)
        self.WARNING_DISTANCE = 10
        self.audio_playing = False
        self.true_count = 0
        self.TARGET_COUNT = 5
        self.roi_centers = roi_centers
        self.roi_radius = roi_radius
        self.circle_status = {}

    def play_warning_audio(self):
        global audio_playing
        text = "경고! 위험하오니 뒤로 한 걸음 물러나 주세요."
        tts = gTTS(text=text, lang='ko', slow=False)
        tts.save("/home/addinedu/dev_ws/src/aris-repo-4/aris_pkg/aris_pkg/prohibit/warning.mp3")
        os.system("mpg321 /home/addinedu/dev_ws/src/aris-repo-4/aris_pkg/aris_pkg/prohibit/warning.mp3")
        time.sleep(4)
        self.audio_playing = False

    def check_capsule_in_roi(self, capsule_mask):
        circle_status = {}
        
        if capsule_mask is not None:
            for i, center in enumerate(self.roi_centers):
                mask = np.zeros_like(capsule_mask, dtype=np.uint8)
                cv2.circle(mask, center, self.roi_radius, 255, -1)

                if capsule_mask.shape != mask.shape:
                    capsule_mask = cv2.resize(capsule_mask, (mask.shape[1], mask.shape[0]))
                capsule_mask = capsule_mask.astype(np.uint8)

                intersection = cv2.bitwise_and(capsule_mask, mask)
                if np.any(intersection):  # 캡슐이 있으면
                    circle_status[f"circle{i+1}"] = 'O'
                else:
                    circle_status[f"circle{i+1}"] = 'X'

        self.circle_status = circle_status 
        return circle_status

    def process_frame(self, frame, robot_monitor):
        results = self.model(frame)
        result = results[0]
        masks = result.masks
        hand_mask, arm_mask = None, None
        capsule_mask = None


        CONFIDENCE_THRESHOLD = 0.8  # Set your desired threshold

        if masks is not None:
            mask_data = masks.data.cpu().numpy()
            
            for idx, mask in enumerate(mask_data):
                confidence = result.boxes[idx].conf  # Get the confidence score
                
                if confidence >= CONFIDENCE_THRESHOLD:
                    class_name = result.names[int(result.boxes[idx].cls)]
                    mask_img = (mask * 255).astype(np.uint8)

                    if class_name == 'hand':
                        frame[mask == 1] = self.hand_color
                        hand_mask = mask
                    elif class_name == 'robot':
                        frame[mask == 1] = self.arm_color
                        arm_mask = mask
                    elif class_name == 'capsule':
                        frame[mask == 1] = self.capsule_color
                        capsule_mask = mask
                else:
                    # Skip detection if confidence is below the threshold
                    continue


        # if masks is not None:
        #     mask_data = masks.data.cpu().numpy()
        #     for idx, mask in enumerate(mask_data):
        #         class_name = result.names[int(result.boxes[idx].cls)]
        #         mask_img = (mask * 255).astype(np.uint8)
        #         if class_name == 'hand':
        #             frame[mask == 1] = self.hand_color
        #             hand_mask = mask
        #         elif class_name == 'robot':
        #             frame[mask == 1] = self.arm_color
        #             arm_mask = mask
        #         elif class_name == 'capsule':
        #             frame[mask == 1] = self.capsule_color
        #             capsule_mask = mask

        # 원을 그리기 (ROI 영역을 시각적으로 표시)
        for i, center in enumerate(self.roi_centers):
            cv2.circle(frame, center, self.roi_radius, (0, 255, 0), 2)  # 원을 그린다
            
            # 캡슐 상태 확인 후 "O" 또는 "X" 텍스트 추가
            circle_status = self.check_capsule_in_roi(capsule_mask)
            status = circle_status.get(f"circle{i+1}", 'X')
            color = (0, 255, 0) if status == 'O' else (0, 0, 255)  # 'O'는 녹색, 'X'는 빨간색으로 표시
            cv2.putText(frame, status, (center[0] - 10, center[1] -35), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)

        # 손과 로봇 팔 간 거리 계산
        if hand_mask is not None and arm_mask is not None:
            arm_mask_uint8 = (arm_mask * 255).astype(np.uint8)
            distance_map = cv2.distanceTransform(255 - arm_mask_uint8, cv2.DIST_L2, 5)
            hand_coords = np.column_stack(np.where(hand_mask > 0))
            min_distance = np.min(distance_map[hand_coords[:, 0], hand_coords[:, 1]]) if len(hand_coords) > 0 else float('inf')

            if min_distance < self.WARNING_DISTANCE:
                self.true_count += 1
                if not robot_monitor.warning_state and self.true_count >= self.TARGET_COUNT:
                    robot_monitor.warning_state = True
                    robot_monitor.warning_time = time.time()
                    cv2.putText(frame, "⚠ Warning!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    if not self.audio_playing:
                        self.audio_playing = True
                        threading.Thread(target=self.play_warning_audio).start()
            else:
                if robot_monitor.warning_state:
                    self.true_count = 0
                    robot_monitor.warning_state = False
                    cv2.putText(frame, "⚠ Warning Cleared!", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        self.circle_status = ''.join(['0' if circle_status.get(f"circle{i+1}", 'X') == 'O' else '1' for i in range(3)])
        robot_monitor.get_logger().info(f"Capsule Status: {self.circle_status}")

        # 서비스 호출 예시
        # request = CapsulePosition.Request()
        # request.req = bit_status
        # client.call(request)

        robot_monitor.send_warning()
        return frame

    def get_bit_status(self):
        return self.circle_status


def main():
    rclpy.init()
    
    roi_centers = [(490, 113), (404, 112), (324, 118)]  # 원의 중심 좌표
    yolo_processor = YOLO_Processor("/home/addinedu/dev_ws/src/aris-repo-4/aris_pkg/aris_pkg/prohibit/best.pt", roi_centers)
    robot_monitor = RobotMonitor(yolo_processor)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        exit()

    print("웹캠 실행 중... 종료하려면 'q'를 누르세요.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다. 종료합니다.")
            break

        frame = yolo_processor.process_frame(frame, robot_monitor)
        rclpy.spin_once(robot_monitor, timeout_sec=0.01)
        cv2.imshow("Segmented Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()