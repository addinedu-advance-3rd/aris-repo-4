import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String  # :불: String 사용
from ultralytics import YOLO
from gtts import gTTS
import os
import threading
import time
class RobotMonitor(Node):
    def __init__(self):
        super().__init__('robot_monitor')
        # :불: String 메시지 사용
        self.pit_status_pub = self.create_publisher(String, '/pit_status', 10)
        self.warning_publisher = self.create_publisher(Bool, '/warning', 10)
        self.start_time = time.time()
        self.send_pit_status_flag = True
        self.prev_warning_state = False
    def send_warning(self, warning_state):
        if warning_state != self.prev_warning_state:
            msg = Bool()
            msg.data = warning_state
            self.warning_publisher.publish(msg)
            self.get_logger().info(f":경고: Warning status sent: {warning_state}")
            self.prev_warning_state = warning_state
    def send_pit_status(self, pit_status):
        if not self.send_pit_status_flag:
            return
        # :불: 피트 상태를 문자열로 변환
        pit_code = "".join([str(i+1) for i in range(3) if pit_status[f"Pit {i+1}"]])
        if pit_code == "":
            pit_code = "0"  # 아무 피트에도 없음
        msg = String()
        msg.data = pit_code
        self.pit_status_pub.publish(msg)
        self.get_logger().info(f":위성_안테나: Pit status sent: {pit_code}")
# YOLO 모델 로드
model_path = "/home/addinedu/dev_ws/runs/segment/train24/weights/best.pt"
session = YOLO(model_path)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()
print("웹캠 실행 중... 종료하려면 'q'를 누르세요.")
confidence_threshold = 0.7
hand_color, arm_color = (0, 255, 0), (0, 0, 255)
pit_colors = [(255, 0, 0), (255, 255, 0), (0, 255, 255)]
WARNING_DISTANCE = 10
audio_playing = False
def play_warning_audio():
    global audio_playing
    text = "경고! 위험하오니 뒤로 한 걸음 물러나 주세요."
    tts = gTTS(text=text, lang='ko', slow=False)
    tts.save("warning.mp3")
    os.system("mpg321 warning.mp3")
    time.sleep(4)
    audio_playing = False
rclpy.init()
robot_monitor = RobotMonitor()
while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다. 종료합니다.")
        break
    results = session(frame)
    result = results[0]
    masks = result.masks
    hand_mask, arm_mask = None, None
    pit_masks = [None, None, None]
    if masks is not None:
        mask_data = masks.data.cpu().numpy()
        for idx, mask in enumerate(mask_data):
            class_name = result.names[int(result.boxes[idx].cls)]
            mask_img = (mask * 255).astype(np.uint8)
            if class_name == 'hand':
                frame[mask == 1] = hand_color
                hand_mask = mask
            elif class_name == 'robot':
                frame[mask == 1] = arm_color
                arm_mask = mask
            elif class_name.startswith('pit_'):
                pit_idx = int(class_name[-1]) - 1
                frame[mask == 1] = pit_colors[pit_idx]
                pit_masks[pit_idx] = mask
    warning_state = False
    if hand_mask is not None and arm_mask is not None:
        arm_mask_uint8 = (arm_mask * 255).astype(np.uint8)
        distance_map = cv2.distanceTransform(255 - arm_mask_uint8, cv2.DIST_L2, 5)
        hand_coords = np.column_stack(np.where(hand_mask > 0))
        min_distance = np.min(distance_map[hand_coords[:, 0], hand_coords[:, 1]]) if len(hand_coords) > 0 else float('inf')
        if min_distance < WARNING_DISTANCE:
            warning_state = True
            cv2.putText(frame, ":경고: Warning!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            if not audio_playing:
                audio_playing = True
                threading.Thread(target=play_warning_audio).start()
    robot_monitor.send_warning(warning_state)
    # :불: Pit 상태 확인 및 문자열 변환 처리
    pit_status = {}
    for i, pit_mask in enumerate(pit_masks):
        if pit_mask is None:
            print(f":경고: Pit {i+1}의 mask 감지되지 않음 -> 캡슐이 있음(O)")
            pit_status[f"Pit {i+1}"] = False  # 캡슐이 있으므로 X
        else:
            print(f":흰색_확인_표시: Pit {i+1} mask detected, shape: {pit_mask.shape}")
            print(f":단안경을_쓴_얼굴: Pit {i+1} mask unique values: {np.unique(pit_mask)}")
            pit_status[f"Pit {i+1}"] = True  # Pit이 비어있으므로 O
        cv2.putText(frame, f"Pit {i+1}: {'X' if pit_status[f'Pit {i+1}'] else 'O'}",
                    (50, 100 + i * 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    if robot_monitor.send_pit_status_flag:
        robot_monitor.send_pit_status(pit_status)
    rclpy.spin_once(robot_monitor, timeout_sec=0.01)
    cv2.imshow("Segmented Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
rclpy.shutdown()
