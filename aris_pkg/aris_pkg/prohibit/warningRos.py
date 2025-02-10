import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from ultralytics import YOLO
from gtts import gTTS
import os
import threading
import time
from collections import Counter
from my_first_pkg_msgs.srv import CapsulePosition  # 서비스 메시지

class RobotMonitor(Node): #로스 노드정의 
    def __init__(self): 
        super().__init__('robot_monitor')
        
        # 서비스 서버 생성 (서비스 요청을 직접 처리)
        self.srv = self.create_service(CapsulePosition, '/process_order', self.process_order_callback)

        self.warning_publisher = self.create_publisher(Bool, '/warning', 10)
        
        #경고 상태 관리 변수 
        self.start_time = time.time()
        self.send_pit_status_flag = True
        self.warning_state = False
        self.warning_time = None
        self.pit_status_history = []
        self.processing_time = 10
        self.pit_code=0
        self.pit_status = {
            "Pit 1": False,
            "Pit 2": False,
            "Pit 3": False
        }
    
    def process_pit_status(self, pit_status):
    
        self.pit_status_history.append(pit_status)  # 히스토리 저장
        self.send_pit_status(pit_status)  # 상태 전송


    def process_order_callback(self, request, response):
        
        self.get_logger().info("📡 Server requested capsule position. Request: {request.req}")
        if request.req:
            # 요청이 들어오면 즉시 캡슐 상태 감지
            #pit_code = self.process_and_send_pit_status()
            # response.message = f"Capsule status processed: {pit_code}"
            response.success = True
            
            response.message = self.pit_code
        else:
            response.success = False
            response.message = "Invalid request"
        return response
    
    #여기서 로그까지는 되는데 자꾸 process_and_send_pit_status()가 실행이 안되는 것 같아.


    def send_warning(self): #캡슐 구멍 상태 전송 함수 
        msg = Bool()
        msg.data = self.warning_state
        self.warning_publisher.publish(msg)
        self.get_logger().info(f"⚠ Warning status sent: {self.warning_state}")

    def send_pit_status(self, pit_status): #pit 상태 전송 함수 
        if not self.send_pit_status_flag:
            return
        self.pit_code = "".join(['1' if pit_status[f"Pit {i+1}"] else '0' for i in range(3)])
        #msg = String()
        #msg.data = pit_code
        #self.pit_status_pub.publish(msg)
        self.get_logger().info(f"📡 Pit status sent: {self.pit_code}")

    
    
    def get_pit_mask(self, pit_index):
        """ 각 Pit에 대한 마스크를 반환하는 함수 """
        # 여기서는 각 Pit에 해당하는 마스크를 YOLO 모델의 예측 결과에서 찾는 로직을 작성해야 해.
        # pit_index가 0, 1, 2일 때마다 Pit1, Pit2, Pit3에 대한 마스크를 반환하는 예시를 들면:

        # 예시: pit_index에 따라 Pit 마스크 처리 로직
        if pit_index == 0:
            # Pit 1에 대한 마스크 처리
            return None  # 또는 실제 마스크 반환
        elif pit_index == 1:
            # Pit 2에 대한 마스크 처리
            return None  # 또는 실제 마스크 반환
        elif pit_index == 2:
            # Pit 3에 대한 마스크 처리
            return None  # 또는 실제 마스크 반환
        else:
            return None


    def process_and_send_pit_status(self):
            """ Pit 상태 감지 및 비트 코드 생성 """
            pit_masks = [None, None, None]  
            self.pit_status = {}


            #self.get_logger().info("🚀 process_and_send_pit_status() 실행됨!")  # ✅ 실행 체크



            for i in range(3):  
                #self.get_logger().info("🚀 process_and_send_pit_status() 실행됨!")
                pit_masks[i] = self.get_pit_mask(i)

            for i, pit_mask in enumerate(pit_masks):
                if pit_mask is None:
                    #self.get_logger().warning(f"⚠ Pit {i+1}의 mask 감지되지 않음 -> 캡슐이 있음(O)")
                    self.pit_status[f"Pit {i+1}"] = False  
                else:
                    #self.get_logger().info(f"✅ Pit {i+1} mask detected")
                    self.pit_status[f"Pit {i+1}"] = True  

            # 감지된 Pit 상태를 비트 문자열로 변환
            pit_code = "".join(['1' if self.pit_status[f"Pit {i+1}"] else '0' for i in range(3)])
            #self.get_logger().info(f"📤 Processed pit status: {pit_code}")
            self.get_logger().info(f"📤 Processed pit status: {pit_code}")
            #print(f"Processed pit status: {pit_code}")
    
    # 비트 코드 전송
            self.send_pit_status({f"Pit {i+1}": self.pit_status[f"Pit {i+1}"] for i in range(3)})
            return pit_code


class YOLO_Processor:
    def __init__(self, model_path): 
        self.model = YOLO(model_path) #욜로 모델 로드 
        self.confidence_threshold = 0.7 #신뢰도 임계값 설정 
        self.hand_color, self.arm_color = (0, 255, 0), (0, 0, 255)
        self.pit_colors = [(255, 0, 0), (255, 255, 0), (0, 255, 255)]
        self.WARNING_DISTANCE = 10  #경고 거리 기준
        self.audio_playing = False
        self.true_count = 0 
        self.TARGET_COUNT = 5

    def play_warning_audio(self):
        global audio_playing
        text = "경고! 위험하오니 뒤로 한 걸음 물러나 주세요."
        tts = gTTS(text=text, lang='ko', slow=False)
        tts.save("warning.mp3")
        os.system("mpg321 warning.mp3")
        time.sleep(4)
        self.audio_playing = False

    def process_frame(self, frame, robot_monitor): #욜로 모델 사용하여 프레임 처리하고 경고 트리거 하기 
        results = self.model(frame)
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
                    frame[mask == 1] = self.hand_color
                    hand_mask = mask
                elif class_name == 'robot':
                    frame[mask == 1] = self.arm_color
                    arm_mask = mask
                elif class_name.startswith('pit_'):
                    pit_idx = int(class_name[-1]) - 1
                    frame[mask == 1] = self.pit_colors[pit_idx]
                    pit_masks[pit_idx] = mask

        if hand_mask is not None and arm_mask is not None: #손이랑 로봇팔 간 거리 계산 
            arm_mask_uint8 = (arm_mask * 255).astype(np.uint8)
            distance_map = cv2.distanceTransform(255 - arm_mask_uint8, cv2.DIST_L2, 5)
            hand_coords = np.column_stack(np.where(hand_mask > 0))
            min_distance = np.min(distance_map[hand_coords[:, 0], hand_coords[:, 1]]) if len(hand_coords) > 0 else float('inf')

            if min_distance < self.WARNING_DISTANCE: #경고 조건 확인 하기
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

        pit_status = {}
        for i, pit_mask in enumerate(pit_masks):
            if pit_mask is None:
                pit_status[f"Pit {i+1}"] = False
            else:
                pit_status[f"Pit {i+1}"] = True
            cv2.putText(frame, f"Pit {i+1}: {'X' if pit_status[f'Pit {i+1}'] else 'O'}", 
                        (50, 100 + i * 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        robot_monitor.process_pit_status(pit_status)
        robot_monitor.send_warning()
        return frame

def main(): #이런 클래스 내부를 말하는 거겠지?
    rclpy.init()
    robot_monitor = RobotMonitor()
    yolo_processor = YOLO_Processor("/home/addinedu/dev_ws/src/aris-repo-4/aris_pkg/aris_pkg/prohibit/best.pt")

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