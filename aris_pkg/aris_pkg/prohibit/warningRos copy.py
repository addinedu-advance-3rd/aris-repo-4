#!/usr/bin/env python3

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
from my_first_pkg_msgs.srv import CapsulePosition  # 서비스 메시지

class RobotMonitor(Node):
    def __init__(self):
        super().__init__('robot_monitor')
        
        # 서비스 서버 생성 (서비스 요청을 직접 처리)
        self.srv = self.create_service(CapsulePosition, '/process_order', self.process_order_callback)

        # 경고 메시지 퍼블리셔 생성
        self.warning_publisher = self.create_publisher(Bool, '/warning', 10)

        self.warning_state = False  
        self.audio_playing = False
        self.pit_status = {}  

    def process_order_callback(self, request, response):
        self.get_logger().info("📡 Server requested capsule position.")
        if request.req is True:
            # 요청이 들어오면 즉시 캡슐 상태 감지
            pit_code = self.process_and_send_pit_status()
            # response.message = f"Capsule status processed: {pit_code}"
            response.success = request.req
            response.message = pit_code
        return response

    def send_warning(self):
        """ 경고 메시지 퍼블리싱 """
        msg = Bool()
        msg.data = self.warning_state
        self.warning_publisher.publish(msg)
        self.get_logger().info(f"⚠ Warning status sent: {self.warning_state}")

    def process_and_send_pit_status(self):
        """ Pit 상태 감지 및 비트 코드 생성 """
        pit_masks = [None, None, None]  

        for i in range(3):  
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
        return pit_code

    def get_pit_mask(self, pit_index):
        """ YOLO를 이용하여 특정 pit 영역의 마스크를 가져옴 """
        ret, frame = cap.read()  
        if not ret:
            self.get_logger().error("❌ 프레임을 읽을 수 없습니다.")
            return None

        results = session(frame)
        result = results[0]
        masks = result.masks

        if masks is not None:
            mask_data = masks.data.cpu().numpy()
            for idx, mask in enumerate(mask_data):
                class_name = result.names[int(result.boxes[idx].cls)]
                if class_name == f'pit_{pit_index+1}':
                    return mask
        return None

    def play_warning_audio(self):
        """ 경고 오디오 재생 """
        text = "경고! 위험하오니 뒤로 한 걸음 물러나 주세요."
        tts = gTTS(text=text, lang='ko', slow=False)
        tts.save("warning.mp3")
        os.system("mpg321 warning.mp3")
        time.sleep(4)
        self.audio_playing = False


# YOLO 모델 로드
model_path = "/home/addinedu/dev_ws/src/aris-repo-4/aris_pkg/aris_pkg/prohibit/best.pt"
session = YOLO(model_path)

# 웹캠 초기화
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

# ROS 2 노드 초기화
rclpy.init()
robot_monitor = RobotMonitor()

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다. 종료합니다.")
        break

    # YOLO를 통한 객체 감지
    results = session(frame)
    result = results[0]
    masks = result.masks

    # Segmentation 결과 표시
    if masks is not None:
        mask_data = masks.data.cpu().numpy()
        
        for idx, mask in enumerate(mask_data):
            class_id = int(result.boxes[idx].cls)
            class_name = result.names[class_id]
            confidence = result.boxes[idx].conf[0].item()

            # 마스크 적용
            mask = (mask * 255).astype(np.uint8)
            colored_mask = np.zeros_like(frame)
            colored_mask[mask == 255] = (0, 255, 0)  # 초록색 마스크

            # 원본 이미지와 합성
            frame = cv2.addWeighted(frame, 1, colored_mask, 0.5, 0)

            # 클래스 이름과 신뢰도 표시
            label = f"{class_name}: {confidence:.2f}"
            cv2.putText(frame, label, (10, 30 + idx * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # 경고 상태 전송
    robot_monitor.send_warning()

    # ROS 2 콜백 함수 실행
    rclpy.spin_once(robot_monitor, timeout_sec=0.01)

    # 결과 이미지 표시
    cv2.imshow("YOLO Segmentation", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()
rclpy.shutdown()
