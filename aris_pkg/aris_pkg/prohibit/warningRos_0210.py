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

class RobotMonitor(Node): #ROS 노드 정의 (RobotMonitor)
    def __init__(self):
        super().__init__('robot_monitor')

        #ROS2 퍼블리셔 생성: pit 상태와 경고 상태를 전송할 퍼블리셔임
        self.pit_status_pub = self.create_publisher(String, '/pit_status', 10)
        self.warning_publisher = self.create_publisher(Bool, '/warning', 10)

        #경고 상태를 관리하는 변수들
        self.start_time = time.time()
        self.send_pit_status_flag = True  
        self.warning_state = False  #경고 상태를 처음에 False로 초기화하기
        self.warning_time = None  #경고 상태가 시작된 시간을 저장할 변수 추가
        self.pit_status_history = []  #20초 동안의 pit 상태를 기록하는 리스트
        self.processing_time = 10  


    #pit(캡슐 구멍) 상태를 전송하는 함수 
    def send_warning(self):
        msg = Bool()
        msg.data = self.warning_state
        self.warning_publisher.publish(msg)
        self.get_logger().info(f"⚠ Warning status sent: {self.warning_state}")


    #pit 상태를 전송해주는 함수
    def send_pit_status(self, pit_status): #pit status는 'Pit 1': 0, 'Pit 2': X 형식으로 들어옴 
        if not self.send_pit_status_flag:  
            return #상태 전송이 비활성화 되면 함수 종료 

        # pit_status는 'Pit 1': O, 'Pit 2': X 와 같은 방식으로 들어옴
        #pit_status = {'Pit 1': True, 'Pit 2': False, 'Pit 3': True}
        pit_code = "".join(['1' if pit_status[f"Pit {i+1}"] else '0' for i in range(3)])
        #if pit 1이 TRUE이면 pit_status['Pit 1']-> True -> '1' 이런식
        #그럼 ['1', '1', '1'] 이런식으로 나오게 되고 
        #이걸 join으로 하나의 문자열로 합쳐서 111 이렇게 됨 
        
        msg = String() #ROS2 string 메세지 객체생성
        msg.data = pit_code   #msg에 이진 문자열 저장 
        self.pit_status_pub.publish(msg) #ROS2 토픽으로 메세지 전송 
        self.get_logger().info(f"📡 Pit status sent: {pit_code}")


    #20초 동안 pit 상태를 기록하고 가장 자주 나온 상태를 전송하는 함수
    def process_pit_status(self, pit_status):

        #10초가 지나면 가장 많이 나온 pit 상태를 하나만 전송
        if self.start_time is not None: #타이머 시작된 후에만 실행 
            current_time = time.time() #time.time()은 현재 시간 
            if current_time - self.start_time <= self.processing_time:
                #현재시간 - 시작시간 <= 10 면 계속 기록 

                # 10초 내에는 pit 상태를 계속 기록 (이진 문자열로 기록)
                pit_code = "".join(['1' if pit_status[f"Pit {i+1}"] else '0' for i in range(3)])
                self.pit_status_history.append(pit_code)
                #pit_status를 이진 문자열 형태로 저장(binary string)
                #Pit 비어있으면 1, 캡슐 있으면 0
                #"Pit 1": X, "Pit 2": O, "Pit 3": X → "101"
                #예시, self.pit_status_history = ["101", "101", "000", "101", "000", "101"]


            else:
                # 10초가 지나면 가장 많이 나온 pit 상태를 전송
                if self.pit_status_history:
                    most_common_status = Counter(self.pit_status_history).most_common(1)[0][0]
                    #most_common(1)은 가장 많이 등장한 요소 1개 반환 
                    #[(‘101’, 4)] 이런식으로 반환 

                    
                    self.send_pit_status({f"Pit {i+1}": bool(int(most_common_status[i])) for i in range(3)})
                    #101을 다시 pit 딕셔너리 형태로 변환 
                    #101" → {"Pit 1": True, "Pit 2": False, "Pit 3": True}
                    #send pit status로 ROS2 메시지 전송 
                self.send_pit_status_flag = False  # 전송 완료 후 더 이상 상태를 전송하지 않음
#지정된 초 동안 pit_status(O/X) 상태 기록













#YOLO 모델 로드
model_path = "/home/addinedu/dev_ws/src/aris-repo-4/aris_pkg/aris_pkg/prohibit/best.pt"
session = YOLO(model_path)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

print("웹캠 실행 중... 종료하려면 'q'를 누르세요.")

#YOLO 추론 시, 사용되는 변수 설정하기 
confidence_threshold = 0.7
hand_color, arm_color = (0, 255, 0), (0, 0, 255)
pit_colors = [(255, 0, 0), (255, 255, 0), (0, 255, 255)]
WARNING_DISTANCE = 10 #경고 거리 
audio_playing = False #오디오 재생 여부 추적 변수 
 
#카운트를 위한 변수
true_count = 0
TARGET_COUNT = 5 
#경고 발송하기 위한 카운트 변수 


#경고 음성 재생 함수 
def play_warning_audio():
    global audio_playing
    text = "경고! 위험하오니 뒤로 한 걸음 물러나 주세요."
    tts = gTTS(text=text, lang='ko', slow=False)
    tts.save("/home/addinedu/dev_ws/src/aris-repo-4/aris_pkg/aris_pkg/prohibit/warning.mp3") #경고 음성파일 저장하기 
    os.system("mpg321 /home/addinedu/dev_ws/src/aris-repo-4/aris_pkg/aris_pkg/prohibit/warning.mp3") #mpg321로 음성 재생하기 
    time.sleep(4)
    audio_playing = False #오디오 재생 후 상태 변경 


#ROS2 초기화 
rclpy.init()
robot_monitor = RobotMonitor() #클래스 인스턴스 생성 




while True:
    ret, frame = cap.read() #ret= T or F (성공적으로 읽냐 마냐), frame= 읽어온 프레임의 이미지 데이터
    if not ret: #(카메라 연결 문제, 웹캠 작동 문제, 다른 이유로 실패할 경우)
        print("프레임을 읽을 수 없습니다. 종료합니다.")
        break #즉, 웹캠으로부터 이미지 데이터를 받아올 수 없다는 상태를 의미함 


    results = session(frame) #YOLO 모델을 사용해서 추론 수행하기 
    result = results[0] #추론 결과 중 첫 번째 이미지를 처리한 결과를 의미함
    #results[0]은 첫번째 프레임에 대한 추론 결과 객체이고 이 안에 masks, name 등이 들어있는 것

    #즉, 반복문이기 때문에 매 반복마다 각각의 새로운 프레임에 대한 결과를 가져오는 것임

#results는 frame에 대한 추론을 수행한 결과를 담고 있는 객체라고 생각하면 됨
#즉, 이 객체는 모델이 추론한 여러 정보(객체탐지, 분할, 키포인트 추적 등)를 포함
#세그멘테이션 마스크, 각 객체 클래스 이름, 컨피던스(모델 신뢰도) 등 포함






    masks = result.masks #마스크 데이터 추출하기
    hand_mask, arm_mask = None, None #손, 로봇팔 마스크 초기화하기 
    pit_masks = [None, None, None] #PiT 영역 마스크 초기화 하기 


    #YOLO 모델에서 감지된 마스크 처리하는 부분
    if masks is not None:
        mask_data = masks.data.cpu().numpy()
        #enumerate(mask_data)는 mask_data 배열을 순회하면서 **인덱스(idx)와 해당 값(mask)**을 반환하는 함수
        for idx, mask in enumerate(mask_data):
            class_name = result.names[int(result.boxes[idx].cls)] #감지된 객체의 클래스명임
            #현재 바운딩 박스 클래스의 인덱스를 가져와서  이에 해당하는 클래스 이름을 
            #resul.names에서 찾아서 반환함 

#int 변환이유
    #->result.boxes[idx].cls는 그 바운딩 박스에 대한 클래스 인덱스를 나타냄
    #예를 들어 cls가 0이면 person 클래스 이런 식임 
    #즉, 현재 인덱스(idx)의 물체에 대한 클래스 인덱스를 나타냄 
    #그런데 result.boxes[idx].cls는 YOLO 추론 결과가 float 형태일 수도 있기 때문 
    #그래서 int형으로 변환해서 1.0->1 이런식으로 정확하게 클래스를 찾기 위함임 

    #예시는 옵시디언에 있음 

            mask_img = (mask * 255).astype(np.uint8)
            #마스크는 보통 배경 0~1  사이값을 가짐(배경=0, 물체=1)
            #즉, mask 배열의 값은 각 픽셀이 물체인지 아닌지를 나타내는 이진 배열임 
#그러나 OpenCV에서 이미지 표시할때는 0~255 값을  사용하기 때문에 이값을 255로 확장하여 픽셀값을 맞춰주는 것임
#astype(np.unit8)은 unit8-8비트 부호 없는 정수 형식으로 변환하여 opencv가 처리할 수 있는 형태로 만듦
            if class_name == 'hand':
                frame[mask == 1] = hand_color #세그멘테이션 마스크 값이 1이면 손
                #즉, 손이 있는 부분의 픽셀만 선택해서 색깔입히기 
                hand_mask = mask
            elif class_name == 'robot':
                frame[mask == 1] = arm_color
                arm_mask = mask
            elif class_name.startswith('pit_'):
                pit_idx = int(class_name[-1]) - 1
                frame[mask == 1] = pit_colors[pit_idx]
                pit_masks[pit_idx] = mask


#경고 카운트 늘려보기랑, 핸드마스크 0에서 키워보기 

    # 경고 상태 초기화 및 거리 계산
    if hand_mask is not None and arm_mask is not None: #손 및 로봇 마스크가 존재하는지 확인, 둘중하나라도 none이면 아래 실행 안함  
        arm_mask_uint8 = (arm_mask * 255).astype(np.uint8)
        distance_map = cv2.distanceTransform(255 - arm_mask_uint8, cv2.DIST_L2, 5)

        hand_coords = np.column_stack(np.where(hand_mask > 0))
        min_distance = np.min(distance_map[hand_coords[:, 0], hand_coords[:, 1]]) if len(hand_coords) > 0 else float('inf')

        if min_distance < WARNING_DISTANCE:
            true_count += 1  # True 상태일 때 카운트 증가
            #목표 카운트에 도달하면 경고 발송
            if not robot_monitor.warning_state and true_count >= TARGET_COUNT:  # 카운트가 목표에 도달하면 경고 발송
                robot_monitor.warning_state = True
                robot_monitor.warning_time = time.time()  # 경고 시작 시간을 기록
                cv2.putText(frame, "⚠ Warning!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                if not audio_playing: #음성 재생 중이 아니면 음성 재생 
                    audio_playing = True
                    threading.Thread(target=play_warning_audio).start()

        else: #거리가 만약 경고 범위를 벗어나면 경고상태 해제
            if robot_monitor.warning_state:
                true_count = 0  # 조건을 벗어나면 카운트 초기화
                robot_monitor.warning_state = False
                cv2.putText(frame, "⚠ Warning Cleared!", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)


    #Pit 상태 확인 및 비트 문자열로 변환 처리
    pit_status = {} #pit 상태 저장할 딕셔너리 생성 
    #{"Pit 1": True, "Pit 2": False, "Pit 3": True} 이런 식으로 저장됨

    for i, pit_mask in enumerate(pit_masks): #i는 피트 번호, pit_mask는 해당 pit 마스크
        if pit_mask is None:#pit mask가 없다면 = 캡슐이 있음을 의미 (캡슐이 pitmask를 가리니까)
            print(f"⚠ Pit {i+1}의 mask 감지되지 않음 -> 캡슐이 있음(O)")
            pit_status[f"Pit {i+1}"] = False  # 캡슐이 있으므로=O
        else:
            print(f"✅ Pit {i+1} mask detected, shape: {pit_mask.shape}")
            print(f"🧐 Pit {i+1} mask unique values: {np.unique(pit_mask)}")
            #유니크는 마스크가 어떤 값을 가지고 있는지 확인 
            pit_status[f"Pit {i+1}"] = True  # Pit이 비어있으므로(캡슐이 없으므로)=X

        # 비트 문자열로 출력
        cv2.putText(frame, f"Pit {i+1}: {'X' if pit_status[f'Pit {i+1}'] else 'O'}", 
                    (50, 100 + i * 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    robot_monitor.process_pit_status(pit_status)  # 지정한 초 동안 pit 상태 변화 저장 
    #이후 최빈값을 ROS2로 전송 

    #손과 팔 가까우면 경고 메세지 보내기
    robot_monitor.send_warning()  # 경고 상태 보내기

    rclpy.spin_once(robot_monitor, timeout_sec=0.01)#ROS2 노드를 실행하기 위해 한 번만 이벤트 처리

    cv2.imshow("Segmented Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break#1ms 키입력 대시, q 누르면 while 루프 종료 

cap.release() #웹캠 사용종료 
cv2.destroyAllWindows() #opencv창 닫기 
rclpy.shutdown() #ros2 노드 종료 

#이게 지금 합친 코드