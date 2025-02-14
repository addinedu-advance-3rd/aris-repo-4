import cv2
import mediapipe as mp

# MediaPipe Hands 초기화
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_drawing = mp.solutions.drawing_utils

# 카메라 캡처 시작
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # 손가락 팁과 MCP 관절 인덱스 정의
            finger_tips = [
                #mp_hands.HandLandmark.THUMB_TIP,
                mp_hands.HandLandmark.INDEX_FINGER_TIP,
                mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                mp_hands.HandLandmark.RING_FINGER_TIP,
                mp_hands.HandLandmark.PINKY_TIP
            ]
            
            finger_mcp = [
                #mp_hands.HandLandmark.THUMB_CMC,  # 엄지는 CMC 관절을 기준
                mp_hands.HandLandmark.INDEX_FINGER_MCP,  # 나머지 손가락은 MCP 관절 사용
                mp_hands.HandLandmark.MIDDLE_FINGER_MCP,
                mp_hands.HandLandmark.RING_FINGER_MCP,
                mp_hands.HandLandmark.PINKY_MCP
            ]
            
            # 주먹 감지 로직: 손가락 끝이 MCP보다 아래로 향해 있는지 확인
            is_fist = True
            for tip, mcp in zip(finger_tips, finger_mcp):
                tip_y = hand_landmarks.landmark[tip].y
                mcp_y = hand_landmarks.landmark[mcp].y
                
                if tip_y < mcp_y:  # 손가락이 접히지 않은 경우
                    is_fist = False
                    break
            
            # 주먹을 쥔 상태라면 'Stop' 라벨 표시
            if is_fist:
                cv2.putText(image, 'Stop', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3)
            else:
                # 엄지 (Thumb)와 약지 (Ring Finger)의 X 좌표 추출
                thumb_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x
                ring_x = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].x
                
                # 왼손 기준으로 엄지가 약지보다 오른쪽에 있으면 손바닥, 왼쪽이면 손등
                if thumb_x > ring_x:
                    cv2.putText(image, 'Palm', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    cv2.putText(image, 'Back of Hand', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
    cv2.imshow('Hand Detection', image)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
