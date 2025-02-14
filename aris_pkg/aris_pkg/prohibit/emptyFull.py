import cv2
import torch
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from my_first_pkg_msgs.srv import CapsulePosition  # 서비스 메시지

# 모델 로드 (YOLOv11n)
model_path = "/home/addinedu/dev/CONE/best.pt"
model = YOLO(model_path)

class Cone_Status(Node):
    def __init__(self):
        super().__init__('cone_status_monitor')

        # 서비스 서버 생성 (서비스 요청을 직접 처리)
        self.srv = self.create_service(CapsulePosition, '/process_cone', self.process_order_callback)

        # 경고 상태 및 퍼블리셔 초기화
        self.cone_state = False
        # self.warning_publisher = self.create_publisher(Bool, '/cone_warning', 10)

    def process_order_callback(self, request, response):
        self.get_logger().info(f"📡 Server requested cone position. Request: {request.req}")
        
        # 현재 cone 상태를 응답에 포함
        response.success = bool(self.cone_state)
        response.message = str(self.cone_state)  # Bool 대신 String으로 일관성 유지

        self.get_logger().info(f"📡 Cone bit status: {self.cone_state}")
        return response

    # def send_cone_status(self):
    #     msg = Bool()
    #     msg.data = self.cone_state
    #     self.warning_publisher.publish(msg)
    #     self.get_logger().info(f"⚠ Warning status sent: {self.cone_state}")

    def process_images(self, cap):
        CONFIDENCE_THRESHOLD = 0.7  # 신뢰도 임계값, names: {0: 'empty', 1: 'full'}

        ret, frame = cap.read()
        if not ret:
            return None

        # 모델 추론
        results = model(frame)

        # 결과 시각화
        for result in results:
            for box in result.boxes:
                confidence = box.conf[0].item()

                # 컨피던스가 0.7 이상인 경우만 표시
                if confidence >= CONFIDENCE_THRESHOLD:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    label = result.names[int(box.cls[0])]
                    self.cone_state = int(box.cls[0])  # empty=0, full=1

                    # 색상 설정 (full=초록, empty=빨강)
                    color = (0, 255, 0) if label == "full" else (0, 0, 255)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, f"{label}: {confidence:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return frame  # 프레임 반환

def main():
    rclpy.init()
    cone_process = Cone_Status()
    cap = cv2.VideoCapture(2)

    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        exit()

    print("웹캠 실행 중... 종료하려면 'q'를 누르세요.")

    while rclpy.ok():
        frame = cone_process.process_images(cap)
        if frame is None:
            print("프레임을 읽을 수 없습니다. 종료합니다.")
            break

        cv2.imshow("Cone Detection", frame)
        rclpy.spin_once(cone_process, timeout_sec=0.01)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
