import rclpy as rp
from rclpy.node import Node

import socket
import re

from my_first_pkg_msgs.msg import AppOrder 

class OrderPublisher(Node):
    
    def __init__(self):
        super().__init__('order')
        self.host = "192.168.0.61"  # 서버 IP (서버가 실행되는 PC의 IP 주소)
        self.port = 6789  # 클라이언트가 사용하는 포트 번호
        self.publisher = self.create_publisher(AppOrder, '/app_order', 10)
        self.order = AppOrder()
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))  # 서버 IP와 포트 번호 바인딩
        self.server_socket.listen(1)  # 최대 1개의 연결을 대기 (필요에 따라 변경 가능)
        print("서버가 시작되었습니다. 클라이언트를 기다립니다...")
        self.client_socket=None
        self.client_address=None
        self.pattern = r"Topping:\s*([^ ]+).*Cup/Cone:\s*([^ ]+)"

    def receive_and_publish_order(self):
        while True:
            try:
                # 클라이언트 연결 수락
                self.client_socket, self.client_address = self.server_socket.accept()
                print(f"클라이언트 {self.client_address} 가 연결되었습니다.")

                while True:
                    # 클라이언트로부터 메시지 수신 (최대 1024 바이트)
                    data = self.client_socket.recv(1024)
                    if data:
                        message = data.decode()  # 수신된 데이터를 문자열로 변환
                        print(f"받은 메시지: {message}")
                        match = re.search(self.pattern, message)                 
                        self.order.topping_type=match.group(1)
                        self.order.cup_or_cone=match.group(2)
                        self.publisher.publish(self.order)
                    else:
                        # 클라이언트가 연결을 끊은 경우
                        print("클라이언트가 연결을 종료했습니다.")
                        break
            except KeyboardInterrupt:
                print("\nServer interrupted by user. Closing server socket...")
                # 클라이언트 연결 종료
                self.client_socket.close()
                # 서버 종료 시 소켓 닫기
                self.server_socket.close()
                print("서버 종료")
            except Exception as e:
                print(f"에러 발생: {e}")
                break
        # 클라이언트 연결 종료
        self.client_socket.close()
        # 서버 종료 시 소켓 닫기
        self.server_socket.close()
        print("서버 종료")

def main(args=None):
    rp.init(args=args)
    
    order_node = OrderPublisher()
    order_node.receive_and_publish_order()
    
    order_node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
