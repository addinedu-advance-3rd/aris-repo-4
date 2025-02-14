#!/usr/bin/env python3
import os
import threading
import urllib.request
import urllib.parse
import subprocess  # for checking if mpg321 is installed
from playsound import playsound

import speech_recognition as sr
import openai

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from my_first_pkg_msgs.msg import AppOrder

from dotenv import load_dotenv

# =============== 추가: Vosk 관련 import ===============
import json
import pyaudio
from vosk import Model, KaldiRecognizer

###############################
# 상태 정의
###############################
STATE_IDLE = 0
STATE_WAITING_CONFIRM = 1

###############################################
# 1️⃣ .env에서 환경 변수 로드 (NAVER TTS & OpenAI)
###############################################

env_path = "/home/addinedu/mimo_ws/src/aris-repo-4/aris_pkg/aris_pkg/gpt_api/.env"

load_dotenv(dotenv_path=env_path)

NAVER_CLIENT_ID = os.getenv("NAVER_CLIENT_ID", "").strip()
NAVER_CLIENT_SECRET = os.getenv("NAVER_CLIENT_SECRET", "").strip()
openai.api_key = os.getenv("OPENAI_API_KEY", "").strip()

if not NAVER_CLIENT_ID or not NAVER_CLIENT_SECRET:
    print("[❌ TTS ERROR] NAVER_CLIENT_ID 또는 NAVER_CLIENT_SECRET이 설정되지 않았습니다.")
    exit(1)

if not openai.api_key:
    print("[❌ OPENAI ERROR] OPENAI_API_KEY가 설정되지 않았습니다.")
    exit(1)

###############################################
# 2️⃣ 네이버 TTS를 활용한 speak 함수 (콜백 기능 포함)
###############################################
def speak(text, callback=None):
    """
    네이버 클라우드 음성 합성 API를 이용해 text를 mp3로 생성 후 재생.
    - mpg321 설치 권장 (playsound는 백업용)
    - callback: TTS가 끝난 뒤 실행할 함수 (스레드로 실행)
    """
    if not text or not isinstance(text, str):
        print("[TTS ERROR] speak()에 잘못된 text가 들어왔습니다.")
        return

    try:
        enc_text = urllib.parse.quote(text)
        data = f"speaker=nara&volume=0&speed=0&pitch=0&format=mp3&text={enc_text}"
        url = "https://naveropenapi.apigw.ntruss.com/tts-premium/v1/tts"


        request = urllib.request.Request(url)
        request.add_header("X-NCP-APIGW-API-KEY-ID", NAVER_CLIENT_ID)
        request.add_header("X-NCP-APIGW-API-KEY", NAVER_CLIENT_SECRET)

        response = urllib.request.urlopen(request, data=data.encode("utf-8"))
        rescode = response.getcode()

        if rescode == 200:
            response_body = response.read()


            file_path = "/tmp/temp_tts.mp3"

            with open(file_path, 'wb') as f:
                f.write(response_body)

            print("[🔊 TTS 재생 시작]")
            ret = subprocess.call(["which", "mpg321"], stdout=subprocess.DEVNULL)
            if ret == 0:
                # mpg321 존재
                os.system(f"mpg321 {file_path}")
            else:
                # mpg321 없으면 playsound 사용
                playsound(file_path)

            os.remove(file_path)
            print("[🔊 TTS 완료]")

            # TTS 종료 후 콜백
            if callback:
                threading.Thread(target=callback, daemon=True).start()
        else:
            print(f"[TTS ERROR] HTTP 응답 코드: {rescode}")
    except Exception as e:
        print(f"[TTS ERROR] {e}")

################################################
# 3️⃣ GPT API로 아이스크림 추천
################################################
def get_ice_cream_recommendation(user_input):
    """
    사용자 취향(맛, 기분 등)을 받아 GPT-3.5로부터
    Flavor, Toppings(또는 None), Cup_or_Cone 을 추천받음.
    """
    try:
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {
                    "role": "system",
                    "content": (


            "You are an employee at an ice cream shop.\n\n"

                        "[Mission]\n"
                        "- Recommend the ice cream flavor and topping that best match the customer's preferences.\n\n"
                        "[Context]\n"
                        "- When a customer speaks via voice input, they might use expressions that differ slightly from the actual menu names (e.g., “스트로베리” might be said instead of “딸기”).\n"
                        "- Similarly, for toppings, a customer might say “초코볼” while the actual menu name is “코코볼.”\n\n"
                        "[Persona]\n"
                        "- You are extremely friendly and strive to accurately reflect the customer's requirements.\n"
                        "- The key is to analyze the customer's input and find the closest matching menu item.\n\n"
                        "[Example]\n"
                        "(1) If a customer says, \"상큼한 맛 좋아해. 바삭한 토핑이랑 먹고 싶어\"\n"
                        "- A possible answer:\n"
                        "Flavor: 딸기\n"
                        "Toppings: 코코볼\n"
                        "Cup_or_Cone: cup\n"
                        "(2) If a customer says, \"초코 토핑 좋아. 블루베리는 어때?\"\n"
                        "- A possible answer:\n\n"
                        "Flavor: 블루베리\n"
                        "Toppings: 코코볼\n"
                        "Cup_or_Cone: cup\n"
                        "[Format]\n"
                        "- You must return exactly three lines, each containing one of the following:\n"
                        "1) Flavor: [either 딸기 or 블루베리]\n"
                        "2) Toppings: [one of 조리퐁, 코코볼, 해바라기씨, None]\n"
                        "3) Cup_or_Cone: [either cup or cone]\n"
                        "- Default values:\n"
                        "- Cup_or_Cone → cup\n"
                        "- Toppings → None (if the customer does not explicitly mention a topping)\n\n"
                        "[Tone]\n"
                        "- Provide the response in a concise, definitive, yet polite and gentle manner.\n\n"
                        "[Additional Rules]\n"
                        "1) If the customer does not mention any topping or uses negative expressions like \"토핑 필요 없어,\" always set Toppings: None.\n"
                        "2) If the customer uses words like \"딸기\" or \"베리,\" match it to Flavor: Strawberry.\n"
                        "3) If the customer uses words like \"blueberry\" or \"블루베리,\" match it to Flavor: 블루베리.\n"
                        "4) For toppings, refer to the following synonyms and choose the closest match (only one):\n"
                        " - 조리퐁 → [조리퐁, 조리뽕, 곡물 토핑, 죠리뽕, 죠리퐁, 안딱딱한 토핑]\n"
                        " - 코코볼 → [코코볼, 초코볼, 초콜릿볼, 초코 토핑, 바삭한 토핑, 쪼코볼]\n"
                        " - 해바라기씨 → [해바라기씨, 해바라기, 씨앗 토핑, sunflower seed]\n"
                        " - None → [no topping, not needed, remove topping, skip]\n"
                        "5) If the customer does not mention \"cone,\" then use Cup_or_Cone: cup as the default.\n"
                        "6) Output example:"
                        "Flavor: 딸기\n"
                        "Toppings: None\n"
                        "Cup_or_Cone: cup\n"
                        "7) If the customer mentions a flavor that is not allowed (e.g., chocolate, sweet potato, etc.), always map it to either '딸기' or '블루베리', and never output any disallowed flavor.\n"
                        "8) If the customer mentions a topping that is not allowed (e.g., sweet potato, chocolate, etc.), always map it to one of '조리퐁', '코코볼', or '해바라기씨', and never output any disallowed topping.\n"
                        "9) If the customer describes the ice cream flavor with expressions such as '새콤달콤하다', '너무 달지 않은', or '덜 달다', always map it to Blueberry flavor.\n"
                    )
                },
                {"role": "user", "content": user_input}
            ],
            max_tokens=200,
            temperature=0.7
        )
        return response["choices"][0]["message"]["content"].strip()
    except Exception as e:
        return f"Error: {e}"

################################################
# 4️⃣ GPT 응답 파싱
################################################
def parse_gpt_recommendation(recommendation_text: str):
    flavor = ""
    toppings = ""
    cup_or_cone = ""

    lines = recommendation_text.split('\n')
    for line in lines:
        line = line.strip()
        if line.lower().startswith("flavor:"):
            flavor = line.split(":", 1)[1].strip()
        elif line.lower().startswith("toppings:"):
            toppings = line.split(":", 1)[1].strip()
        elif line.lower().startswith("cup_or_cone:"):
            cup_or_cone = line.split(":", 1)[1].strip()

    return flavor, toppings, cup_or_cone

################################################
# 5️⃣ 음성 인식 (10초)
################################################
def speech_to_text(callback, phrase_time_limit=10, pause_threshold=0.1, non_speaking_duration=0.1):
    def recognize():
        r = sr.Recognizer()
        r.dynamic_energy_threshold = False
        r.energy_threshold = 300  # 환경에 맞게 적절한 값으로 조정

        r.pause_threshold = pause_threshold
        r.non_speaking_duration = non_speaking_duration
        with sr.Microphone() as source:
            r.adjust_for_ambient_noise(source, duration=1)
            print("🎙️ 음성 인식 중! 최대 {}초간 말하세요.".format(phrase_time_limit))
            audio_data = r.listen(source, phrase_time_limit=phrase_time_limit)

        try:
            print("📝 음성을 텍스트로 변환 중...")
            text = r.recognize_google(audio_data, language="ko-KR")
            print(f"🎙️ 인식된 음성: {text}")
            callback(text)
        except sr.UnknownValueError:
            callback("STT_ERROR")
        except sr.RequestError as e:
            callback(f"STT_ERROR: {e}")

    threading.Thread(target=recognize, daemon=True).start()
# =============== 추가: Vosk (오픈소스)로 짧은 발화 인식 함수 ===============
VOSK_MODEL_PATH = "/home/addinedu/dev_ws/src/aris-repo-4/aris_pkg/aris_pkg/gpt_api/vosk-model-small-ko-0.22"  # <-- 실제 모델 경로로 수정


# =============== 추가: Vosk (오픈소스)로 짧은 발화 인식 함수 ===============
VOSK_MODEL_PATH = "/home/addinedu/Downloads/vosk-model-small-ko-0.22"  # <-- 실제 모델 경로로 수정

try:
    vosk_model = Model(VOSK_MODEL_PATH)
    print("[VOSK] 한국어 모델 로드 완료.")
except Exception as e:
    print("[VOSK ERROR] 모델 로드 실패:", e)
    vosk_model = None

def short_speech_to_text_vosk(callback, record_seconds=2):
    """
    Vosk를 이용해서 약 record_seconds 초 동안 마이크 입력을 받아
    인식된 텍스트를 callback으로 넘겨줍니다.
    (네/아니오처럼 짧은 응답 인식에 활용)
    """
    def recognize():
        if vosk_model is None:
            print("[VOSK ERROR] 모델이 로드되지 않아 STT 수행 불가.")
            callback("STT_ERROR")
            return

        rec = KaldiRecognizer(vosk_model, 16000)

        # PyAudio 설정
        p = pyaudio.PyAudio()
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=8000
        )
        stream.start_stream()

        print(f"🎙️ [Vosk] 짧은 발화 녹음 시작({record_seconds}초)...")
        frames = []
        # 4000 바이트씩 읽어들일 때, 1초에 대략 4번 * record_seconds
        for _ in range(int(16000 / 8000 * record_seconds)):
            data = stream.read(4000, exception_on_overflow=False)
            frames.append(data)

        # 녹음 종료
        print("📝 [Vosk] 음성 인식 중...")
        for frame in frames:
            rec.AcceptWaveform(frame)

        final_result = rec.FinalResult()
        result_json = json.loads(final_result)
        recognized_text = result_json.get("text", "").strip()

        print(f"🎙️ [Vosk] 인식된 음성: {recognized_text}")

        if recognized_text:
            callback(recognized_text)
        else:
            callback("STT_ERROR")

        stream.stop_stream()
        stream.close()
        p.terminate()

    threading.Thread(target=recognize, daemon=True).start()
################################################
# 6️⃣ IceCreamNode
################################################
class IceCreamNode(Node):
    def __init__(self):
        super().__init__('ice_cream_node')

        self.state = STATE_IDLE
        self.rec_flavor = ""
        self.rec_toppings = ""
        self.rec_cup_or_cone = ""

        self.sub_voice = self.create_subscription(
            Bool,
            '/voice_recognize',
            self.voice_callback,
            10
        )
        self.pub_order = self.create_publisher(AppOrder, '/app_order', 10)

        self.get_logger().info("IceCreamNode with Naver TTS + Yes/No confirmation initialized.")

    def voice_callback(self, msg: Bool):
        if msg.data:
            if self.state == STATE_IDLE:
                self.get_logger().info("[STATE_IDLE] → 안내 멘트 후 사용자 취향 음성 인식")
                intro_msg = "안녕하세요 고객님. 아이스크림은 딸기, 블루베리 맛이 있고 토핑은 조리퐁, 코코볼, 해바라기씨가 있습니다. 원하시는 맛과 토핑을 골라주세요."
                speak(intro_msg)                
                # 주문(취향) 단계: 길게 말해도 잘 인식되도록 설정
                speech_to_text(
                    self.on_preference_received,
                    phrase_time_limit=10,        # 긴 시간 허용
                    pause_threshold=0.6,        # 천천히 말해도 끊기지 않도록
                    non_speaking_duration=0.5   # 단어 사이 침묵도 여유 있게
                )
            elif self.state == STATE_WAITING_CONFIRM:
                self.get_logger().info("[STATE_WAITING_CONFIRM] → 네/아니오 음성 인식")
                # 확인 단계: 짧은 응답을 빠르게 인식
                short_speech_to_text_vosk(
                self.on_confirmation_received,
                record_seconds=2
                )
        else:
            self.get_logger().info("음성인식 트리거 OFF.")

    def on_preference_received(self, user_text: str):
        if user_text.startswith("STT_ERROR"):
            speak("다시 말씀해주세요.",
                  callback=lambda: speech_to_text(
                      self.on_preference_received,
                      phrase_time_limit=10,
                      pause_threshold=0.6,
                      non_speaking_duration=0.5
                  ))
            return

        # GPT 추천
        recommendation = get_ice_cream_recommendation(user_text)
        self.get_logger().info(f"GPT 추천:\n{recommendation}")

        # 파싱
        flavor, toppings, cup_or_cone = parse_gpt_recommendation(recommendation)
        self.rec_flavor = flavor
        self.rec_toppings = toppings
        self.rec_cup_or_cone = cup_or_cone


        msg = f"{flavor}+{toppings}+{cup_or_cone} 이대로 주문하시겠습니까?\n" \

              "네라고 하시면 주문이 완료되고, 아니오라고 하시면 다시 주문 하실 수 있습니다."

        self.get_logger().info(f"[TTS MESSAGE] {msg}")
        self.state = STATE_WAITING_CONFIRM

        # 네/아니오 확인 단계 → 짧고 빠른 인식
        speak(msg, callback=lambda:
        short_speech_to_text_vosk(
        self.on_confirmation_received,
        record_seconds=2
        ))

    def on_confirmation_received(self, confirm_text: str):
        if confirm_text.startswith("STT_ERROR"):
            speak("네 또는 아니오라고 말씀해주세요.",
                  callback=lambda: short_speech_to_text_vosk(
                  self.on_confirmation_received,
                  record_seconds=2
                  ))
            return

        text_lower = confirm_text.lower().strip()

        # "네" → 주문 완료 후 프로그램 종료
        if any(x in text_lower for x in ["네", "예", "yes", "응", "어","넷", "넵","옙", "그래", "네에", "내", "엥", "넹", "에", "애", "네엡", "넴"]):

            speak("주문이 완료되었습니다.")
            # AppOrder 발행
            order_msg = AppOrder()
            order_msg.topping_type = self.rec_toppings
            order_msg.cup_or_cone = self.rec_cup_or_cone if self.rec_cup_or_cone else "cup"
            self.pub_order.publish(order_msg)

            self.get_logger().info(f"[주문 완료] flavor={self.rec_flavor}, toppings={self.rec_toppings}, cup_or_cone={order_msg.cup_or_cone}")

            rclpy.shutdown()

            self.state = STATE_IDLE
            self.rec_flavor = ""
            self.rec_toppings = ""
            self.rec_cup_or_cone = ""

        elif any(x in text_lower for x in ["아니오", "no", "싫어", "아니", "아니요", "아뇨", "노노", "됐어", "아녀", "안녕", "하네요", "하니오", "하뇨", "하녀", "아네요", "아니유", "됐어", "아닝", "아냐", "안해도 돼", "이어"]):

            speak("알겠습니다. 다른 조합을 원하시면 다시 말씀해주세요.",
                  callback=lambda: speech_to_text(
                      self.on_preference_received,
                      phrase_time_limit=10,        # 다시 주문 취향 입력이므로 길게
                      pause_threshold=0.6,
                      non_speaking_duration=0.5
                  ))
            self.state = STATE_IDLE
            self.rec_flavor = ""
            self.rec_toppings = ""
            self.rec_cup_or_cone = ""
        else:
            speak("네 또는 아니오로만 말씀해주세요.",
                  callback=lambda: short_speech_to_text_vosk(
                  self.on_confirmation_received,
                  record_seconds=2
                  ))

#############################################
# 7️⃣ main
#############################################
def main(args=None):
    rclpy.init(args=args)
    node = IceCreamNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt → 노드 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
