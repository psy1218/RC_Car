from picamera2 import Picamera2
import cv2
from flask import Flask, Response
import serial
import time
import glob 

# === Serial 포트 설정 ===
def auto_serial_connect():
    candidates = glob.glob("/dev/ttyACM*")
    for port in candidates:
        try:
            s = serial.Serial(port, 9600, timeout=1)
            print(f"✅ Connected to {port}")
            return s
        except:
            continue
    raise Exception("❌ 아두이노 연결 실패: ttyACM 포트를 찾을 수 없습니다.")

ser = auto_serial_connect()
time.sleep(2)  # 아두이노 초기화 대기

def reconnect_serial():
    global ser
    try:
        ser.close()
    except:
        pass
    time.sleep(1)
    return auto_serial_connect()


# === PID 클래스 정의 ===
class PID:
    def __init__(self, Kp=0.3, Ki=0.005, Kd=0.1):
        self.Kp = Kp  # 현재 오차에 반응하는 정도
        self.Ki = Ki  # 오차 누적에 반응하는 정도
        self.Kd = Kd  # 오차 변화 속도에 반응하는 정도
        self.integral = 0  # 오차 누적값
        self.last_error = 0  # 이전 오차

    def compute(self, error):
        self.integral += error
        self.integral = max(min(self.integral, 100), -100)

        derivative = error - self.last_error
        self.last_error = error

        # 오차 작으면 누적 줄이기
        if abs(error) < 1:
            self.integral *= 0.8

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output

# PID 인스턴a스
pid = PID()

# 카메라 설정
yaho = Picamera2()
config = yaho.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"}  # ✅ 해상도 변경
)
yaho.configure(config)
yaho.set_controls({"FrameDurationLimits": (16666, 16666)})
yaho.start()

# === Flask 앱 설정 ===
# 전역 변수
prev_cx = 160
YAHO = Flask(__name__)

@YAHO.route('/')
def index():
    return "RC카 PID 제어 중"

# 영상 스트리밍 생성 함수
def generate():
    global prev_cx , ser
    
    while True:
        frame = yaho.capture_array("main")
        frame = cv2.flip(frame, 1)  # 좌우 반전

        # === ROI 설정 (범퍼 제외) ===
        roi_y_start = 280
        roi_y_end = 400
        roi = frame[roi_y_start:roi_y_end, :]

        # 전처리
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (11, 11), 10)


        binary = cv2.adaptiveThreshold(blur, 255,
        cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,15,3)

        # 노이즈 제거
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))  
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        # 윤곽선 필터링
        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 300]

        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] != 0:
                cx_raw = int(M['m10'] / M['m00'])

                # Moving Average 필터
                alpha = 0.5
                cx = int(prev_cx * (1 - alpha) + cx_raw * alpha)
                prev_cx = cx

                # PID 조향값 계산
                center_x = 320
                error = cx - center_x
                steering = pid.compute(error)
                steering = max(min(int(steering), 50), -50)  # -50~50 제한

                # Serial 전송
                try:
                    print(f"steering: {steering}, cx: {cx}, error: {error}")
                    ser.write(f"{steering}\n".encode())
                    ser.flush()  
                    time.sleep(0.01)  
                except serial.SerialException as e:
                    print(f"⚠️ Serial Error: {e}")
                    try:
                        ser.close()
                    except:
                        pass
                    ser = auto_serial_connect()
                    continue
                

                # 디버그 시각화
                cy = roi_y_start + (roi_y_end - roi_y_start) // 2
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                cv2.putText(frame, f"steer: {steering}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)


        # (선택) ROI 영역 시각화 - 노란 박스
        cv2.rectangle(frame, (0, roi_y_start), (640, roi_y_end), (0, 255, 255), 2)

        # 인코딩 및 전송
        ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        if ret:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')

@YAHO.route('/video_feed')
def video_feed():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# 서버 실행
if __name__ == "__main__":
    YAHO.run(host='0.0.0.0', port=5000, threaded=True)