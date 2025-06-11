from picamera2 import Picamera2  # 라즈베리파이에 연결된 카메라를 사용,처리하는 라이브러리
import cv2  # 라인트레이싱의 이미지 전처리 및 컨투어 탐색을 위한 라이브러리
from flask import Flask, Response  #웹 서버를 구성,영상 스트리밍
import serial  # 아두이노와 시리얼 통신을 위한 라이브러리
import time  # 대기시간을 위한 라이브러리.
import glob  # 시리얼 경로 탐색을 위해 사용
from collections import deque  # 이전 각도를 저장할 큐를 구현하기 위한 라이브러리 

# 진성씨 고생이 많으세요... ㅠㅠ 
################################################## 시리얼설정
# 라즈베리파이와 연결된 시리얼 포트를 찾아 반환하는 함수
def auto_serial_connect():
    candidates = glob.glob("/dev/ttyACM*")  # 연결 가능한 시리얼 포트를 찾습니다.
    for port in candidates:  #/dev/ttyACM*"로 검색된 모든 포트를 확인합니다.
        try:
            # 포트를 시도하며 연결합니다.
            serial_tmp = serial.Serial(port, 9600, timeout=1) # 9600 속도로 연결가능한 시리얼을 연결합니다. 
            print(f"✅ Connected to {port}")  # 연결 성공 시 포트를 출력합니다.
            return serial_tmp
        except:
            continue  # 연결 실패 시 다음 포트를 시도합니다.
    raise Exception("❌ 아두이노 연결 실패: ttyACM 포트를 찾을 수 없습니다.")

ser = auto_serial_connect()  # 초기 시리얼 연결 설정

# 아두이노 초기화 대기시간 설정
time.sleep(2)

#시리얼 연결을 재설정하는 함수
def reconnect_serial():
    
    global ser
    try:
        ser.close()  # 기존 연결을 닫습니다.
    except:
        pass
    time.sleep(1)
    return auto_serial_connect()  # 새로운 연결을 설정합니다.

################################################## 서버 생성,설정
YAHO = Flask(__name__)  # Flask 서버 객체를 생성합니다.

@YAHO.route('/')
def index():
    return "RC카 PID 제어 중"  # 기본 경로에 입장시 "RC카 PID 제어 중"메시지를 보여줍니다.


################################################### 카메라 설정 
yaho = Picamera2()  # 라즈베리파이 5에 연결된 카메라를 사용할 카메라 객체를 생성합니다.
config = yaho.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"}  #  카메라로 촬영한 이미지를 640*480픽셀, RGB로 받아옵니다.
)
yaho.configure(config)  # 설정을 적용합니다.

yaho.set_controls({"FrameRate": 30})  # 프레임 속도를 30으로 설정합니다.
yaho.set_controls({"Brightness": -0.5})  # 밝기를 -0.5로 설정합니다.
yaho.set_controls({"AeEnable": False}) # 자동밝기 조절 off
yaho.start()  # 카메라를 시작합니다.



################################################## 라인트레이싱

prev_x = 320  # 이전 프레임에서 인식한 라인의 중앙 위치를 저장합니다. 초기값은 프레임 중심입니다.
queue = deque(maxlen=10)  # 이전 10프레임의 라인 중앙 위치를 저장하는 큐입니다.
queue.append(320)  # 연산의 편의를 위해 초기값을 프레임 중심위치를 한 개를 추가합니다.


# 인식한 면적의 가중치를 매겨 점수를 계산합니다.
def weighted_contour_score(contour): 
    
    global prev_x, queue, arear_que  # generate함수에서 사용하는 글로벌변수들을 선언합니다.
    area = cv2.contourArea(contour)  # 컨투어의 면적을 계산합니다.

    min_area, max_area = 1800, 5500  # 면적의 유효 범위를 설정합니다.
    if area < min_area:
        return None  # 면적이 너무 작으면 무시합니다.
    if area > max_area:
        return None  # 면적이 너무 크면 무시합니다.

    prev_x_temp = prev_x  # 글로벌변수 prev_x값을 변경하여 사용하도록 로컬변수를 선언합니다.


    # 중심 좌표를 계산합니다.
    M = cv2.moments(contour) #탐색된 컨투어의 모멘트 정보를 저장합니다.
    if M['m00'] != 0: # 컨투어의 내부 픽셀값이 0이 아닌경우 
        cx = int(M['m10'] / M['m00'])  # x좌표와 픽셀곱의 합과 내부 픽셀값의 합으로 나눕니다.
        #전처리 과정에서 픽셀값들이 이진화 되었기 때문에 계산된 값은 중심위치가 됩니다.
    else:
        return None  # 유효하지 않은 컨투어는 무시합니다.

    if prev_x_temp is None: #이전좌표가 제대로 인식되지 못 한 경우
        prev_x_temp = int(sum(queue) / len(queue))  # 프레임 중심을 이전값으로 가정합니다. 

    # 이전 중심과 현재 중심의 거리 계산
    distance = abs(int(sum(queue) / len(queue)) - cx) # 이전10프레임간 인식한 라인의 중심의 평균과 현재 위치의 차이를 구합니다.
    max_distance = 320  # 중심과 거리차이의 최대값 설정

    # 정규화된 면적과 거리 계산
    normalized_area = (area - min_area) / (max_area - min_area) 
    normalized_distance = distance / max_distance

    # (가중치를 적용)
    distance_weight = 0.9  # 거리 가중치
    area_weight = 0.1  # 면적 가중치
    score = distance_weight * (1 - normalized_distance) + area_weight * normalized_area

    return score #계산된 점수 반환

# 영상 생성, 전처리, 스트리밍 함수
def generate():
    global prev_x, ser, queue

    while True:
        frame = yaho.capture_array("main")  # 카메라에서 프레임을 캡처합니다.
        frame = cv2.flip(frame, 1)  # 프레임을 좌우 반전합니다.

        offset = 30 # 라인을 인식하는 폭을 30*2로 설정합니다
        x1 = frame.shape[0] - 100  # 라인을 인식하는 위치1를 하단(범퍼제외)으로 설정합니다. 
        x2 = (frame.shape[0] >> 1) - 100  # 라인을 인식하는 위치2를 중,상단으로 설정합니다.

        
        roi1 = frame[x1 - offset:x1 + offset, :, :]#위치1에 해당하는 영역만큼을 프레임에서 복사합니다.
        roi2 = frame[x2 - offset:x2 + offset, :, :]#위치2에 해당하는 영역만큼을 프레임에서 복사합니다.

        # 각 채널RGB 에 대해 히스토그램 평활화 및 기타 전처리를 수행합니다.
        for roi in [roi1, roi2]:
            for i in range(3):
                roi[:, :, i] = cv2.equalizeHist(roi[:, :, i]) # 대비를 강조하여 라인의 경계를 뚜렷이 합니다.
                roi[:, :, i] = cv2.morphologyEx(roi[:, :, i], cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (30, 10)))
                #30*10크기 사각형으로 모폴로지 연산을 통해 노이즈를 제거합니다.
                roi[:, :, i] = cv2.morphologyEx(roi[:, :, i], cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (10, 30)))
                #10*30크기 사각형으로 모폴로지 연산을 통해 노이즈를 제거합니다.
                roi[:, :, i] = cv2.GaussianBlur(roi[:, :, i], (21, 21), 0)
                #21*21사이즈로 가우시안 블러 처리를해 이미지를 부드럽게 합니다.
                _, roi[:, :, i] = cv2.threshold(roi[:, :, i], 0, 100, cv2.THRESH_BINARY_INV)
                #밝기값 100을 기준으로 이진화합니다. 

        # ROI를 흑백으로 변환합니다.
        roi1_gray = cv2.cvtColor(roi1, cv2.COLOR_BGR2GRAY)
        roi2_gray = cv2.cvtColor(roi2, cv2.COLOR_BGR2GRAY)

        # 컨투어를 찾습니다.
        contours1, _ = cv2.findContours(roi1_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours2, _ = cv2.findContours(roi2_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 컨투어 필터링 및 점수 계산
        c1_max, c2_max = None, None

        for contours, x_pos in [(contours1, x1), (contours2, x2)]: #각 roi에서 인식된 경계선으로 도형을 추출합니다.
            contours = [c for c in contours if weighted_contour_score(c) is not None] #weighted_contour_score함수의 조건에 맞는 컨투어만을 받아와 저장합니다..
            if contours:
                largest = max(contours, key=weighted_contour_score) #weighted_contour_score함수에서 거리,면적의 가중치로 계산된 가장 높은 값을 라인으로 가정합니다.
                M = cv2.moments(largest) # 라인의 모멘트 정보를 저장합니다.
                if M['m00'] != 0:
                    cx_raw = int(M['m10'] / M['m00']) #라인의 중심위치를 계산합니다.
                    if x_pos == x1: #각 roi를 나누어 값을 저장합니다.
                        c1_max = cx_raw 
                    else:
                        c2_max = cx_raw
                    cv2.circle(frame, (cx_raw, x_pos), 5, (255, 255, 255), -1)

        # 중심 좌표 계산
        if c1_max is not None and c2_max is not None: # 두 위치에서 모두 검출된 경우
            prev_x = int((c1_max + c2_max * 2) / 3) # 앞에서 인식된 위치에 높은 가중치를 둡니다.
        elif c1_max is None and c2_max is not None: # 하나만 인식된 경우 그 값만을 사용합니다.
            prev_x = c2_max
        elif c1_max is not None and c2_max is None:
            prev_x = c1_max
        elif c1_max is None and c2_max is None:# 모두 인식되지 않은경우 최근 10프레임의 값을 토대로 왼쪽이나 오른쪽으로 이동하도록, 값을 프레임의 양 끝을 인식한것으로 가정합니다.
            prev_x = 0 if int(sum(queue) / len(queue)) <= 320 else 640

        queue.append(prev_x) # 계산된 중심위치 값을 큐에 삽입합니다.

        # 모터의각도로 변환 아두이노에서  -50 ~50의 값으로 받으드려 조향합니다.
        center_x = 320 #프레임의 중심위치
        steering = int(((prev_x - center_x) / center_x) * 50) #현재 인식된 라인의 중심을 -50 ~50으로 매핑합니다.
        steering = max(-49, min(steering, 49)) # 만약 벗어나는경우 변환가능한 최대,최소값으로 설정합니다.

        try:
            ser.write(f"{steering}\n".encode())  # 아두이노로 변환된 모터각도값을 전송합니다.
            ser.flush()  # 데이터 송신을 보장합니다.
            time.sleep(0.01)  # 짧은 지연 시간을 둡니다.
        except serial.SerialException as e: #만약 연결이 안되어 있다면
            print(f"⚠️ Serial Error: {e}")
            ser = auto_serial_connect()  # 시리얼 재연결을 시도합니다.

        # 스트리밍 데이터를 반환합니다.
        ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 30]) #jpg형태로 이미지 퀄리티를 30으로 하여 웹에 실시간으로 표시하도록 합니다.
        if ret:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')

@YAHO.route('/video_feed') # 비디오 스티리밍 경로를 설정합니다.
def video_feed():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame') #비디오 스트리밍 경로가 열리면 generate함수를 실행시키며 스트리밍을 시작합니다.

# 서버 실행
if __name__ == "__main__":
    YAHO.run(host='0.0.0.0', port=5001, threaded=False, debug=False, use_debugger=False, use_reloader=False) #서버를 실행합니다.
