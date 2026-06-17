from flask import Flask, Response
import cv2
import numpy as np
import subprocess
import threading
import time
import serial

# 🔌 [라즈베리파이 UART 순정 핀 포트 활성화]
ser = serial.Serial('/dev/serial0', 115200, timeout=1)

app = Flask(__name__)

# 1. YOLOv4-tiny 딥러닝 경량화 모델 로드
net = cv2.dnn.readNet('/home/JSH/sub_project/yolov4-tiny.weights', '/home/JSH/sub_project/yolov4-tiny.cfg')
model = cv2.dnn_DetectionModel(net)
model.setInputParams(size=(256, 256), scale=1/255, swapRB=True)

# 쓰레드 간 실시간 이미지 공유 전역 스토리지
current_frame = None
detected_boxes = []

# [쓰레드 1] 백그라운드 카메라 영상 무한 캡처 파이프라인
def video_capture_thread():
    global current_frame
    print("▶ [카메라 쓰레드] 가동 시작!")
    
    cmd = [
        'libcamera-vid', '-t', '0',
        '--width', '640', '--height', '480',
        '--framerate', '24', '--codec', 'mjpeg', '-o', '-'
    ]
    pipe = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    buffer_bytes = b''
    
    try:
        while True:
            chunk = pipe.stdout.read(4096)
            if not chunk: break
            buffer_bytes += chunk
            
            a = buffer_bytes.find(b'\xff\xd8')
            b = buffer_bytes.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg_data = buffer_bytes[a:b+2]
                buffer_bytes = buffer_bytes[b+2:]
                
                frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    current_frame = frame
    finally:
        pipe.terminate()

# [쓰레드 2] 사람(ID 0번) 초고속 판별 및 STM32 실시간 오차 송신 엔진 (중복 제거 및 통합 완료)
def ai_inference_thread():
    global current_frame, detected_boxes
    print("▶ [AI 딥러닝 쓰레드] 가동 시작!")
    frame_count = 0
    
    while True:
        if current_frame is None:
            time.sleep(0.01)
            continue
            
        img = current_frame.copy()
        frame_count += 1
        print(f"   [AI 스캔] {frame_count}번째 프레임 연산 중...", end="\r")
        
        # YOLO 스캔 작동
        class_ids, confidences, boxes = model.detect(img, confThreshold=0.4, nmsThreshold=0.4)
        
        temp_boxes = []
        for (class_id, confidence, box) in zip(class_ids, confidences, boxes):
            # 사람(0번) 정수만 즉각 추려내어 연산 속도 최적화
            if int(class_id) == 0:
                temp_boxes.append((box, float(confidence)))
                
        detected_boxes = temp_boxes
        
        # 🎯 [실전 데이터 송신 연산 주입]
        if len(temp_boxes) > 0:
            (x, y, w, h), conf = temp_boxes[0]
            
            # 사람 바운딩 박스의 중앙 좌표 계산
            cx = x + (w / 2)
            cy = y + (h / 2)
            
            # 640x480 화면 정중앙(320, 240) 기준 픽셀 오차 도출
            err_x = int(cx - 320)
            err_y = int(cy - 240)
            
            # STM32가 수신할 정품 패킷 전송 (예: X-45Y23\n)
            packet = f"X{err_x}Y{err_y}\n"
            ser.write(packet.encode('utf-8'))
            print(f"📡 STM32로 전송 중 -> {packet.strip()}")
            
        time.sleep(0.01)

# [Flask 웹 스트리밍 루프]
def generate_frames():
    global current_frame, detected_boxes
    
    while True:
        if current_frame is None:
            time.sleep(0.01)
            continue
            
        display_frame = current_frame.copy()
        
        for (box, conf) in detected_boxes:
            x, y, w, h = box
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            text = f"Human: {conf * 100:.1f}%"
            cv2.putText(display_frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        ret, buffer = cv2.imencode('.jpg', display_frame)
        if not ret: continue
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.03)

@app.route('/')
def index():
    return "<h1>Raspberry Pi AI Smooth YOLOv4</h1><img src='/video_feed' width='640'>"

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    t1 = threading.Thread(target=video_capture_thread, daemon=True)
    t2 = threading.Thread(target=ai_inference_thread, daemon=True)
    t1.start()
    t2.start()
    
    app.run(host='0.0.0.0', port=5001, debug=False)