#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
사람 추적 팬/틸트 카메라 - 라즈베리파이 측 비전 파이프라인

역할 분담
  라즈베리파이 : 영상 캡처 -> 사람 검출 -> "화면 중앙 대비 픽셀 오차" 산출 -> UART 송신
  STM32        : 오차 수신 -> PD 제어 -> 서보 PWM 구동
  즉 파이는 '어디에 있는지'만 알려주고 '어떻게 움직일지'는 STM32가 결정한다.

스레드 구성 (의도적으로 4개로 분리)
  [T1] capture   : libcamera-vid 파이프에서 MJPEG 프레임을 잘라내 최신 프레임 유지
  [T2] inference : YOLOv4-tiny 추론 -> 추적 상태(오차/검출여부) 갱신
  [T3] uart_tx   : 추론 속도와 무관하게 고정 20Hz로 패킷 송신 (링크 keepalive)
  [T4] rtsp      : 주석이 그려진 프레임을 ffmpeg에 밀어넣어 H.264 RTSP로 송출
  + Flask(메인)  : 디버그용 MJPEG over HTTP

  T2와 T3를 분리한 이유:
    추론이 느려지거나(5fps) 사람이 안 잡혀도 송신 주기는 20Hz로 일정해야 한다.
    STM32는 패킷이 300ms 끊기면 '링크 두절'로 판단하므로, 검출 여부와 무관하게
    일정 주기로 계속 보내야 정상 상태와 장애 상태를 구분할 수 있다.
"""

import os
import shutil
import subprocess
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response

# ---------------------------------------------------------------
# 설정 (환경변수로 덮어쓸 수 있게 해 다른 환경에서도 실행 가능하도록)
# ---------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))

FRAME_W = int(os.getenv("FRAME_W", 640))
FRAME_H = int(os.getenv("FRAME_H", 480))
CAM_FPS = int(os.getenv("CAM_FPS", 24))
CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2

MODEL_CFG = os.getenv("MODEL_CFG", os.path.join(HERE, "yolov4-tiny.cfg"))
MODEL_WEIGHTS = os.getenv("MODEL_WEIGHTS", os.path.join(HERE, "yolov4-tiny.weights"))
INPUT_SIZE = int(os.getenv("INPUT_SIZE", 256))   # cfg는 416이지만 파이 성능상 256으로 축소
CONF_TH = float(os.getenv("CONF_TH", 0.4))
NMS_TH = float(os.getenv("NMS_TH", 0.4))
PERSON_CLASS_ID = 0                              # COCO 데이터셋의 person

SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/serial0")
SERIAL_BAUD = int(os.getenv("SERIAL_BAUD", 115200))
TX_HZ = float(os.getenv("TX_HZ", 20.0))          # STM32 링크 타임아웃(300ms)보다 충분히 빠르게

HTTP_PORT = int(os.getenv("HTTP_PORT", 5001))

RTSP_ENABLE = os.getenv("RTSP_ENABLE", "1") == "1"
RTSP_MODE = os.getenv("RTSP_MODE", "push")       # push: MediaMTX로 푸시 / listen: ffmpeg가 직접 서버
RTSP_URL = os.getenv("RTSP_URL", "rtsp://127.0.0.1:8554/live")
RTSP_FPS = int(os.getenv("RTSP_FPS", 15))
RTSP_BITRATE = os.getenv("RTSP_BITRATE", "2M")


# ---------------------------------------------------------------
# 공유 상태
#   프레임은 '항상 새 객체로 교체'하고 소비자는 절대 in-place 수정하지 않는다.
#   덕분에 참조 교체만 락으로 보호하면 되고 프레임 복사 비용을 락 밖으로 뺄 수 있다.
# ---------------------------------------------------------------
class FrameStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None
        self._boxes = []

    def set_frame(self, frame):
        with self._lock:
            self._frame = frame

    def set_boxes(self, boxes):
        with self._lock:
            self._boxes = boxes

    def get(self):
        with self._lock:
            return self._frame, list(self._boxes)


class TrackState:
    """추론 스레드가 쓰고 UART 송신 스레드가 읽는 추적 상태."""

    def __init__(self):
        self._lock = threading.Lock()
        self._err_x = 0
        self._err_y = 0
        self._detected = False

    def update(self, err_x, err_y, detected):
        with self._lock:
            self._err_x = err_x
            self._err_y = err_y
            self._detected = detected

    def snapshot(self):
        with self._lock:
            return self._err_x, self._err_y, self._detected


frames = FrameStore()
track = TrackState()
stop_event = threading.Event()


# ---------------------------------------------------------------
# 공통 유틸
# ---------------------------------------------------------------
def draw_overlay(frame, boxes):
    """검출 박스와 조준선을 그린 새 프레임을 반환 (원본 미변경)."""
    out = frame.copy()
    cv2.drawMarker(out, (CENTER_X, CENTER_Y), (0, 0, 255),
                   cv2.MARKER_CROSS, 20, 1)
    for (box, conf) in boxes:
        x, y, w, h = box
        cx, cy = int(x + w / 2), int(y + h / 2)
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.line(out, (CENTER_X, CENTER_Y), (cx, cy), (255, 200, 0), 1)
        cv2.putText(out, "Human {:.0f}%".format(conf * 100), (x, max(y - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return out


# ---------------------------------------------------------------
# [T1] 카메라 캡처
#   cv2.VideoCapture 대신 libcamera-vid를 subprocess로 띄우고 stdout 파이프를 직접 읽는다.
#   MJPEG 스트림에는 프레임 경계가 없으므로 JPEG 마커로 직접 잘라낸다.
#     SOI(Start of Image) = FF D8 , EOI(End of Image) = FF D9
# ---------------------------------------------------------------
def capture_thread():
    print("[T1] 카메라 캡처 시작")
    cmd = [
        "libcamera-vid", "-t", "0",
        "--width", str(FRAME_W), "--height", str(FRAME_H),
        "--framerate", str(CAM_FPS),
        "--codec", "mjpeg", "-o", "-",
    ]
    pipe = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    buf = b""
    try:
        while not stop_event.is_set():
            chunk = pipe.stdout.read(4096)
            if not chunk:
                break
            buf += chunk

            soi = buf.find(b"\xff\xd8")
            eoi = buf.find(b"\xff\xd9", soi + 2) if soi != -1 else -1
            if soi != -1 and eoi != -1:
                jpg = buf[soi:eoi + 2]
                buf = buf[eoi + 2:]
                frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    frames.set_frame(frame)

            # 마커를 못 찾은 채 버퍼가 비정상적으로 커지면 스트림이 깨진 것이므로 버린다
            if len(buf) > 4 * 1024 * 1024:
                buf = b""
    finally:
        pipe.terminate()
        print("[T1] 카메라 캡처 종료")


# ---------------------------------------------------------------
# [T2] YOLOv4-tiny 추론
# ---------------------------------------------------------------
def inference_thread():
    print("[T2] 추론 스레드 시작")
    net = cv2.dnn.readNet(MODEL_WEIGHTS, MODEL_CFG)
    model = cv2.dnn_DetectionModel(net)
    model.setInputParams(size=(INPUT_SIZE, INPUT_SIZE), scale=1 / 255, swapRB=True)

    while not stop_event.is_set():
        frame, _ = frames.get()
        if frame is None:
            time.sleep(0.01)
            continue

        t0 = time.perf_counter()
        class_ids, confidences, boxes = model.detect(
            frame, confThreshold=CONF_TH, nmsThreshold=NMS_TH)
        infer_ms = (time.perf_counter() - t0) * 1000

        persons = [(box, float(conf))
                   for cid, conf, box in zip(class_ids, confidences, boxes)
                   if int(cid) == PERSON_CLASS_ID]
        frames.set_boxes(persons)

        if persons:
            # 가장 큰 박스를 추적 대상으로 선택 (YOLO 출력 순서에 의존하면 대상이 튄다)
            box, _conf = max(persons, key=lambda p: p[0][2] * p[0][3])
            x, y, w, h = box
            err_x = int(x + w / 2) - CENTER_X
            err_y = int(y + h / 2) - CENTER_Y
            track.update(err_x, err_y, True)
        else:
            # 검출 실패를 명시적으로 알린다. 마지막 오차를 남겨두면 STM32가
            # 그 값을 계속 적분해 카메라가 한계각까지 밀려가는 드리프트가 발생한다.
            track.update(0, 0, False)

        print("[T2] infer={:6.1f}ms  persons={}".format(infer_ms, len(persons)), end="\r")


# ---------------------------------------------------------------
# [T3] UART 송신 - 고정 주기 (검출 여부와 무관하게 항상 송신)
#   프로토콜 v2 : X<오차x>Y<오차y>D<0|1>\n     D=1 검출 / D=0 미검출
# ---------------------------------------------------------------
def uart_tx_thread():
    try:
        import serial
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    except Exception as exc:
        print("[T3] 시리얼 열기 실패({}) - 송신 없이 계속 진행합니다".format(exc))
        return

    print("[T3] UART 송신 시작 {}@{}, {:.0f}Hz".format(SERIAL_PORT, SERIAL_BAUD, TX_HZ))
    period = 1.0 / TX_HZ
    next_at = time.monotonic()
    try:
        while not stop_event.is_set():
            err_x, err_y, detected = track.snapshot()
            packet = "X{}Y{}D{}\n".format(err_x, err_y, 1 if detected else 0)
            try:
                ser.write(packet.encode("ascii"))
            except Exception as exc:
                print("\n[T3] 송신 오류: {}".format(exc))

            next_at += period
            time.sleep(max(0.0, next_at - time.monotonic()))
    finally:
        ser.close()
        print("[T3] UART 송신 종료")


# ---------------------------------------------------------------
# [T4] RTSP 송출
#   OpenCV로 주석을 그린 BGR 프레임을 ffmpeg stdin에 그대로 밀어넣고
#   ffmpeg가 H.264로 인코딩해 RTSP로 내보낸다.
#
#   mode=push   : MediaMTX 같은 RTSP 서버에 푸시 (권장, 다중 시청자 안정적)
#   mode=listen : ffmpeg 자체가 RTSP 서버로 대기 (외부 의존성 없음, 단일 시청자)
# ---------------------------------------------------------------
def pick_encoder():
    """하드웨어 인코더가 있으면 우선 사용하고 없으면 libx264로 폴백."""
    try:
        out = subprocess.run(["ffmpeg", "-hide_banner", "-encoders"],
                             capture_output=True, text=True, timeout=10).stdout
        if "h264_v4l2m2m" in out:
            return "h264_v4l2m2m"
    except Exception:
        pass
    return "libx264"


def build_ffmpeg_cmd(encoder):
    cmd = [
        "ffmpeg", "-hide_banner", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "bgr24",
        "-s", "{}x{}".format(FRAME_W, FRAME_H), "-r", str(RTSP_FPS),
        "-i", "-",
        "-an",
        "-c:v", encoder,
        "-pix_fmt", "yuv420p",
        "-b:v", RTSP_BITRATE,
        "-g", str(RTSP_FPS),   # GOP=1초 -> 시청자가 늦게 붙어도 1초 안에 첫 화면
        "-bf", "0",            # B프레임 제거 -> 인코딩 지연 최소화
    ]
    if encoder == "libx264":
        cmd += ["-preset", "ultrafast", "-tune", "zerolatency"]
    cmd += ["-f", "rtsp", "-rtsp_transport", "tcp"]
    if RTSP_MODE == "listen":
        cmd += ["-rtsp_flags", "listen"]
    cmd += [RTSP_URL]
    return cmd


def rtsp_thread():
    if not shutil.which("ffmpeg"):
        print("[T4] ffmpeg가 없어 RTSP를 비활성화합니다")
        return

    encoder = pick_encoder()
    cmd = build_ffmpeg_cmd(encoder)
    print("[T4] RTSP 송출 시작 ({}, mode={}) -> {}".format(encoder, RTSP_MODE, RTSP_URL))

    period = 1.0 / RTSP_FPS
    proc = None
    next_at = time.monotonic()

    while not stop_event.is_set():
        # ffmpeg가 죽어 있으면 재기동한다 (MediaMTX 재시작이나 네트워크 순단 대비)
        if proc is None or proc.poll() is not None:
            if proc is not None:
                print("\n[T4] ffmpeg가 종료됨 - 2초 후 재기동")
                time.sleep(2.0)
            proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL)
            next_at = time.monotonic()

        frame, boxes = frames.get()
        if frame is not None:
            try:
                proc.stdin.write(draw_overlay(frame, boxes).tobytes())
            except (BrokenPipeError, ValueError, OSError):
                # 파이프가 끊기면 다음 루프에서 재기동시킨다
                try:
                    proc.kill()
                except Exception:
                    pass
                proc = None
                continue

        next_at += period
        time.sleep(max(0.0, next_at - time.monotonic()))

    if proc is not None:
        proc.kill()
    print("[T4] RTSP 송출 종료")


# ---------------------------------------------------------------
# Flask - 디버그용 MJPEG over HTTP
#   RTSP와 병행하는 이유: 브라우저에서 플러그인 없이 즉시 확인 가능해
#   현장 디버깅 속도가 압도적으로 빠르다 (대역폭은 RTSP보다 훨씬 큼).
# ---------------------------------------------------------------
app = Flask(__name__)


@app.route("/")
def index():
    return ("<h2>Human Tracking Camera</h2>"
            "<img src='/video_feed' width='{}'>"
            "<p>RTSP: <code>{}</code></p>").format(
        FRAME_W, RTSP_URL if RTSP_ENABLE else "(disabled)")


@app.route("/video_feed")
def video_feed():
    def gen():
        while not stop_event.is_set():
            frame, boxes = frames.get()
            if frame is None:
                time.sleep(0.01)
                continue
            ok, buf = cv2.imencode(".jpg", draw_overlay(frame, boxes))
            if ok:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + buf.tobytes() + b"\r\n")
            time.sleep(0.03)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


# ---------------------------------------------------------------
def main():
    workers = [
        threading.Thread(target=capture_thread, daemon=True),
        threading.Thread(target=inference_thread, daemon=True),
        threading.Thread(target=uart_tx_thread, daemon=True),
    ]
    if RTSP_ENABLE:
        workers.append(threading.Thread(target=rtsp_thread, daemon=True))

    for w in workers:
        w.start()

    try:
        app.run(host="0.0.0.0", port=HTTP_PORT, debug=False, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        time.sleep(0.5)


if __name__ == "__main__":
    main()
