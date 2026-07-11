#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
@Project : Autonomous Driving Edge AI & CAN Bridge
@File    : bridge_code.py
@Desc    : Raspberry Pi 환경에서 NCNN 딥러닝 프레임워크를 이용한 실시간 객체 인식 및 
           STM32 제어용 SocketCAN 통신 브릿지 시스템.
           - Multi-threading 기반 카메라 버퍼 지연(Latency) 최소화
           - NMS(Non-Maximum Suppression) 및 Bounding Box 원본 좌표 복원
           - FSM(Finite State Machine) 기반 자율주행 상태 판단 및 Fail-Safe 제어
=============================================================================
"""

import argparse
import os
import re
import time
import zipfile
import threading
from datetime import datetime

import cv2
import numpy as np

# NCNN 및 SocketCAN 라이브러리 (Edge 환경 최적화)
try:
    import ncnn
except ImportError:
    ncnn = None

try:
    import can
except ImportError:
    can = None

# 인식 대상 클래스 및 신뢰도(Confidence) 임계값 설정
CLASSES = [
    "child_protect", "child_protect_end", "crosswalk", 
    "green_light", "red_light", "yellow_light",
]

CLASS_CONF = {
    "child_protect": 0.35, "child_protect_end": 0.35, "crosswalk": 0.30,
    "green_light": 0.30, "red_light": 0.30, "yellow_light": 0.30,
}

# 시각화를 위한 Bounding Box 색상 맵핑 (OpenCV BGR 기준)
CLASS_COLORS = {
    "child_protect": (0, 255, 255), "child_protect_end": (255, 255, 0),
    "crosswalk": (255, 0, 255), "green_light": (0, 255, 0),
    "red_light": (0, 0, 255), "yellow_light": (0, 255, 255),
}

# 자율주행 차량 제어 상태 (CAN 프로토콜 규격)
STATE_STOP = 0
STATE_SLOW = 1
STATE_GO = 2

STATE_NAME = {STATE_STOP: "STOP", STATE_SLOW: "SLOW", STATE_GO: "GO"}


def parse_camera_source(value: str):
    """카메라 소스 파싱 (웹캠 인덱스 또는 자동 탐색)"""
    if value == "auto":
        return "auto"
    try:
        return int(value)
    except ValueError:
        return value


class LatestFrameCamera:
    """
    @brief 비동기 카메라 프레임 캡처 클래스 (Producer-Consumer Pattern)
    @note  OpenCV의 기본 버퍼링으로 인한 I/O 지연(Latency)을 해결하기 위해,
           별도의 스레드(Thread)에서 프레임을 지속적으로 읽고 메인 루프에는 Mutex(Lock)로
           보호된 최신(Latest) 프레임만 전달하여 실시간성을 보장함.
    """
    def __init__(self, source="auto", width=640, height=480, fps=30):
        self.cap = None
        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = 0.0
        self.running = False

        if source == "auto":
            print("🔍 사용 가능한 웹캠 포트를 탐색합니다...")
            for i in range(6):
                cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
                if cap.isOpened():
                    self._set_camera_options(cap, width, height, fps)
                    ok, frame = cap.read()
                    if ok and frame is not None:
                        self.cap = cap
                        print(f"✅ {i}번 포트에서 웹캠 영상 스트림을 찾았습니다.")
                        break
                    cap.release()
        else:
            self.cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
            self._set_camera_options(self.cap, width, height, fps)

        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError("유효한 웹캠을 찾을 수 없습니다.")

        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    @staticmethod
    def _set_camera_options(cap, width, height, fps):
        """웹캠 하드웨어 파라미터 강제 설정 (MJPG 코덱 사용으로 디코딩 부하 감소)"""
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # 하드웨어 버퍼 최소화

    def _reader(self):
        """[Thread] 지속적으로 카메라 프레임을 읽어와 버퍼를 비움"""
        while self.running:
            ok, frame = self.cap.read()
            if ok and frame is not None:
                with self.lock: # Race Condition 방지
                    self.frame = frame
                    self.frame_time = time.time()
            else:
                time.sleep(0.003)

    def read_latest(self):
        """[Main] 가장 최근에 캡처된 프레임 반환"""
        with self.lock:
            if self.frame is None:
                return None, 0.0
            return self.frame.copy(), self.frame_time

    def release(self):
        self.running = False
        if hasattr(self, "thread"):
            self.thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()


class AnnotatedVideoRecorder:
    """
    @brief 타임스탬프 기반 동적 FPS 영상 녹화기
    @note  AI 추론 속도(FPS) 변동으로 인해 저장된 영상의 재생 속도가 왜곡되는 현상을 
           방지하기 위해, 실제 시간(timestamp)을 기준으로 프레임을 동적으로 복제/스킵함.
    """
    def __init__(self, path, fps=20.0, width=0, height=0, realtime=True, max_duplicate=10):
        self.path = path
        self.fps = float(fps)
        self.width = int(width)
        self.height = int(height)
        self.realtime = bool(realtime)
        self.max_duplicate = max(1, int(max_duplicate))
        self.writer = None
        self.opened_size = None
        self.start_time = None
        self.frames_written = 0

    def _fourcc_from_path(self):
        ext = os.path.splitext(self.path)[1].lower()
        return cv2.VideoWriter_fourcc(*"XVID") if ext == ".avi" else cv2.VideoWriter_fourcc(*"mp4v")

    def _open(self, frame):
        h, w = frame.shape[:2]
        out_w = self.width if self.width > 0 else w
        out_h = self.height if self.height > 0 else h
        os.makedirs(os.path.dirname(os.path.abspath(self.path)) or ".", exist_ok=True)

        self.writer = cv2.VideoWriter(self.path, self._fourcc_from_path(), self.fps, (out_w, out_h))
        self.opened_size = (out_w, out_h)

    def write(self, frame, timestamp=None):
        if self.writer is None:
            self._open(frame)

        frame = cv2.resize(frame, self.opened_size, interpolation=cv2.INTER_LINEAR)

        if not self.realtime:
            self.writer.write(frame)
            self.frames_written += 1
            return

        now = time.time() if timestamp is None else float(timestamp)
        if self.start_time is None:
            self.start_time = now
            self.writer.write(frame)
            self.frames_written = 1
            return

        # 누적 경과 시간에 따른 필요 프레임 수 계산 및 동기화 (Framedrop/Duplication)
        elapsed = max(0.0, now - self.start_time)
        target_frames = int(elapsed * self.fps) + 1
        frames_to_write = min(target_frames - self.frames_written, self.max_duplicate)

        for _ in range(frames_to_write):
            self.writer.write(frame)

        self.frames_written += max(0, frames_to_write)

    def release(self):
        if self.writer is not None:
            self.writer.release()
            print(f"✅ 영상 저장 완료: {self.path} | frames={self.frames_written}")


def prepare_ncnn_dir(ncnn_dir, ncnn_zip=None):
    """NCNN 모델 압축 해제 및 디렉토리 검증"""
    if os.path.isdir(ncnn_dir):
        return ncnn_dir
    if ncnn_zip and os.path.isfile(ncnn_zip):
        extract_base = os.path.dirname(os.path.abspath(ncnn_zip)) or "."
        with zipfile.ZipFile(ncnn_zip, "r") as z:
            z.extractall(extract_base)
        candidate = os.path.join(extract_base, "best_ncnn_model")
        if os.path.isdir(candidate):
            return candidate
    raise FileNotFoundError(f"NCNN 모델 폴더를 찾을 수 없습니다: {ncnn_dir}")


def read_imgsz_from_metadata(ncnn_dir, default=320):
    """YOLO Export 시 생성된 metadata.yaml에서 Input Shape 동적 추출"""
    meta = os.path.join(ncnn_dir, "metadata.yaml")
    if not os.path.isfile(meta):
        return default
    text = open(meta, "r", encoding="utf-8", errors="ignore").read()
    m = re.search(r"imgsz:\s*\n\s*-\s*(\d+)\s*\n\s*-\s*(\d+)", text) or re.search(r"imgsz:\s*\[\s*(\d+)\s*,\s*(\d+)\s*\]", text)
    return int(m.group(1)) if m else default


def letterbox(image, new_shape=(320, 320), color=(114, 114, 114)):
    """
    @brief 모델 입력용 Affine 변환 (Letterboxing)
    @note  종횡비(Aspect Ratio)를 유지하면서 이미지를 리사이징하고, 남는 공간을 패딩함.
           원본 좌표계로 복원하기 위한 ratio와 padding 값을 반환.
    """
    h, w = image.shape[:2]
    new_h, new_w = new_shape
    r = min(new_w / w, new_h / h)
    resized_w, resized_h = int(round(w * r)), int(round(h * r))
    dw, dh = (new_w - resized_w) / 2, (new_h - resized_h) / 2

    if (w, h) != (resized_w, resized_h):
        image = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)

    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))

    image = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return image, r, (left, top)


def preprocess_bgr_to_chw(frame_bgr, imgsz):
    """OpenCV BGR(HWC) 포맷을 NCNN 입력용 RGB(CHW) Float32 포맷으로 변환"""
    img, ratio, pad = letterbox(frame_bgr, (imgsz, imgsz))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    arr = img.astype(np.float32) / 255.0
    chw = np.transpose(arr, (2, 0, 1))
    return np.ascontiguousarray(chw), ratio, pad


def _xywh_to_original_box(row, imgsz, ratio, pad, frame_shape):
    """YOLO 출력 텐서(Center X, Center Y, W, H)를 역연산하여 원본 이미지 픽셀 좌표(x1, y1, x2, y2)로 복원"""
    cx, cy, bw, bh = map(float, row[:4])

    if max(abs(cx), abs(cy), abs(bw), abs(bh)) <= 2.0: # 정규화된 좌표 방어 코드
        cx *= imgsz; cy *= imgsz; bw *= imgsz; bh *= imgsz

    x1, y1 = cx - bw / 2.0, cy - bh / 2.0
    x2, y2 = cx + bw / 2.0, cy + bh / 2.0
    pad_x, pad_y = pad

    x1, y1 = (x1 - pad_x) / ratio, (y1 - pad_y) / ratio
    x2, y2 = (x2 - pad_x) / ratio, (y2 - pad_y) / ratio

    frame_h, frame_w = frame_shape[:2]
    x1, y1 = int(max(0, min(frame_w - 1, round(x1)))), int(max(0, min(frame_h - 1, round(y1))))
    x2, y2 = int(max(0, min(frame_w - 1, round(x2)))), int(max(0, min(frame_h - 1, round(y2))))

    return None if x2 <= x1 or y2 <= y1 else [x1, y1, x2, y2]


def postprocess_yolo_output(output, class_conf, default_conf=0.30, iou_thres=0.45, imgsz=320, ratio=1.0, pad=(0, 0), frame_shape=None):
    """
    @brief YOLO 추론 결과 후처리 및 NMS (Non-Maximum Suppression) 적용
    @note  클래스별 임계값을 적용하고, 중복 인식된 Bounding Box를 IoU 기준으로 제거함.
    """
    pred = np.squeeze(output)
    if pred.ndim == 1: pred = pred[None, :]
    if pred.shape[0] in {4 + len(CLASSES), 5 + len(CLASSES)} and pred.shape[1] > pred.shape[0]:
        pred = pred.T

    boxes_by_class, scores_by_class, detections_by_class = ({i: [] for i in range(len(CLASSES))} for _ in range(3))

    for row in pred:
        attrs = row.shape[0]
        if attrs >= 5 + len(CLASSES):
            obj = float(row[4])
            scores = row[5:5 + len(CLASSES)]
            cls_id = int(np.argmax(scores))
            conf = obj * float(scores[cls_id])
        elif attrs >= 4 + len(CLASSES):
            scores = row[4:4 + len(CLASSES)]
            cls_id = int(np.argmax(scores))
            conf = float(scores[cls_id])
        else: continue

        cls_name = CLASSES[cls_id]
        if conf < class_conf.get(cls_name, default_conf): continue

        box = _xywh_to_original_box(row, imgsz, ratio, pad, frame_shape)
        if not box: continue

        x1, y1, x2, y2 = box
        boxes_by_class[cls_id].append([x1, y1, x2 - x1, y2 - y1])
        scores_by_class[cls_id].append(float(conf))
        detections_by_class[cls_id].append({"class_id": cls_id, "class_name": cls_name, "conf": float(conf), "box": box})

    final_detections = []
    # OpenCV DNN 모듈을 활용한 클래스 독립적 NMS 처리
    for cls_id in range(len(CLASSES)):
        if not boxes_by_class[cls_id]: continue
        idxs = cv2.dnn.NMSBoxes(boxes_by_class[cls_id], scores_by_class[cls_id], 0.0, iou_thres)
        if len(idxs) == 0: continue
        for i in np.array(idxs).flatten():
            final_detections.append(detections_by_class[cls_id][int(i)])

    final_detections.sort(key=lambda d: d["conf"], reverse=True)
    return final_detections


class NcnnYolo:
    """NCNN 딥러닝 추론 엔진 래퍼(Wrapper)"""
    def __init__(self, ncnn_dir, imgsz, threads=4, input_name="in0", output_name="out0"):
        if ncnn is None: raise RuntimeError("ncnn 패키지가 필요합니다.")
        self.imgsz = imgsz
        self.input_name = input_name
        self.output_name = output_name
        self.param_path = os.path.join(ncnn_dir, "model.ncnn.param")
        self.bin_path = os.path.join(ncnn_dir, "model.ncnn.bin")

        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False
        self.net.opt.num_threads = threads
        self.net.load_param(self.param_path)
        self.net.load_model(self.bin_path)

    def infer(self, frame_bgr):
        chw, ratio, pad = preprocess_bgr_to_chw(frame_bgr, self.imgsz)
        ex = self.net.create_extractor()
        try:
            ex.input(self.input_name, ncnn.Mat(chw).clone())
            ret, out = ex.extract(self.output_name)
            if ret != 0: raise RuntimeError(f"NCNN extract 실패: ret={ret}")
            return np.array(out), ratio, pad
        finally:
            try: ex.clear()
            except Exception: pass


def unique_class_names(detections):
    return list({det["class_name"] for det in detections})


def decide_state(detected_classes, ctx, args):
    """
    @brief 자율주행 유한 상태 기계 (Finite State Machine)
    @note  비전 인식 결과를 바탕으로 주행 정책(Policy)을 결정.
           스쿨존 유지, 횡단보도 정지(타이머 락), 신호등 판별 로직을 우선순위에 따라 처리.
    """
    now = time.time()
    detected = set(detected_classes)

    # 1. 스쿨존 구역 판단 (Context 메모리 유지)
    if "child_protect" in detected:
        ctx["is_school_zone"] = True
    elif "child_protect_end" in detected:
        ctx["is_school_zone"] = False

    new_state = STATE_SLOW if ctx["is_school_zone"] else STATE_GO

    # 2. 횡단보도 제어 로직 (정지 후 쿨다운 메커니즘 적용)
    if ctx["crosswalk_state"] == "IDLE":
        if "crosswalk" in detected:
            ctx["crosswalk_state"] = "STOPPED"
            ctx["stop_time"] = now
            new_state = STATE_STOP
    elif ctx["crosswalk_state"] == "STOPPED":
        if now - ctx["stop_time"] < args.crosswalk_stop:
            new_state = STATE_STOP
        else:
            ctx["crosswalk_state"] = "COOLDOWN"
            ctx["cooldown_time"] = now
            new_state = STATE_SLOW if ctx["is_school_zone"] else STATE_GO
    elif ctx["crosswalk_state"] == "COOLDOWN":
        if now - ctx["cooldown_time"] >= args.crosswalk_cooldown:
            ctx["crosswalk_state"] = "IDLE"

    # 3. 신호등 판단 (가장 높은 우선순위로 상태 덮어쓰기)
    if "red_light" in detected:
        new_state = STATE_STOP
    elif "yellow_light" in detected and new_state != STATE_STOP:
        new_state = STATE_SLOW
    elif "green_light" in detected and ctx["crosswalk_state"] != "STOPPED":
        new_state = STATE_SLOW if ctx["is_school_zone"] else STATE_GO

    return new_state


def open_can(args):
    """SocketCAN 인터페이스 초기화"""
    if args.no_can: return None
    if can is None: raise RuntimeError("python-can 패키지가 필요합니다.")
    return can.interface.Bus(channel=args.can_channel, interface="socketcan")


def send_state(bus, can_id, state):
    """결정된 상태(State)를 STM32로 CAN 송신"""
    if bus is None: return
    msg = can.Message(arbitration_id=can_id, data=[state, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)
    bus.send(msg)


def draw_detections(frame, detections):
    """시각화: 인식된 Bounding Box 및 Label 표시"""
    for det in detections:
        class_name, conf, (x1, y1, x2, y2) = det["class_name"], det["conf"], det["box"]
        color = CLASS_COLORS.get(class_name, (255, 255, 255))
        label = f"{class_name} {conf:.2f}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
        
        bg_y1 = y1 if y1 - (th + baseline + 8) < 0 else y1 - (th + baseline + 8)
        cv2.rectangle(frame, (x1, bg_y1), (x1 + tw + 8, bg_y1 + th + baseline + 8), color, -1)
        cv2.putText(frame, label, (x1 + 4, bg_y1 + th + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
    return frame


def draw_status(frame, fps, infer_ms, state, detected_classes, ctx):
    """시각화: 시스템 상태(FPS, FSM Status) 오버레이"""
    text_1 = f"FPS {fps:.1f} | infer {infer_ms:.1f}ms | State {STATE_NAME.get(state, state)}"
    text_2 = f"detect: {detected_classes if detected_classes else 'none'} | zone={ctx['is_school_zone']} | cross={ctx['crosswalk_state']}"
    cv2.rectangle(frame, (5, 5), (600, 60), (0, 0, 0), -1)
    cv2.putText(frame, text_1, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    cv2.putText(frame, text_2, (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    return frame


def build_record_path(args):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return args.record_path if args.record_path else f"demo_detect_{ts}.mp4"


def main():
    parser = argparse.ArgumentParser(description="YOLO NCNN -> STM32F446 CAN bridge + recorder")
    parser.add_argument("--ncnn-dir", default="best_ncnn_model")
    parser.add_argument("--ncnn-zip", default=None)
    parser.add_argument("--camera", default="auto")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--imgsz", type=int, default=0)
    parser.add_argument("--conf", type=float, default=0.30)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--input-name", default="in0")
    parser.add_argument("--output-name", default="out0")
    parser.add_argument("--can-channel", default="can0")
    parser.add_argument("--can-id", type=lambda x: int(x, 0), default=0x100)
    parser.add_argument("--no-can", action="store_true")
    parser.add_argument("--crosswalk-stop", type=float, default=3.0)
    parser.add_argument("--crosswalk-cooldown", type=float, default=10.0)
    parser.add_argument("--log-interval", type=float, default=0.25)
    parser.add_argument("--send-interval", type=float, default=0.05)
    parser.add_argument("--detect-every", type=int, default=1)
    parser.add_argument("--box-hold", type=float, default=0.15)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--record-path", default=None)
    parser.add_argument("--record-fps", type=float, default=20.0)
    parser.add_argument("--record-mode", choices=["realtime", "loop"], default="realtime")
    parser.add_argument("--record-max-duplicate", type=int, default=10)
    parser.add_argument("--show-status", action="store_true")

    args = parser.parse_args()
    args.detect_every = max(1, int(args.detect_every))

    ncnn_dir = prepare_ncnn_dir(args.ncnn_dir, args.ncnn_zip)
    imgsz = args.imgsz if args.imgsz > 0 else read_imgsz_from_metadata(ncnn_dir, default=320)

    print("=" * 78)
    print("🚀 자율주행 Edge AI 시작 (NCNN + CAN Bridge)")
    print("=" * 78)

    bus = open_can(args)
    model = NcnnYolo(ncnn_dir, imgsz, args.threads, args.input_name, args.output_name)
    cam = LatestFrameCamera(parse_camera_source(args.camera), args.width, args.height, args.fps)

    recorder = AnnotatedVideoRecorder(build_record_path(args), fps=args.record_fps, width=args.width, height=args.height, realtime=(args.record_mode == "realtime"), max_duplicate=args.record_max_duplicate) if args.record else None

    # Context 딕셔너리를 활용하여 프레임 간 상태 영속성(Persistence) 유지
    ctx = {"is_school_zone": False, "crosswalk_state": "IDLE", "stop_time": 0.0, "cooldown_time": 0.0}

    current_state, last_sent_state = STATE_GO, None
    last_send_time, last_log_time, fps_time = 0.0, 0.0, time.time()
    loop_count, fps_frame_count = 0, 0
    display_fps, last_infer_ms = 0.0, 0.0
    last_raw_detections, last_draw_detections = [], []
    last_draw_time = 0.0

    try:
        while True:
            frame, frame_time = cam.read_latest()
            if frame is None:
                time.sleep(0.005)
                continue

            loop_count += 1
            fps_frame_count += 1
            now = time.time()

            # 연산 부하 조절을 위한 Frame Drop 적용 (detect-every)
            if (loop_count % args.detect_every) == 0:
                t0 = time.time()
                output, ratio, pad = model.infer(frame)
                last_infer_ms = (time.time() - t0) * 1000.0

                last_raw_detections = postprocess_yolo_output(
                    output, CLASS_CONF, default_conf=args.conf, iou_thres=args.iou, 
                    imgsz=imgsz, ratio=ratio, pad=pad, frame_shape=frame.shape
                )
                if last_raw_detections:
                    last_draw_detections = last_raw_detections
                    last_draw_time = time.time()

            detected_classes = unique_class_names(last_raw_detections)
            new_state = decide_state(detected_classes, ctx, args)
            now = time.time()

            # CAN 통신 트래픽 최적화: 상태 변경 시 또는 Heartbeat 주기에만 전송
            if (new_state != last_sent_state) or (now - last_send_time >= args.send_interval):
                send_state(bus, args.can_id, new_state)
                last_sent_state = new_state
                last_send_time = now

            if new_state != current_state:
                print(f"➡️ 상태 변경: {STATE_NAME.get(current_state)} -> {STATE_NAME.get(new_state)}")
            current_state = new_state

            # 모니터링 로그 출력 (log-interval)
            if now - last_log_time >= args.log_interval:
                display_fps = fps_frame_count / (now - fps_time) if (now - fps_time) > 0 else 0.0
                last_log_time, fps_frame_count, fps_time = now, 0, now

            # 시각화 및 영상 녹화 처리
            annotated = frame.copy() if (args.display or args.record) else frame
            if args.display or args.record:
                if last_raw_detections:
                    draw_detections(annotated, last_raw_detections)
                elif args.box_hold > 0 and (now - last_draw_time) <= args.box_hold:
                    draw_detections(annotated, last_draw_detections)

                if args.show_status:
                    draw_status(annotated, display_fps, last_infer_ms, current_state, detected_classes, ctx)

            if recorder:
                recorder.write(annotated, timestamp=now)

            if args.display:
                cv2.imshow("Edge AI Vision", annotated)
                if (cv2.waitKey(1) & 0xFF) == ord("q"): break

    except KeyboardInterrupt:
        print("\n🛑 프로그램 강제 종료 (Ctrl+C)")
    finally:
        cam.release()
        if recorder: recorder.release()
        if args.display: cv2.destroyAllWindows()

        # 💡 [핵심] Graceful Shutdown: 프로그램 종료 시 차량 폭주(Runaway) 방지를 위해 정지(STOP) 명령 강제 전송
        if bus is not None:
            try:
                print("\n🛑 [Fail-Safe] 자동차 정지(STOP) 명령을 전송합니다...")
                for _ in range(3):
                    send_state(bus, args.can_id, STATE_STOP)
                    time.sleep(0.05)
                bus.shutdown()
            except Exception as e:
                print(f"⚠️ 정지 명령 전송 실패: {e}")

        print("👋 시스템 리소스 해제 완료")

if __name__ == "__main__":
    main()
