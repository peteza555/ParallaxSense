"""ParallaxSense — AI ท่านั่งจาก USB webcam (รันบน laptop)

ขั้นตอน:  MediaPipe Pose (pre-trained) หาจุดบนร่างกาย -> แปลงเป็น feature ที่ไม่ขึ้นกับขนาดภาพ
          -> โมเดล RandomForest ที่เทรนจากข้อมูลของผู้ใช้เอง ตัดสิน good / bad + ความมั่นใจ
          -> ส่งผลขึ้น NETPIE: @msg/ai (ให้ gateway คุม servo = closed loop, M4) และ shadow ของ device AI
ภาพจากกล้องไม่ถูกส่งออกจากเครื่อง ส่งแค่ผลลัพธ์ที่เป็นตัวเลข

คำสั่ง (รันจากโฟลเดอร์โปรเจกต์):
  ai/.venv/Scripts/python ai/posture_ai.py cams               หา index ของกล้อง
  ai/.venv/Scripts/python ai/posture_ai.py check              ทดสอบกล้อง + MediaPipe (ไม่เปิดหน้าต่าง)
  ai/.venv/Scripts/python ai/posture_ai.py collect good       อัดท่านั่งดี (ค่าเริ่มต้น 60 วินาที)
  ai/.venv/Scripts/python ai/posture_ai.py collect bad        อัดท่านั่งไม่ดี: ก้ม / ห่อไหล่ / เอียง / ชิดจอ
  ai/.venv/Scripts/python ai/posture_ai.py train              เทรน + รายงาน accuracy -> ai/model/
  ai/.venv/Scripts/python ai/posture_ai.py run                ใช้งานจริง + ส่งขึ้น NETPIE (ถ้ามี ai/.env)
ตัวเลือก: --cam N (index กล้อง), --secs N (เวลาอัด), --no-window
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
import urllib.request
from collections import deque
from pathlib import Path

os.environ.setdefault("GLOG_minloglevel", "2")  # ปิด log INFO ของ MediaPipe/LiteRT
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")

import cv2  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent
DATA_CSV = ROOT / "data" / "posture.csv"
MODEL_DIR = ROOT / "model"
MODEL_PATH = MODEL_DIR / "posture_model.joblib"
METRICS_PATH = MODEL_DIR / "metrics.json"
POSE_TASK = MODEL_DIR / "pose_landmarker_lite.task"
POSE_URLS = [
    "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task",
    "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task",
]
ENV_PATH = ROOT / ".env"

# ---------------- landmark index ของ MediaPipe Pose (33 จุด) ----------------
NOSE, L_EYE, R_EYE, L_EAR, R_EAR, MOUTH_L, MOUTH_R, L_SH, R_SH = 0, 2, 5, 7, 8, 9, 10, 11, 12
UPPER_BODY_LINES = [(L_EAR, L_EYE), (L_EYE, NOSE), (NOSE, R_EYE), (R_EYE, R_EAR), (MOUTH_L, MOUTH_R), (L_SH, R_SH)]
MIN_VIS = 0.5

FEATURES = [
    "nose_x", "nose_y", "leye_x", "leye_y", "reye_x", "reye_y",
    "lear_x", "lear_y", "rear_x", "rear_y", "mouth_x", "mouth_y",  # ตำแหน่งเทียบจุดกึ่งกลางไหล่ / ความกว้างไหล่
    "shoulder_tilt", "ear_tilt", "eye_tilt",                       # องศา: ไหล่ / หัว เอียง
    "shoulder_w_img", "eye_dist_rel",                              # ใกล้กล้อง, ขนาดหน้าเทียบไหล่
    "nose_z_rel", "ear_z_rel",                                     # ความลึก: หัวยื่นมาข้างหน้า
]

# การตัดสินผลแบบหน่วงเวลา (กันผลกระพริบ)
SMOOTH_FRAMES = 15          # เฉลี่ยความน่าจะเป็นย้อนหลัง ~0.5-1 วินาที
BAD_ON, BAD_OFF = 0.60, 0.40
AWAY_AFTER_S = 2.0          # ไม่เจอคนเกินนี้ = ไม่อยู่หน้าจอ
PUBLISH_EVERY_S = 1.0


# ======================================================================
def open_camera(idx: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW) if sys.platform == "win32" else cv2.VideoCapture(idx)
    if not cap.isOpened():
        sys.exit(f"[CAM] cannot open camera {idx} -> run: python ai/posture_ai.py cams")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap


def ensure_pose_model() -> Path:
    if POSE_TASK.exists():
        return POSE_TASK
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    for url in POSE_URLS:
        try:
            print(f"[POSE] downloading model {url}")
            urllib.request.urlretrieve(url, POSE_TASK)
            return POSE_TASK
        except Exception as e:  # noqa: BLE001
            print(f"[POSE] download failed: {e}")
    sys.exit("[POSE] cannot download pose_landmarker_lite.task (check internet)")


class PoseDetector:
    """MediaPipe Pose Landmarker (Tasks API) โหมด VIDEO"""

    def __init__(self):
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision

        self.mp = mp
        opts = vision.PoseLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(ensure_pose_model())),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.lm = vision.PoseLandmarker.create_from_options(opts)
        self.last_ts = 0

    def detect(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
        ts = max(int(time.monotonic() * 1000), self.last_ts + 1)  # timestamp ต้องเพิ่มขึ้นเสมอ
        self.last_ts = ts
        res = self.lm.detect_for_video(img, ts)
        return res.pose_landmarks[0] if res.pose_landmarks else None

    def close(self):
        self.lm.close()


# ---------------- feature ----------------
def features_from(lms, w_img: int, h_img: int):
    """แปลง landmark -> feature vector; None ถ้าเห็นไหล่/จมูกไม่ชัด"""
    if lms is None or min(lms[i].visibility for i in (NOSE, L_SH, R_SH)) < MIN_VIS:
        return None
    p = {i: np.array([lms[i].x * w_img, lms[i].y * h_img]) for i in (NOSE, L_EYE, R_EYE, L_EAR, R_EAR, MOUTH_L, MOUTH_R, L_SH, R_SH)}
    z = {i: lms[i].z * w_img for i in (NOSE, L_EAR, R_EAR, L_SH, R_SH)}  # z ของ MediaPipe ใช้สเกลเดียวกับ x
    c = (p[L_SH] + p[R_SH]) / 2
    sw = float(np.linalg.norm(p[L_SH] - p[R_SH]))
    if sw < 1e-3:
        return None

    def rel(v):
        return (v - c) / sw

    def tilt(a, b):  # องศาของเส้น a->b เทียบแนวนอน (เอียงซ้าย/ขวา)
        d = p[a] - p[b]
        return math.degrees(math.atan2(d[1], abs(d[0]) + 1e-6)) * (1 if d[0] >= 0 else -1)

    mouth = (p[MOUTH_L] + p[MOUTH_R]) / 2
    sh_z = (z[L_SH] + z[R_SH]) / 2
    f = []
    for v in (p[NOSE], p[L_EYE], p[R_EYE], p[L_EAR], p[R_EAR], mouth):
        f.extend(rel(v))
    f += [tilt(L_SH, R_SH), tilt(L_EAR, R_EAR), tilt(L_EYE, R_EYE)]
    f += [sw / w_img, float(np.linalg.norm(p[L_EYE] - p[R_EYE])) / sw]
    f += [(z[NOSE] - sh_z) / sw, ((z[L_EAR] + z[R_EAR]) / 2 - sh_z) / sw]
    return np.array(f, dtype=np.float32)


def draw_pose(img, lms):
    if lms is None:
        return
    h, w = img.shape[:2]
    pts = {i: (int(lms[i].x * w), int(lms[i].y * h)) for i in range(13)}
    for a, b in UPPER_BODY_LINES:
        cv2.line(img, pts[a], pts[b], (255, 200, 0), 2)
    for i in pts:
        cv2.circle(img, pts[i], 4, (0, 255, 255), -1)


def put(img, text, y, color=(255, 255, 255), scale=0.7):
    cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def show(win: bool, img) -> bool:
    """แสดงภาพ (กลับซ้ายขวาเหมือนกระจก) คืน False ถ้ากด q / ESC"""
    if not win:
        return True
    cv2.imshow("ParallaxSense AI", img)
    return (cv2.waitKey(1) & 0xFF) not in (ord("q"), 27)


class Latency:
    """เก็บเวลาประมวลผลต่อเฟรม: mean / min / max (ใช้ในรายงาน metrics)"""

    def __init__(self):
        self.v = []

    def add(self, ms):
        self.v.append(ms)

    def summary(self):
        if not self.v:
            return "n/a"
        a = np.array(self.v)
        return f"mean={a.mean():.1f} min={a.min():.1f} max={a.max():.1f} p95={np.percentile(a, 95):.1f} ms (n={len(a)})"


# ======================================================================
def cmd_cams(_args):
    for i in range(5):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW) if sys.platform == "win32" else cv2.VideoCapture(i)
        ok = cap.isOpened() and cap.read()[0]
        print(f"camera {i}: {'OK ' + str(int(cap.get(3))) + 'x' + str(int(cap.get(4))) if ok else '-'}")
        cap.release()


def cmd_check(args):
    cap, det, lat = open_camera(args.cam), PoseDetector(), Latency()
    seen, n = 0, 60
    t0 = time.time()
    for _ in range(n):
        ok, frame = cap.read()
        if not ok:
            sys.exit("[CAM] read failed")
        t = time.perf_counter()
        lms = det.detect(frame)
        f = features_from(lms, frame.shape[1], frame.shape[0])
        lat.add((time.perf_counter() - t) * 1000)
        seen += f is not None
    fps = n / (time.time() - t0)
    print(f"[CHECK] frames={n} person(upper body) detected={seen}/{n} fps={fps:.1f}")
    print(f"[CHECK] pose+feature latency {lat.summary()}")
    if seen < n // 2:
        print("[CHECK] hint: sit so head and both shoulders are in view, light from the front")
    cap.release()
    det.close()


def cmd_collect(args):
    label = args.label
    DATA_CSV.parent.mkdir(parents=True, exist_ok=True)
    new_file = not DATA_CSV.exists()
    session = time.strftime("%Y%m%d-%H%M%S") + "-" + label
    cap, det = open_camera(args.cam), PoseDetector()
    win = not args.no_window
    print(f"[COLLECT] label={label} for {args.secs}s, session={session}")
    if label == "bad":
        print("[COLLECT] vary it: lean head down, hunch shoulders, lean left/right, move close to screen")
    else:
        print("[COLLECT] sit upright, small natural moves (typing, looking around the screen) are good")

    # นับถอยหลังให้เตรียมท่า
    t_end = time.time() + 3
    while time.time() < t_end:
        ok, frame = cap.read()
        img = cv2.flip(frame, 1)
        put(img, f"GET READY: {label.upper()}  {t_end - time.time():.0f}", 40, (0, 255, 255), 1.0)
        if not show(win, img):
            return

    rows, t0 = 0, time.time()
    with DATA_CSV.open("a", newline="") as fh:
        wr = csv.writer(fh)
        if new_file:
            wr.writerow(["t", "session", "label"] + FEATURES)
        while time.time() - t0 < args.secs:
            ok, frame = cap.read()
            if not ok:
                break
            lms = det.detect(frame)
            f = features_from(lms, frame.shape[1], frame.shape[0])
            if f is not None:
                wr.writerow([f"{time.time():.3f}", session, label] + [f"{x:.5f}" for x in f])
                rows += 1
            img = frame.copy()
            draw_pose(img, lms)
            img = cv2.flip(img, 1)  # แสดงแบบกระจก (คำนวณ feature จากภาพจริงไม่กลับด้าน)
            color = (0, 200, 0) if label == "good" else (0, 0, 255)
            put(img, f"REC {label.upper()}  {args.secs - (time.time() - t0):.0f}s  rows={rows}", 35, color)
            if f is None:
                put(img, "no upper body in view", 70, (0, 0, 255))
            if not show(win, img):
                break
    cap.release()
    det.close()
    cv2.destroyAllWindows()
    print(f"[COLLECT] saved {rows} rows -> {DATA_CSV}")


def load_data():
    if not DATA_CSV.exists():
        sys.exit("[TRAIN] no data yet -> run: collect good / collect bad")
    with DATA_CSV.open() as fh:
        rows = list(csv.DictReader(fh))
    X = np.array([[float(r[k]) for k in FEATURES] for r in rows], dtype=np.float32)
    y = np.array([1 if r["label"] == "bad" else 0 for r in rows])
    sess = np.array([r["session"] for r in rows])
    return X, y, sess


def build_model():
    from sklearn.ensemble import RandomForestClassifier
    from sklearn.pipeline import make_pipeline
    from sklearn.preprocessing import StandardScaler

    return make_pipeline(
        StandardScaler(),
        RandomForestClassifier(n_estimators=150, min_samples_leaf=3, class_weight="balanced", random_state=0, n_jobs=-1),
    )


def cmd_train(_args):
    import joblib
    from sklearn.metrics import accuracy_score, classification_report, confusion_matrix

    X, y, sess = load_data()
    if len(set(y)) < 2:
        sys.exit("[TRAIN] need both 'good' and 'bad' data")
    # เฟรมที่ติดกันหน้าตาเหมือนกันมาก ถ้าสุ่มแบ่งจะได้ accuracy สูงเกินจริง
    # จึงแบ่งตามเวลา: 20% ท้ายของแต่ละ session เป็นชุดทดสอบ
    test = np.zeros(len(y), dtype=bool)
    for s in np.unique(sess):
        idx = np.where(sess == s)[0]
        test[idx[int(len(idx) * 0.8):]] = True
    model = build_model()
    model.fit(X[~test], y[~test])
    pred = model.predict(X[test])
    acc = accuracy_score(y[test], pred)
    cm = confusion_matrix(y[test], pred, labels=[0, 1])
    print(f"[TRAIN] rows: good={int((y == 0).sum())} bad={int((y == 1).sum())} | train={int((~test).sum())} test={int(test.sum())}")
    print(f"[TRAIN] test accuracy = {acc * 100:.1f}%  (held-out last 20% of every session)")
    print("[TRAIN] confusion matrix [rows=true good,bad | cols=pred good,bad]")
    print(cm)
    print(classification_report(y[test], pred, labels=[0, 1], target_names=["good", "bad"], digits=3, zero_division=0))
    rf = model[-1]
    top = sorted(zip(FEATURES, rf.feature_importances_), key=lambda t: -t[1])[:6]
    print("[TRAIN] most useful features: " + ", ".join(f"{k}={v:.2f}" for k, v in top))

    final = build_model().fit(X, y)  # โมเดลที่ใช้จริงเทรนด้วยข้อมูลทั้งหมด
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    joblib.dump({"model": final, "features": FEATURES, "trained_at": time.strftime("%Y-%m-%d %H:%M:%S")}, MODEL_PATH)
    METRICS_PATH.write_text(json.dumps({
        "test_accuracy": round(float(acc), 4),
        "confusion_matrix": cm.tolist(),
        "rows_good": int((y == 0).sum()), "rows_bad": int((y == 1).sum()),
        "sessions": int(len(np.unique(sess))),
        "top_features": [[k, round(float(v), 3)] for k, v in top],
        "trained_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }, indent=2))
    print(f"[TRAIN] saved {MODEL_PATH} and {METRICS_PATH}")


# ---------------- NETPIE ----------------
def load_env():
    env = {}
    if ENV_PATH.exists():
        for line in ENV_PATH.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                env[k.strip()] = v.strip().strip('"').strip("'")
    return env


class Netpie:
    def __init__(self, env):
        import paho.mqtt.client as mqtt

        self.ok = False
        cid = env.get("NETPIE_CLIENT_ID")
        if not cid:
            print("[NETPIE] ai/.env not found -> offline mode (copy ai/.env.example to ai/.env)")
            self.c = None
            return
        try:
            self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid)  # paho-mqtt 2.x
        except AttributeError:
            self.c = mqtt.Client(client_id=cid)  # paho-mqtt 1.x
        self.c.username_pw_set(env.get("NETPIE_TOKEN", ""), env.get("NETPIE_SECRET", ""))
        if env.get("NETPIE_TLS", "0") == "1":
            self.c.tls_set()
        self.c.on_connect = lambda c, u, f, rc, *a: self._on_connect(rc)
        self.c.on_disconnect = lambda *a: setattr(self, "ok", False)
        host, port = env.get("NETPIE_HOST", "mqtt.netpie.io"), int(env.get("NETPIE_PORT", "1883"))
        self.c.reconnect_delay_set(1, 10)
        self.c.connect_async(host, port, keepalive=30)
        self.c.loop_start()
        print(f"[NETPIE] connecting to {host}:{port} ...")

    def _on_connect(self, rc):
        self.ok = (rc == 0) if isinstance(rc, int) else not rc.is_failure
        print(f"[NETPIE] {'connected' if self.ok else 'connect FAILED rc=' + str(rc)}")

    def publish(self, bad, prob, present, fps):
        if not (self.c and self.ok):
            return False
        msg = {"bad": int(bad), "p": round(prob, 3), "present": int(present)}
        self.c.publish("@msg/ai", json.dumps(msg, separators=(",", ":")))  # gateway subscribe -> ยกธง
        shadow = {"data": {"ai_bad": int(bad), "ai_conf": round(prob * 100), "ai_present": int(present), "ai_fps": round(fps, 1)}}
        self.c.publish("@shadow/data/update", json.dumps(shadow, separators=(",", ":")))
        return True

    def close(self):
        if self.c:
            self.c.loop_stop()
            self.c.disconnect()


def cmd_run(args):
    import joblib

    if not MODEL_PATH.exists():
        sys.exit("[RUN] no model yet -> run: collect good, collect bad, train")
    # joblib = pickle: โหลดได้เฉพาะไฟล์ที่ cmd_train สร้างเองบนเครื่องนี้ ห้ามใช้ไฟล์โมเดลที่ได้มาจากที่อื่น
    bundle = joblib.load(MODEL_PATH)
    if bundle["features"] != FEATURES:
        sys.exit("[RUN] model was trained with different features -> run train again")
    model = bundle["model"]
    model[-1].set_params(n_jobs=1)  # ทำนายทีละเฟรม: ใช้หลาย thread กลับช้ากว่า (overhead ~40 ms)
    print(f"[RUN] model trained at {bundle['trained_at']}  (q / ESC to quit)")

    cap, det, net = open_camera(args.cam), PoseDetector(), Netpie(load_env())
    win = not args.no_window
    lat, hist = Latency(), deque(maxlen=SMOOTH_FRAMES)
    bad, present, last_seen, last_pub, last_log = False, False, 0.0, 0.0, time.time()
    fps, t_prev, prob = 0.0, time.time(), 0.0
    t_start = time.time()
    try:
        while not (args.max_secs and time.time() - t_start >= args.max_secs):
            ok, frame = cap.read()
            if not ok:
                print("[CAM] read failed")
                break
            t = time.perf_counter()
            lms = det.detect(frame)
            f = features_from(lms, frame.shape[1], frame.shape[0])
            now = time.time()
            if f is not None:
                hist.append(float(model.predict_proba(f.reshape(1, -1))[0][1]))
                last_seen = now
            lat.add((time.perf_counter() - t) * 1000)
            present = now - last_seen < AWAY_AFTER_S
            if not present:
                hist.clear()
            prob = float(np.mean(hist)) if hist else 0.0
            if present and not bad and prob >= BAD_ON:
                bad = True
            elif bad and (prob <= BAD_OFF or not present):
                bad = False

            dt = now - t_prev
            t_prev = now
            fps = 0.9 * fps + 0.1 * (1 / dt) if dt > 0 else fps

            if now - last_pub >= PUBLISH_EVERY_S:
                last_pub = now
                net.publish(bad, prob, present, fps)
            if now - last_log >= 10:
                last_log = now
                print(f"[RUN] {'BAD' if bad else 'good'} p_bad={prob:.2f} present={int(present)} fps={fps:.1f} | latency {lat.summary()} | netpie={'on' if net.ok else 'off'}")

            if win:
                img = frame.copy()
                draw_pose(img, lms)
                img = cv2.flip(img, 1)
                if not present:
                    put(img, "AWAY", 40, (200, 200, 200), 1.1)
                elif bad:
                    put(img, f"BAD POSTURE  {prob * 100:.0f}%", 40, (0, 0, 255), 1.1)
                else:
                    put(img, f"GOOD  {(1 - prob) * 100:.0f}%", 40, (0, 200, 0), 1.1)
                put(img, f"fps {fps:.1f} | {lat.v[-1]:.0f} ms | NETPIE {'on' if net.ok else 'off'}", img.shape[0] - 15, scale=0.55)
                if not show(win, img):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[RUN] latency per frame (pose + model): {lat.summary()}")
        cap.release()
        det.close()
        net.close()
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description="ParallaxSense posture AI (USB webcam + MediaPipe + RandomForest)")
    ap.add_argument("cmd", choices=["cams", "check", "collect", "train", "run"])
    ap.add_argument("label", nargs="?", choices=["good", "bad"], help="for collect")
    ap.add_argument("--cam", type=int, default=0, help="camera index (see: cams)")
    ap.add_argument("--secs", type=int, default=60, help="collect duration in seconds")
    ap.add_argument("--no-window", action="store_true", help="do not open a preview window")
    ap.add_argument("--max-secs", type=int, default=0, help="run: stop after N seconds (0 = until q/ESC)")
    args = ap.parse_args()
    if args.cmd == "collect" and not args.label:
        ap.error("collect needs a label: good or bad")
    {"cams": cmd_cams, "check": cmd_check, "collect": cmd_collect, "train": cmd_train, "run": cmd_run}[args.cmd](args)


if __name__ == "__main__":
    main()
