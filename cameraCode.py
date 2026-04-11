
import numpy as np
import cv2
import depthai as dai
import time
import serial
from datetime import timedelta

FPS = 10.0

RGB_SOCKET = dai.CameraBoardSocket.CAM_A
LEFT_SOCKET = dai.CameraBoardSocket.CAM_B
RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C

ser = serial.Serial(
    port="COM3",      # Change this to the correct port for your system (e.g., /dev/ttyACM0 on Linux)
    baudrate=9600,
    timeout=1
)

time.sleep(2)

# ---------------------------
# FPS COUNTER
# ---------------------------
class FPSCounter:
    def __init__(self):
        self.times = []

    def tick(self):
        now = time.time()
        self.times.append(now)
        self.times = self.times[-10:]

    def get(self):
        if len(self.times) < 2:
            return 0
        return (len(self.times) - 1) / (self.times[-1] - self.times[0])


# ---------------------------
# DEPTH COLORIZER
# ---------------------------
def colorize_depth(depth):
    mask = depth == 0

    valid = depth[depth != 0]
    if len(valid) == 0:
        return np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)

    mn = np.percentile(valid, 3)
    mx = np.percentile(valid, 95)

    log_depth = np.zeros_like(depth, dtype=np.float32)
    np.log(depth, where=depth != 0, out=log_depth)

    log_min = np.log(mn)
    log_max = np.log(mx)

    log_depth = np.clip(log_depth, log_min, log_max)

    depth_color = np.interp(log_depth, (log_min, log_max), (0, 255)).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_color, cv2.COLORMAP_JET)

    depth_color[mask] = 0
    return depth_color


# ---------------------------
# ROBUST DEPTH SAMPLING
# ---------------------------
def get_stable_depth(depth, x, y, window=20):
    h, w = depth.shape

    x1 = max(0, x - window)
    x2 = min(w, x + window)
    y1 = max(0, y - window)
    y2 = min(h, y + window)

    region = depth[y1:y2, x1:x2]
    valid = region[region > 0]

    if len(valid) == 0:
        return 0

    return np.median(valid)


# ---------------------------
# ANGLE + DISTANCE
# ---------------------------
def compute_angle_and_distance(depth, x, y):
    h, w = depth.shape

    d = get_stable_depth(depth, x, y)
    dist_m = d / 1000.0

    fx = w * 0.8
    cx = w / 2

    angle_rad = np.arctan((x - cx) / fx)
    angle_deg = np.degrees(angle_rad)

    return dist_m, angle_deg


# ---------------------------
# PATH SCANNING (simple)
# ---------------------------
def find_best_path(depth):
    h, w = depth.shape
    scan_y = int(h * 0.6)

    samples = []
    for x in range(0, w, 20):
        d = get_stable_depth(depth, x, scan_y)
        if d > 0:
            samples.append((x, d))

    if not samples:
        return None

    # pick farthest point = safest direction
    best_x = max(samples, key=lambda s: s[1])[0]

    return best_x, scan_y


# ---------------------------
# MOUSE INPUT
# ---------------------------
target_point = None

def mouse_callback(event, x, y, flags, param):
    global target_point
    if event == cv2.EVENT_LBUTTONDOWN:
        target_point = (x, y)


# ---------------------------
# PIPELINE (UNCHANGED)
# ---------------------------
pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.Camera).build(RGB_SOCKET)
camLeft = pipeline.create(dai.node.Camera).build(LEFT_SOCKET)
camRight = pipeline.create(dai.node.Camera).build(RIGHT_SOCKET)

stereo = pipeline.create(dai.node.StereoDepth)
sync = pipeline.create(dai.node.Sync)

stereo.setExtendedDisparity(True)
stereo.setLeftRightCheck(True)

sync.setSyncThreshold(timedelta(seconds=1 / (2 * FPS)))

rgbOut = camRgb.requestOutput(size=(1280, 960), fps=FPS, enableUndistortion=True)
leftOut = camLeft.requestOutput(size=(640, 400), fps=FPS)
rightOut = camRight.requestOutput(size=(640, 400), fps=FPS)

rgbOut.link(sync.inputs["rgb"])
leftOut.link(stereo.left)
rightOut.link(stereo.right)

stereo.depth.link(sync.inputs["depth_aligned"])
rgbOut.link(stereo.inputAlignTo)

queue = sync.out.createOutputQueue()


# ---------------------------
# RUN
# ---------------------------
fps_counter = FPSCounter()
compute_angle_and_distance
with pipeline:
    pipeline.start()

    print("[INFO] Camera started")

    cv2.namedWindow("Stereo Depth")
    cv2.setMouseCallback("Stereo Depth", mouse_callback)

    while True:
        msg = queue.get()
        fps_counter.tick()

        rgb = msg["rgb"].getCvFrame()
        depth = msg["depth_aligned"].getFrame()

        depth_color = colorize_depth(depth)
        blended = cv2.addWeighted(rgb, 0.4, depth_color, 0.6, 0)

        h, w, _ = rgb.shape

        # -------------------------
        # TARGET (CLICK OR CENTER)
        # ---------------------------
        if target_point is not None:
            tx, ty = target_point
        else:
            tx, ty = w // 2, h // 2

        dist, angle = compute_angle_and_distance(depth, tx, ty)

        cv2.circle(blended, (tx, ty), 6, (0, 255, 0), -1)

        # ---------------------------
        # BEST PATH
        # ---------------------------
        best = find_best_path(depth)
        if best:
            bx, by = best
            best_dist, best_angle = compute_angle_and_distance(depth, bx, by)
            
            # **Format the message to send over serial**
            message = f"{best_angle:.2f},{best_dist:.2f}\n"  # **Formatted as CSV**
            ser.write(message.encode("utf-8"))  # **Send via serial connection**


            cv2.circle(blended, (bx, by), 8, (0, 255, 255), -1)
            cv2.putText(
                blended,
                "Best Path",
                (bx - 40, by - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )

        # ---------------------------
        # TEXT OVERLAY
        # ---------------------------
        cv2.putText(
            blended,
            f"FPS: {fps_counter.get():.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            blended,
            f"Dist: {dist:.2f} m",
            (10, 70),
            0.7,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            blended,
            f"Angle: {angle:.1f} deg",
            (10, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        cv2.imshow("Stereo Depth", blended)

        if cv2.waitKey(1) == ord("q"):
            break

    cv2.destroyAllWindows()
