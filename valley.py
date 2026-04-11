
import numpy as np
import cv2
import depthai as dai
import time
from datetime import timedelta

FPS = 10.0

RGB_SOCKET = dai.CameraBoardSocket.CAM_A
LEFT_SOCKET = dai.CameraBoardSocket.CAM_B
RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C


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
# PIPELINE (FIXED)
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

# LINKS (same as working example style)
rgbOut.link(sync.inputs["rgb"])
leftOut.link(stereo.left)
rightOut.link(stereo.right)

stereo.depth.link(sync.inputs["depth_aligned"])
rgbOut.link(stereo.inputAlignTo)

# IMPORTANT: THIS is what your working code uses
queue = sync.out.createOutputQueue()


# ---------------------------
# RUN
# ---------------------------
fps_counter = FPSCounter()

with pipeline:
    pipeline.start()

    print("[INFO] Camera started")

    while True:
        msg = queue.get()

        fps_counter.tick()

        rgb = msg["rgb"].getCvFrame()
        depth = msg["depth_aligned"].getFrame()

        depth_color = colorize_depth(depth)

        blended = cv2.addWeighted(rgb, 0.4, depth_color, 0.6, 0)

        cv2.putText(
            blended,
            f"FPS: {fps_counter.get():.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
        )

        cv2.imshow("Stereo Depth", blended)

        if cv2.waitKey(1) == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    run()
