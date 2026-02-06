# Depth test program for OAK-D Lite
# Last author: Max Smith
#
# Tests the depth sensor

import depthai as dai
import numpy as np
import cv2
import serial

# -----------------------
# Serial Setup (to Pico)
# -----------------------
SERIAL_PORT = 'COM3'  # Change to your Pico port
BAUD_RATE = 115200
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except Exception as e:
    print(f"Error connecting to serial: {e}")
    ser = None

# -----------------------
# DepthAI Pipeline Setup
# -----------------------
pipeline = dai.Pipeline()

# RGB camera
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setFps(30)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

# Mono cameras for depth
mono_left = pipeline.create(dai.node.MonoCamera)
mono_right = pipeline.create(dai.node.MonoCamera)
mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

# Stereo depth
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setConfidenceThreshold(200)
mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)

xout_depth = pipeline.create(dai.node.XLinkOut)
xout_depth.setStreamName("depth")
stereo.depth.link(xout_depth.input)

# Optional: send left and right mono images for debugging
xout_mono_left = pipeline.create(dai.node.XLinkOut)
xout_mono_left.setStreamName("mono_left")
mono_left.out.link(xout_mono_left.input)

xout_mono_right = pipeline.create(dai.node.XLinkOut)
xout_mono_right.setStreamName("mono_right")
mono_right.out.link(xout_mono_right.input)

# -----------------------
# Start pipeline
# -----------------------
with dai.Device(pipeline) as device:
    q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    q_mono_left = device.getOutputQueue(name="mono_left", maxSize=4, blocking=False)
    q_mono_right = device.getOutputQueue(name="mono_right", maxSize=4, blocking=False)

    while True:
        rgb_frame = q_rgb.get().getCvFrame()
        depth_frame = q_depth.get().getCvFrame()
        mono_left_frame = q_mono_left.get().getCvFrame()
        mono_right_frame = q_mono_right.get().getCvFrame()

        # Normalize depth for display
        depth_vis = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = np.uint8(depth_vis)
        depth_vis_color = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)
        depth_vis_color = cv2.resize(depth_vis_color, (rgb_frame.shape[1], rgb_frame.shape[0]))

        # Resize mono cameras to smaller thumbnails
        thumb_h, thumb_w = 120, 160
        mono_left_thumb = cv2.resize(cv2.cvtColor(mono_left_frame, cv2.COLOR_GRAY2BGR), (thumb_w, thumb_h))
        mono_right_thumb = cv2.resize(cv2.cvtColor(mono_right_frame, cv2.COLOR_GRAY2BGR), (thumb_w, thumb_h))

        # Combine RGB + Depth vertically
        main_view = cv2.vconcat([rgb_frame, depth_vis_color])

        # Add mono thumbnails on top-left
        top_row = cv2.hconcat([mono_left_thumb, mono_right_thumb, np.zeros((thumb_h, main_view.shape[1]-2*thumb_w, 3), dtype=np.uint8)])
        combined = cv2.vconcat([top_row, main_view])

        # -----------------------
        # Obstacle avoidance logic
        # -----------------------
        h, w = depth_frame.shape
        left_region = depth_frame[h//3:h*2//3, :w//3]
        center_region = depth_frame[h//3:h*2//3, w//3:w*2//3]
        right_region = depth_frame[h//3:h*2//3, w*2//3:]

        min_left = np.min(left_region)
        min_center = np.min(center_region)
        min_right = np.min(right_region)

        threshold = 400  # mm

        if min_center > threshold:
            command = "FORWARD\n"
        elif min_left > min_right:
            command = "LEFT\n"
        elif min_right > min_left:
            command = "RIGHT\n"
        else:
            command = "STOP\n"

        if ser:
            ser.write(command.encode())
            print(f"Command: {command.strip()} | Left: {min_left} | Center: {min_center} | Right: {min_right}")

        # Show full multi-view
        cv2.imshow("OAK-D Lite Full View", combined)

        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
