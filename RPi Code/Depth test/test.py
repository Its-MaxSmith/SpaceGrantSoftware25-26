import depthai as dai
import cv2

pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setFps(30)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)

xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

with dai.Device(pipeline) as device:
    q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    while True:
        rgb_frame = q_rgb.get().getCvFrame()  # This is normal color
        cv2.imshow("RGB Camera", rgb_frame)
        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
