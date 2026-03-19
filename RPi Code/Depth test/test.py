# Test py

import cv2
import depthai as dai
import numpy as np
import serial
import serial.tools.list_ports
import math
import argparse

############################################
# SETTINGS
############################################

OBSTACLE_DISTANCE_MM = 150
FORWARD_TARGET_M = 1.0
MAX_STEERING_RAD = math.radians(45)

############################################
# SERIAL CONNECTION
############################################

def connect_to_pico():
    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        if "ACM" in port.device or "USB" in port.device:
            try:
                ser = serial.Serial(port.device,115200,timeout=1)
                print(f"[INFO] Connected to Pico: {port.device}")
                return ser
            except:
                pass

    print("[WARNING] Pico not detected")
    return None

############################################
# VALLEY DETECTION
############################################

def find_valley(depthFrame):

    depth = depthFrame.astype(np.float32)
    obstacle_mask = depth < OBSTACLE_DISTANCE_MM

    h,w = depth.shape

    roi = obstacle_mask[int(h*0.5):,:]

    free_space = np.sum(~roi,axis=0)

    free_space = cv2.GaussianBlur(free_space,(31,1),0)

    valley_index = int(np.argmax(free_space))

    center = w//2

    theta = (valley_index-center)/center * MAX_STEERING_RAD

    x = FORWARD_TARGET_M * math.cos(theta)
    y = FORWARD_TARGET_M * math.sin(theta)

    return valley_index,x,y,theta,free_space

############################################
# WALL DETECTION
############################################

def detect_walls(depthFrame):
    if True:
        return 0,0,0
    depth = depthFrame.astype(np.float32)
    obstacle_mask = depth < OBSTACLE_DISTANCE_MM

    h,w = depth.shape

    roi = obstacle_mask[int(h*0.5):,:]

    col_sum = np.sum(roi,axis=0)

    threshold = roi.shape[0] * 0.1

    left_wall = None
    right_wall = None

    for i in range(w):
        if col_sum[i] > threshold:
            left_wall = i
            break

    for i in range(w-1,-1,-1):
        if col_sum[i] > threshold:
            right_wall = i
            break

    if left_wall is None:
        left_wall = 0

    if right_wall is None:
        right_wall = w-1

    centerline = (left_wall + right_wall)//2

    return left_wall,right_wall,centerline

############################################
# SEND TARGET TO PICO
############################################

def send_target(ser,x,y,theta):

    if ser is None:
        return ser

    try:
        msg = f"{x:.2f},{y:.2f},{theta:.3f}\n"
        ser.write(msg.encode())

    except serial.SerialException:
        print("[WARNING] Serial lost")
        ser = None

    return ser

############################################
# VISUALIZATION
############################################

def visualize(depthFrame,valley_index,free_space,x,y,theta,left_wall,right_wall,centerline):

    vis = cv2.normalize(depthFrame,None,0,255,cv2.NORM_MINMAX).astype(np.uint8)
    vis = cv2.applyColorMap(vis,cv2.COLORMAP_JET)

    h,w = vis.shape[:2]

    center = w//2

    # Valley direction
    cv2.line(vis,(valley_index,0),(valley_index,h),(0,255,0),2)

    # Walls
    cv2.line(vis,(left_wall,0),(left_wall,h),(255,0,0),2)
    cv2.line(vis,(right_wall,0),(right_wall,h),(0,0,255),2)

    # Centerline between walls
    cv2.line(vis,(centerline,0),(centerline,h),(0,255,255),2)

    # Camera center
    cv2.line(vis,(center,0),(center,h),(255,255,255),1)

    # Bounding boxes
    box_width = 40

    cv2.rectangle(vis,
                  (left_wall-box_width,int(h*0.5)),
                  (left_wall+box_width,h),
                  (255,0,0),2)

    cv2.rectangle(vis,
                  (right_wall-box_width,int(h*0.5)),
                  (right_wall+box_width,h),
                  (0,0,255),2)

    # Navigation text
    cv2.putText(vis,f"x = {x:.2f} m",(20,30),
                cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),2)

    cv2.putText(vis,f"y = {y:.2f} m",(20,60),
                cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),2)

    cv2.putText(vis,f"theta = {math.degrees(theta):.1f} deg",(20,90),
                cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),2)

    ############################################
    # LEGEND / KEY
    ############################################

    legend_y = 140

    cv2.putText(vis,"Legend:",(20,legend_y),
                cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),2)

    legend_y += 25
    cv2.putText(vis,"Green  - Valley Direction",(20,legend_y),
                cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)

    legend_y += 25
    cv2.putText(vis,"Blue   - Left Wall",(20,legend_y),
                cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)

    legend_y += 25
    cv2.putText(vis,"Red    - Right Wall",(20,legend_y),
                cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)

    legend_y += 25
    cv2.putText(vis,"Yellow - Centerline",(20,legend_y),
                cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,255),2)

    legend_y += 25
    cv2.putText(vis,"White  - Camera Center",(20,legend_y),
                cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)

    cv2.imshow("Depth Nav",vis)

############################################
# MAIN LOOP
############################################

def main(video_mode):

    cv2.namedWindow("Depth Nav", cv2.WINDOW_NORMAL)

    ser = connect_to_pico()

    device = dai.Device()

    with dai.Pipeline(device) as pipeline:

        outputQueues = {}

        sockets = device.getConnectedCameras()

        for socket in sockets:

            cam = pipeline.create(dai.node.Camera).build(socket)

            outputQueues[str(socket)] = cam.requestFullResolutionOutput().createOutputQueue()

        pipeline.start()

        print("[INFO] Navigation running")

        try:

            while pipeline.isRunning():

                for name,queue in outputQueues.items():

                    framePacket = queue.tryGet()

                    if framePacket is None:
                        continue

                    depthFrame = framePacket.getFrame()

                    valley_index,x,y,theta,free_space = find_valley(depthFrame)

                    left_wall,right_wall,centerline = detect_walls(depthFrame)

                    ser = send_target(ser,x,y,theta)

                    if video_mode:
                        visualize(depthFrame,valley_index,free_space,
                                  x,y,theta,left_wall,right_wall,centerline)

                if video_mode and cv2.waitKey(1) == ord('q'):
                    break

        except KeyboardInterrupt:
            print("[INFO] Keyboard interrupt")

        finally:

            if ser:
                ser.close()

            cv2.destroyAllWindows()

            print("[INFO] Shutting down")

############################################
# ENTRY
############################################

if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("--video",action="store_true",
                        help="Run with visualization")

    args = parser.parse_args()

    main(args.video)