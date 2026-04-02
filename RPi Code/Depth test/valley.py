#!/usr/bin/env python3

import numpy as np
import cv2
import depthai as dai
import time

class FPSCounter:
    def __init__(self):
        self.frameTimes = []

    def tick(self):
        now = time.time()
        self.frameTimes.append(now)
        self.frameTimes = self.frameTimes[-10:]

    def getFps(self):
        if len(self.frameTimes) <= 1:
            return 0
        return (len(self.frameTimes) - 1) / (self.frameTimes[-1] - self.frameTimes[0])

class StereoApp:
    def __init__(self, fps=10.0):
        self.fps = fps
        # Using the exact same socket constants from your original
        self.LEFT_SOCKET = dai.CameraBoardSocket.CAM_B
        self.RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C
        
        self.pipeline = dai.Pipeline()
        self.fpsCounter = FPSCounter()
        self._build_pipeline()

    def _build_pipeline(self):
        # 1. Create nodes using the most standard syntax
        self.camLeft = self.pipeline.create(dai.node.MonoCamera)
        self.camRight = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        self.xoutDepth = self.pipeline.create(dai.node.XLinkOut)

        # 2. Configure Nodes
        self.camLeft.setBoardSocket(self.LEFT_SOCKET)
        self.camRight.setBoardSocket(self.RIGHT_SOCKET)
        self.camLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.camRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.camLeft.setFps(self.fps)
        self.camRight.setFps(self.fps)

        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        self.stereo.setExtendedDisparity(True)
        self.xoutDepth.setStreamName("depth")

        # 3. Explicit Linking (Output -> Input)
        self.camLeft.out.link(self.stereo.left)
        self.camRight.out.link(self.stereo.right)
        self.stereo.depth.link(self.xoutDepth.input)

    def colorizeDepth(self, frameDepth):
        invalidMask = frameDepth == 0
        try:
            # Re-using your exact logic for log-scaling
            minDepth = np.percentile(frameDepth[frameDepth != 0], 3)
            maxDepth = np.percentile(frameDepth[frameDepth != 0], 95)
            logDepth = np.zeros_like(frameDepth, dtype=np.float32)
            np.log(frameDepth, where=frameDepth != 0, out=logDepth)
            
            logMinDepth = np.log(minDepth)
            logMaxDepth = np.log(maxDepth)
            
            np.nan_to_num(logDepth, copy=False, nan=logMinDepth)
            logDepth = np.clip(logDepth, logMinDepth, logMaxDepth)

            depthFrameColor = np.interp(logDepth, (logMinDepth, logMaxDepth), (0, 255))
            depthFrameColor = depthFrameColor.astype(np.uint8)
            depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
            depthFrameColor[invalidMask] = 0
            return depthFrameColor
        except:
            return np.zeros((frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)

    def find_best_path(self, depth_frame):
        # 1. Downscale for faster processing on RPi
        search_w, search_h = 160, 100
        small_depth = cv2.resize(depth_frame, (search_w, search_h), interpolation=cv2.INTER_NEAREST)
        
        # 2. Define search window (robot size in pixels)
        window_size = 15 
        max_dist = 0
        best_x, best_y = search_w // 2, search_h // 2

        # 3. Sliding window search
        for y in range(0, search_h - window_size, 5):
            for x in range(0, search_w - window_size, 5):
                roi = small_depth[y : y + window_size, x : x + window_size]
                
                valid_pixels = roi[roi > 0]
                if len(valid_pixels) < (window_size**2 * 0.7): # Require 70% valid data
                    continue
                
                # We want the "cleanest" far-away spot
                current_min = np.min(valid_pixels)
                if current_min > max_dist:
                    max_dist = current_min
                    best_x = x + window_size // 2
                    best_y = y + window_size // 2

        # 4. Convert back to 640x400 coordinates for drawing
        final_x = int(best_x * (640 / search_w))
        final_y = int(best_y * (400 / search_h))

        # 5. CALCULATE CONTROL VALUES
        # Distance in meters
        distance_m = max_dist / 1000.0
        
        # Angle in degrees (HFOV is 72 for standard OAK-D mono)
        hfov = 72.0
        angle_deg = ((final_x - 320) / 640.0) * hfov

        return (final_x, final_y), distance_m, angle_deg
    
    def run(self):
        # Start the device
        with dai.Device(self.pipeline) as device:
            # Get the output queue
            queue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            
            windowName = "Robot Navigation View"
            cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)

            print("Navigation Started. Press 'q' to exit.")

            while True:
                # 1. Get the depth packet
                inDepth = queue.get()
                self.fpsCounter.tick()
                
                # 2. Extract frame and colorize for the UI
                depthFrame = inDepth.getFrame() # millimeters
                displayFrame = self.colorizeDepth(depthFrame)

                # 3. Calculate Path (Target Point, Distance, Angle)
                # target_pt = (x, y) in pixels
                # dist = meters
                # angle = degrees (-36 to +36)
                target_pt, dist, angle = self.find_best_path(depthFrame)

                # 4. Draw UI Elements
                # Draw a crosshair in the center of the screen
                cv2.line(displayFrame, (320, 180), (320, 220), (255, 255, 255), 1)
                cv2.line(displayFrame, (300, 200), (340, 200), (255, 255, 255), 1)

                # Draw the Target Point (The "Valley")
                cv2.circle(displayFrame, target_pt, 15, (0, 255, 0), 2)
                cv2.line(displayFrame, (320, 400), target_pt, (0, 255, 0), 2) # Path line

                # 5. Overlay Telemetry Data
                cv2.rectangle(displayFrame, (5, 5), (280, 85), (0, 0, 0), -1) # Background box
                
                fps_text = f"FPS: {self.fpsCounter.getFps():.1f}"
                dist_text = f"DIST: {dist:.2f}m"
                angle_text = f"ANGLE: {angle:.1f} deg"
                
                # Determine steering instruction
                if abs(angle) < 5:
                    drive_msg = "FORWARD"
                    msg_color = (0, 255, 0) # Green
                elif angle > 0:
                    drive_msg = "TURN RIGHT"
                    msg_color = (0, 165, 255) # Orange
                else:
                    drive_msg = "TURN LEFT"
                    msg_color = (0, 165, 255) # Orange

                cv2.putText(displayFrame, fps_text, (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(displayFrame, dist_text, (15, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(displayFrame, angle_text, (15, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, msg_color, 2)
                
                # Action command at the bottom
                cv2.putText(displayFrame, drive_msg, (240, 380), cv2.FONT_HERSHEY_SIMPLEX, 1, msg_color, 3)

                # 6. Show the frame
                cv2.imshow(windowName, displayFrame)

                # Optional: Send to Serial here
                # self.send_to_arduino(dist, angle)

                if cv2.waitKey(1) == ord("q"):
                    break

            cv2.destroyAllWindows()

if __name__ == "__main__":
    app = StereoApp(fps=50.0)
    app.run()