import depthai as dai
try:
    pipeline = dai.Pipeline()
    with dai.Device(pipeline) as device:
        print("Connected to OAK device successfully!")
        print(f"USB Speed: {device.getUsbSpeed()}")
except Exception as e:
    print(f"Connection failed: {e}")