# Depth test program for OAK-D Lite
# Last author: Max Smith
#
# Tests the depth sensor

import depthai as dai

devices = dai.Device.getAllAvailableDevices()
print(devices)