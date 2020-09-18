from imuDev.MpuRm3100 import IMU
import time

DRDY = 27 #GPIO 27
SSN = 17 #GPIO 17
imu = IMU(SSN,DRDY)

imu.start()

while True:
    time.sleep(0.01)
    if imu.Readings !=None:
        # pass
        # print("%0.2f  , %0.2f,  %0.2f" % (imu.Readings['Roll'],imu.Readings['Pitch'],imu.Readings['Yaw']))
        print(imu.Readings['Yaw'])

# from rm3100  import RM3100

# DRDY = 27 #GPIO 27
# SSN = 17 #GPIO 17
# rm3100 = RM3100(SSN,DRDY)
# while True:
#     print(rm3100.readMag())

