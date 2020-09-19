import time
import math
from .MPU6050 import MPU6050
from .rm3100  import RM3100


# from MPU6050 import MPU6050

# from rm3100  import RM3100

import threading

def toAnotherRange(OldValue,OldMin,OldMax,NewMin,NewMax):
    NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
    return NewValue

class IMU (threading.Thread):
    def __init__(self, SSN, DRDY):
        threading.Thread.__init__(self)
        self.SSN = SSN
        self.DRDY = DRDY
        self.rm3100 = RM3100(self.SSN,self.DRDY)
        self.Readings = None
        self.Rates = None
        # Sensor initialization
        self.i2c_bus = 1
        self.device_address = 0x68
        # The offsets are different for each device and should be changed
        # accordingly using a calibration procedure
        self.x_accel_offset = -2500
        self.y_accel_offset = -1441
        self.z_accel_offset = 1305
        self.x_gyro_offset = -2
        self.y_gyro_offset = -72
        self.z_gyro_offset = -5
        self.enable_debug_output = False

        self.mpu = MPU6050(self.i2c_bus, self.device_address, self.x_accel_offset, self.y_accel_offset,
                    self.z_accel_offset, self.x_gyro_offset, self.y_gyro_offset, self.z_gyro_offset,
                    self.enable_debug_output)


        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        self.mpu_int_status = self.mpu.get_int_status()
        print(hex(self.mpu_int_status))

        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()
        self.FIFO_count = self.mpu.get_FIFO_count()

        self.count = 0
        self.FIFO_buffer = [0]*64

        self.FIFO_count_list = list()


        self.pitch = 0
        self.roll = 0
        self.ThetaYaw = 0
        self.Alfa = 0.8
        self.HeadingNew = 0


    def run(self):
        while True:
            try:
                self.FIFO_count = self.mpu.get_FIFO_count()
                self.mpu_int_status = self.mpu.get_int_status()

                # If overflow is detected by status or fifo count we want to reset
                if (self.FIFO_count == 1024) or (self.mpu_int_status & 0x10):
                    self.mpu.reset_FIFO()
                # Check if fifo data is ready
                elif (self.mpu_int_status & 0x02):
                    # Wait until packet_size number of bytes are ready for reading, default
                    # is 42 bytes
                    while self.FIFO_count < self.packet_size:
                        self.FIFO_count = self.mpu.get_FIFO_count()
                    self.FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
                    self.accel = self.mpu.DMP_get_acceleration_int16(self.FIFO_buffer)
                    self.quat = self.mpu.DMP_get_quaternion_int16(self.FIFO_buffer)
                    self.grav = self.mpu.DMP_get_gravity(self.quat)
                    self.roll_pitch_yaw = self.mpu.DMP_get_euler_roll_pitch_yaw(self.quat, self.grav)
                    
                    self.Rates = self.mpu.readGyro()


                    self.roll = self.roll_pitch_yaw.x*math.pi/180
                    self.pitch = self.roll_pitch_yaw.y*math.pi/180

                    self.roll = toAnotherRange(self.roll,0,230*math.pi/180,0,90*math.pi/180)
                    self.pitch = toAnotherRange(self.pitch,0,240*math.pi/180,0,90*math.pi/180)
                    yawIMU = -self.roll_pitch_yaw.z*37/15*30/27*20/22
                    

                    mag = self.rm3100.readMag()

                    if mag !=None:
                        
                    

                        # CMx = mag['x']*math.cos(self.pitch) + mag['y']*math.sin(self.roll)* math.sin(self.pitch) - mag['z']*math.cos(self.roll)*math.sin(self.pitch)
                        # CMy =  mag['y']*math.cos(self.roll)+ mag['z']*math.sin(self.roll)


                        CMx = mag['x']*math.cos(self.pitch) + mag['z']*math.sin(self.pitch)
                        CMy = mag['x']*math.sin(self.roll)*math.sin(self.pitch) + mag['y']*math.cos(self.roll)- mag['z']*math.sin(self.roll)*math.cos(self.pitch)

                        self.HeadingNew = -(math.atan2(CMy,CMx) * 180 / math.pi)
                        if self.HeadingNew > 180:
                            self.HeadingNew = self.HeadingNew - 360


                                                
                
                        self.ThetaYaw = self.Alfa * self.ThetaYaw + (1 - self.Alfa) * self.HeadingNew
                        self.Readings ={'Roll':self.roll * 180 /math.pi,'Pitch':self.pitch* 180 /math.pi,'Yaw':self.ThetaYaw }
                    else:
                        self.Readings = None
            except:
                pass           

