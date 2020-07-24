
import threading
from smbus2 import SMBus

from time import sleep
class smbusRead (threading.Thread):
    def __init__(self,bus,address):
        super(smbusRead , self).__init__(name="SMBUS thread")
        self.bus = bus
        self.address = address
        self.DataExist = False
        self.Data = None
        self.smbusReadRun = True

    def run(self):
        while self.smbusReadRun:
            try:
                block = self.bus.read_i2c_block_data(self.address,0,32)
                string = ""
                if block[0]!=0:
                    for i in range(0,32):
                        try:
                            if block[i]!=0:
                                string +=chr(block[i])
                        except:
                            pass
                    self.Data = string.strip()
                    print(self.Data)
                    self.DataExist = True
            except Exception as e:
                pass

    def SendDataOfType(self,address,data,bus):
        try:
            bus.write_i2c_block_data(address,255,data)
        except:
            pass
