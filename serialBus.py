
import threading
import serial


from time import sleep
class serialBus (threading.Thread):
    def __init__(self):
        super(serialBus , self).__init__(name="SerialBus thread")
        self.DataExist = False
        self.Data = None
        self.serialBusRun = True
        self.ser = serial.Serial(
            port='/dev/serial0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
            baudrate = 115200,rtscts = 0
        )

    def run(self):
        while self.serialBusRun:
            try:
                data = self.ser.readline()
                if(len(data)>1):
                    self.Data = str(data)[2:-5]
                    print(self.Data)
                    self.DataExist = True
            except Exception as e:
                pass

    def SendDataOfType(self,data):
        try:
            self.ser.write(bytearray(data))
        except Exception as e:
            print(e)
