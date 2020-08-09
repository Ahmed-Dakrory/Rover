import spidev
import RPi.GPIO as GPIO


class Spi(object):
    def __init__(self, SSN):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        self.SSN = SSN
        GPIO.setwarnings(False)    
        GPIO.setmode(GPIO.BCM)    
        GPIO.setup(self.SSN, GPIO.OUT)
        print("SSN %s " % self.SSN)
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz=1000000
        # Default to mode 0.
        self.spi.mode = 0
        self.spi.no_cs = True
        self.spi.lsbfirst = False


    def write(self,list_of_address):
        GPIO.output(self.SSN, GPIO.LOW)
        # self.spi.xfer2(list_of_address)
        self.spi.writebytes(list_of_address)
        GPIO.output(self.SSN, GPIO.HIGH)

    def read(self,list_from_address):
        GPIO.output(self.SSN, GPIO.LOW)
        # self.spi.xfer2(list_from_address)
        # data = self.spi.xfer2([0])
        data = self.spi.readbytes(3)
        GPIO.output(self.SSN, GPIO.HIGH)

        return data

# spi = Spi(17)

# for i in range(0,50):
#     print(spi.read([55]))
import serial
ser = serial.Serial(
        port='/dev/serial0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 115200,rtscts = 0
)
ser.flush()

except_counter = 0
for i in range(0,50):
    try:
        data = ser.readline()
        data = str(data)[2:-5]
        ser.write(bytearray([125,133,155]))
        if(len(data)>1):
            print(data)
    except serial.serialutil.SerialException:
        except_counter +=1
        if except_counter == 5:
            break
    

ser.close()