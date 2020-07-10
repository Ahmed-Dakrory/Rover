# Rover
Python RM3100
=============

This project contains a python module for interfacing with SPI RM3100 integrated with MPU6050 from user space via the spidev linux kernel driver.

All code is MIT licensed unless explicitly stated otherwise.


Hints
-----
This project is depends on spidev library to make the SPI connection and 
you have to check the SPI connection as mentioned here before make the communication
https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md

Usage
-----


```Connection
The MPU6050 to RPI
SDA --> SDA
SCL --> SCL
VCC --> 3.3V
GND --> GND

The RM3100 to RPI
MOSI --> MOSI
MISO --> MISO
SSN --> GPIO PIN(17 for example)
DRDY --> GPIO PIN(27 for example)
VCC --> 3.3V
GND --> GND
SCK --> SCLK

The Teensy to RPI
18(A4) --> SDA
19(A5) --> SCL
GND --> GND

first make sure for the directions to be as shown


```

![alt text](https://raw.githubusercontent.com/Ahmed-Dakrory/RM3100_With_MPU6050/master/Directions.jpg)


## Setup and Running

``` 
``` 

First when you run this code
> python3 controller.py
 
it will load the map for the first time only which reduce the time which will be taken every time loading

then you will send the GPS target points as follow from the Teensy Serial

if you will send one point 
> lat,longG

if you will send two points
> lat,long;lat2,long2G


### Note: Not Maximum point than two points


the system will handle those points and check if they will be inside the map then it will run the routing if ok, else send a Brake value equal to 255

if the routing done, it will send different Values as follow to teensy
Time, Angle, Speed, Error, FixMode, NumberofBytes

you can Stop showing the Serial data by send D to Teensy
you can Stop Navigation by send B to Teensy
you can Run Navigation by send S to Teensy 

to get out of this mission and start a new mission just send F to Teensy, then send the GPS locations again and so on


the fix mode and the error is descriped in the Arduino files


``` 
``` 

# Have fun :)

