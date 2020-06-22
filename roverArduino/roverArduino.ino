// Wire Slave Sender
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Sends data as an I2C/TWI slave device
// Refer to the "Wire Master Reader" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event

  Wire.onRequest(sendNumberOfLocations); // sendData event
}

void loop() {

}


long timeBefore = 0;

uint16_t SteeringAngle = 0;
uint16_t RobotSpeed = 0;
uint16_t BrakeValue = 0;

byte byteStruct[6];
void receiveEvent(int howMany) {
  
  timeBefore = micros();
  if (Wire.available())
  {
    for(int i=0;i<howMany;i++){
      byteStruct[i] = Wire.read();
      }
    
       SteeringAngle = (byteStruct[1] << 8) | byteStruct[2];
    
       RobotSpeed = (byteStruct[3] << 8) | byteStruct[4];
    
       BrakeValue = (byteStruct[5]);
    
  }

  
    Serial.print(micros() - timeBefore);
    Serial.print("\t");
    Serial.print(SteeringAngle);
    Serial.print("\t");
    Serial.print(RobotSpeed);
    Serial.print("\t");
    Serial.print(BrakeValue);
    Serial.print("\t");
    Serial.println(howMany);
    
  
  
}


void sendNumberOfLocations() {
  // device address is specified in datasheet
  Wire.write("51.9228757,4.4800224,51.9228757,4.4800224,51.9228757,4.4800224"); // respond with message of 6 bytes

  // as expected by master
}